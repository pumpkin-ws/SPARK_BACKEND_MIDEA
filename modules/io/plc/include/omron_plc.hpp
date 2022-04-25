#ifndef Omron_PLC_HPP_
#define Omron_PLC_HPP_

#include "udp_client.hpp"
#include <vector>

#pragma pack(push)
#pragma pack(1)
typedef struct FinsHeader {
    unsigned char icf;
    unsigned char rsv;
    unsigned char gct;
    unsigned char dna;
    unsigned char da1;
    unsigned char da2;
    unsigned char sna;
    unsigned char sa1;
    unsigned char sa2;
    unsigned char sid;
    /* command */
    unsigned char mrc;
    unsigned char src;
} FinsHeader_S;

typedef struct FinsResponse {
    FinsHeader_S hdr;
    /* responce */
    unsigned char mres;   /* main response code */
    unsigned char sres;   /* sub-response code */
    
    unsigned char data[6];
} FinsResponse_S;

typedef struct FinsMemoryCmd {
    FinsHeader_S hdr;
    unsigned char area;     /* memory area code */
    unsigned short addr;    /* address */
    unsigned char bit;      /* bit */
    unsigned short num;     /* no. of items */
} FinsMemoryCmd_S;

typedef struct FinsMemoryCmdWrite {
    FinsHeader_S hdr;
    unsigned char area;     /* memory area code */
    unsigned short addr;    /* address */
    unsigned char bit;      /* bit */
    unsigned short num;     /* no. of items */
    unsigned short data[2];
} FinsMemoryCmdWrite_S;

#pragma pack(pop)

class OmronPLC : public UDPClient {
public:
    explicit OmronPLC(int port_num, std::string ip) : UDPClient(port_num, ip) {};
    /**
     * @brief Send request and receive response from the server
     * 
     * @param msg 
     * @return std::string the return message
     */
    virtual int getIO(unsigned short addr, int pin_num);
    virtual bool setDO(unsigned short addr, int pin_num, int do_value);
    int getDM(unsigned short addr);
    int setDM(unsigned short addr, int dm_value);
    /**
     * @brief addr of 0, 1, 2 are DIs, addr of 100, 101, 102 are DOs
     *  io_array represents result, if addr is 0 or 1 or 2, io_array range [0, 11] - 12 pins
     *  if addr is 100 or 101 or 102, io_array range [0, 7]
     * 
     * @param addr if DI, zone 0, 1, 2; if DO, zone 100, 101, 102
     * @param io_array true means DI or DO is activated, false means it is deactivated
     */
    void getIOArray(unsigned int addr, std::vector<bool>& io_array);
    ~OmronPLC(){};
protected:
    // save up for inheritence
private:
    int sendReceiv(int plc_id, void *send, int len , void *receiv, int rcv_len);
    int getDoMemoryData(unsigned short addr);
    OmronPLC(const OmronPLC&) = delete; // delete the copy constructor
    OmronPLC& operator=(const OmronPLC&) = delete; // delete the assignment operator
};

#endif