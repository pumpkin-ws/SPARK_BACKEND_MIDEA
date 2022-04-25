#ifndef UDP_CLIENT_HPP_
#define UDP_CLIENT_HPP_

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <arpa/inet.h>

class UDPClient {
public:
    explicit UDPClient(int port_num, std::string ip);
    /**
     * @brief Send request and receive response from the server
     * 
     * @param msg 
     * @return std::string the return message
     */
    virtual bool sendRequestAndGetResponse(std::string msg);
    virtual int getIO(unsigned short addr, int pin_num) = 0;
    // virtual bool getDO(int pin_num) = 0;
    virtual bool setDO(unsigned short addr, int pin_num, int do_value) = 0;
    // virtual bool getDM(int pin_num) = 0;
    // virtual bool setDM(int pin_num, bool do_value) = 0;
    ~UDPClient();
protected:
    std::string IP_ADDR;
    unsigned int SERVER_PORT;
    int m_sock_fd;
    struct sockaddr_in m_addr_serv; // structure describing an internet socket address
private:
    UDPClient(const UDPClient&) = delete; // delete the copy constructor
    UDPClient& operator=(const UDPClient&) = delete; // delete the assignment operator
};

#endif