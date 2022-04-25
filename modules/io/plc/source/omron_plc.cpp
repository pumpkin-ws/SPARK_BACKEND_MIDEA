#include "omron_plc.hpp"
#include <math.h>


int OmronPLC::getDoMemoryData(unsigned short addr) {
    FinsMemoryCmd_S read_memory_cmd;
    FinsResponse_S resp;

    read_memory_cmd.hdr.icf = 0x80;
    read_memory_cmd.hdr.rsv = 0x00;
    read_memory_cmd.hdr.gct = 0x02;
    read_memory_cmd.hdr.dna = 0x00;
    read_memory_cmd.hdr.da1 = 0x01;
    read_memory_cmd.hdr.da2 = 0x00;
    read_memory_cmd.hdr.sna = 0x00;
    //read_memory_cmd.hdr.sa1 = 0x63;
    read_memory_cmd.hdr.sa1 = 0x02;
    read_memory_cmd.hdr.sa2 = 0x00;
    read_memory_cmd.hdr.sid = 0x00;
    read_memory_cmd.hdr.mrc = 0x01;
    read_memory_cmd.hdr.src = 0x01;
    switch (addr) {
        case 0:
        case 1:
        case 2:
        case 100:
        case 101:
        case 102:
           read_memory_cmd.area = 0xB0;
            break;
        default:
            // printf("OmronPLC::getIO ERROR: there is no address %d\n", addr);
            return -1;
    }
    read_memory_cmd.addr = htons(addr);
    read_memory_cmd.bit = 0x00;
    read_memory_cmd.num = htons(1);

    int len = sizeof(FinsMemoryCmd_S);
    len = sendReceiv(m_sock_fd, &read_memory_cmd, len, &resp, sizeof(resp));
    int result = ntohs(*((uint16_t *)resp.data));
    return result;
}

int OmronPLC::getIO(unsigned short addr, int pin_num) {
    if (pin_num < 0 || pin_num > 11) {
        // printf("OmronPLC::getIO ERROR: pin_num error, it must be [0, 7]\n");
        return -1;
    }
    
    int result = getDoMemoryData(addr);

    int target = result >> pin_num;
    // printf("OmronPLC::getIO target is %d\n", target);

    return (target & 1);

}

int OmronPLC::sendReceiv(int plc_id, void *send, int receive_len , void *receiv, int rcv_len) {
    char client_ko;
    struct sockaddr_in ip;
    socklen_t ip_len;

    FinsResponse_S *resp = (FinsResponse_S*)receiv;

    ip_len = sizeof(struct sockaddr_in);
    FinsMemoryCmd_S *send_cmd = (FinsMemoryCmd_S*)send;
    send_cmd->hdr.sid = 2;

    if (sendto(m_sock_fd, send, receive_len, 0, (struct sockaddr*)&m_addr_serv, sizeof(m_addr_serv)) != receive_len) {
        // printf("OmronPLC::sendReceiv ERROR: send error\n");
        return -1;
    }

    int continue_receive = 0;
    do {
        continue_receive = 0;
        if ((receive_len = recvfrom(m_sock_fd, receiv, rcv_len, 0, (struct sockaddr*)&m_addr_serv, &ip_len)) < 0) {
            // printf("OmronPLC::sendReceiv ERROR: recvfrom error\n");
            return -1;
        }
        if (send_cmd->hdr.sid != resp->hdr.sid) {
            continue_receive = 1;
            ip_len = sizeof(struct sockaddr_in);
        }
    } while (continue_receive);

    // printf("revc data length is %d\n", receive_len);

    if (receive_len < 14 || resp->mres != 0 || resp->sres != 0) {
        if (receive_len < 14) {
            // printf("FINS length error\n");
        } else {
            // printf("Response error\n");
        }
        return -1;
    }
    return 0;
}

int OmronPLC::getDM(unsigned short addr) {
    FinsMemoryCmd_S read_memory_cmd;
    FinsResponse_S resp;

    read_memory_cmd.hdr.icf = 0x80;
    read_memory_cmd.hdr.rsv = 0x00;
    read_memory_cmd.hdr.gct = 0x02;
    read_memory_cmd.hdr.dna = 0x00;
    read_memory_cmd.hdr.da1 = 0x01;
    read_memory_cmd.hdr.da2 = 0x00;
    read_memory_cmd.hdr.sna = 0x00;
    read_memory_cmd.hdr.sa1 = 0x02;
    read_memory_cmd.hdr.sa2 = 0x00;
    read_memory_cmd.hdr.sid = 0x00;
    read_memory_cmd.hdr.mrc = 0x01;
    read_memory_cmd.hdr.src = 0x01;
    read_memory_cmd.area = 0x82;
    read_memory_cmd.addr = htons(addr);
    read_memory_cmd.bit = 0x00;
    read_memory_cmd.num = htons(1);
    int len = sizeof(FinsMemoryCmd_S);
    len = sendReceiv(m_sock_fd, &read_memory_cmd, len, &resp, sizeof(resp));
    int result = ntohs(*((uint16_t *)resp.data));
    return result;
}

bool OmronPLC::setDO(unsigned short addr, int pin_num, int do_value) {
    if (pin_num < 0 || pin_num > 7) {
        printf("OmronPLC::getIO ERROR: pin_num error, it must be [0, 7]\n");
        return -1;
    }
    if ((do_value != 0) && (do_value != 1)) {
        printf("OmronPLC::getIO ERROR: do value error, it must be [0, 1]\n");
        return -1;
    }
    FinsMemoryCmdWrite_S write_memory_cmd;
    FinsResponse_S resp;

    write_memory_cmd.hdr.icf = 0x80;
    write_memory_cmd.hdr.rsv = 0x00;
    write_memory_cmd.hdr.gct = 0x02;
    write_memory_cmd.hdr.dna = 0x00;
    write_memory_cmd.hdr.da1 = 0x01;
    write_memory_cmd.hdr.da2 = 0x00;
    write_memory_cmd.hdr.sna = 0x00;
    write_memory_cmd.hdr.sa1 = 0x02;
    write_memory_cmd.hdr.sa2 = 0x00;
    write_memory_cmd.hdr.sid = 0x00;
    write_memory_cmd.hdr.mrc = 0x01;
    write_memory_cmd.hdr.src = 0x02;
    switch (addr) {
        case 100:
        case 101:
        case 102:
           write_memory_cmd.area = 0xB0;
            break;
        default:
            printf("OmronPLC::getIO ERROR: there is no address %d\n", addr);
            return -1;
    }
    write_memory_cmd.addr = htons(addr);
    write_memory_cmd.bit = 0x00;
    write_memory_cmd.num = htons(1);
    int value = 1 << pin_num;
    int memory_value = getDoMemoryData(addr);
    (do_value == 1) ? (memory_value += value) : (memory_value -= value);
    short *data = (short *)&memory_value;
    write_memory_cmd.data[0] = htons(data[0]);
    write_memory_cmd.data[1] = htons(data[1]);

    int len = sizeof(FinsMemoryCmd_S) + 2;
    while (getIO(addr, pin_num) != do_value) {
        sendReceiv(m_sock_fd, &write_memory_cmd, len, &resp, sizeof(resp));
    }
    return 0;
}

int OmronPLC::setDM(unsigned short addr, int dm_value) {
    FinsMemoryCmdWrite_S write_memory_cmd;
    FinsResponse_S resp;

    write_memory_cmd.hdr.icf = 0x80;
    write_memory_cmd.hdr.rsv = 0x00;
    write_memory_cmd.hdr.gct = 0x02;
    write_memory_cmd.hdr.dna = 0x00;
    write_memory_cmd.hdr.da1 = 0x01;
    write_memory_cmd.hdr.da2 = 0x00;
    write_memory_cmd.hdr.sna = 0x00;
    write_memory_cmd.hdr.sa1 = 0x02;
    write_memory_cmd.hdr.sa2 = 0x00;
    write_memory_cmd.hdr.sid = 0x00;
    write_memory_cmd.hdr.mrc = 0x01;
    write_memory_cmd.hdr.src = 0x02;
    write_memory_cmd.area = 0x82;
    write_memory_cmd.addr = htons(addr);
    write_memory_cmd.bit = 0x00;
    write_memory_cmd.num = htons(1);
    short *data = (short *)&dm_value;
    write_memory_cmd.data[0] = htons(data[0]);
    write_memory_cmd.data[1] = htons(data[1]);

    int len = sizeof(FinsMemoryCmd_S) + 2;
    while (getDM(addr) != dm_value) {
        sendReceiv(m_sock_fd, &write_memory_cmd, len, &resp, sizeof(resp));
    }
    return 0;
}

void OmronPLC::getIOArray(unsigned int addr, std::vector<bool>& io_array) {
    int result = getDoMemoryData(addr);
    io_array.clear();
    if ((addr == 100) || (addr == 101) || (addr == 102)) {
        for (int i = 0; i < 8; i++) {
            int target = result >> i;
            io_array.push_back(target & 1);
        }
        return;
    } else if ((addr == 0) || (addr == 1) || (addr == 2)) {
        for (int i = 0; i < 12; i++) {
            int target = result >> i;
            io_array.push_back(target & 1);
        }
        return;
    }
    return;
}