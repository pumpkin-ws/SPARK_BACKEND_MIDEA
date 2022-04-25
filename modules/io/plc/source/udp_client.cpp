#include <udp_client.hpp>

UDPClient::UDPClient(int port_num, std::string ip) 
: IP_ADDR(ip), SERVER_PORT(port_num) {
    this->m_sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(m_sock_fd < 0) {
        perror("socket");
        exit(1);
    }
    // set the socket's internet address
    memset(&m_addr_serv, 0, sizeof(m_addr_serv)); // set all member of the internet socket address to 0s
    this->m_addr_serv.sin_family = AF_INET; // IP4 protocal
    this->m_addr_serv.sin_addr.s_addr = inet_addr(IP_ADDR.c_str());
    m_addr_serv.sin_port = htons(SERVER_PORT);
    // connect to the server
    int conn_status = connect(m_sock_fd, (struct sockaddr*)&m_addr_serv, sizeof(m_addr_serv));
    if (conn_status == 0) {
        printf("Connection successful!\n");
    } else if (conn_status == -1) {
        printf("Connection failed!\n");
        exit(-1);
    } else {
        printf("Unknown output.");
        exit(-1);
    }
};

/**
 * @brief Inherited classes should rewrite this function to match the protocals provided by the specific devices. Msg should be the message protocals 
 * 
 * @param msg 
 * @return std::string the return message from the server
 */
bool UDPClient::sendRequestAndGetResponse(std::string msg) {
    int send_num;
    int recv_num;

    char recv_buf[200] = {0};
    char send_buf[200] = {0};

    uint8_t cmd[20] = {0};
    cmd[0] = 0x80;
    cmd[1] = 0x00;
    cmd[2] = 0x02;
    cmd[3] = 0x00;
    cmd[4] = 0x01;
    cmd[5] = 0x00;
    cmd[6] = 0x00;
    cmd[7] = 0x02;
    cmd[8] = 0x00;
    cmd[9] = 0x00;
    cmd[10] = 0x01;
    cmd[11] = 0x02;
    cmd[12] = 0xB0;
    cmd[13] = 0x00;
    cmd[14] = 0x65;
    cmd[15] = 0x00;
    cmd[16] = 0x00;
    cmd[17] = 0x01;
    cmd[18] = 0x00;
    cmd[19] = 0x00;

    int len = sizeof(m_addr_serv);

    send_num = sendto(m_sock_fd, cmd, sizeof(cmd), 0, (struct sockaddr*)&m_addr_serv, len);
    // printf("The send number is %d.\n", send_num);
    if(send_num < 0) {
        perror("sendto error");
        exit(1);
    }

    // printf("Waiting for received signal\n");
    char rcv[20] = {0};
    
    recv_num = recvfrom(m_sock_fd, rcv, sizeof(rcv), 0, (struct sockaddr*)&m_addr_serv, (socklen_t *)&len);

    if(recv_num < 0) {
        perror("recvfrom error");
        exit(1);
    }
    recv_buf[recv_num] = '\0'; // append a null character to the received message
    for(int i =0; i < 20; i++) {
        std::cout << std::hex <<  int(rcv[i]);
    }
    return true;
};


UDPClient::~UDPClient() {
    close(m_sock_fd);
}