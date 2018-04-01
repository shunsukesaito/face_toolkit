#include "tcp_client.h"

TCPClient::TCPClient(const std::string &address, const short port)
{
    address_ = address;
    port_ = port;
    
    // create socket first
    std::cout << "Creating Socket..." << std::endl;
    if ((fd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("cannot create socket");
    }
    
    // bind socket
    std::cout << "Bindling Socket..." << std::endl;
    memset((char *)&myaddr_, 0, sizeof(myaddr_));
    myaddr_.sin_family = AF_INET;
    myaddr_.sin_addr.s_addr = htonl(INADDR_ANY);
    myaddr_.sin_port = 0;
    if (::bind(fd_, (struct sockaddr *)&myaddr_, sizeof(myaddr_)) < 0) {
        perror("bind failed");
    }
    
    /* fill in the server's address and data */
    std::cout << "Setting Host..." << std::endl;
    struct addrinfo hint = {0};

    hint.ai_flags = AI_NUMERICHOST;
    hint.ai_family = AF_UNSPEC;
    hint.ai_socktype = SOCK_STREAM;
    hint.ai_protocol = IPPROTO_TCP;
    struct addrinfo *addrs = NULL;
    int ret = getaddrinfo(address.c_str(), NULL, &hint, &addrs);
    if (ret == EAI_NONAME)
    {
        hint.ai_flags = 0;
        ret = getaddrinfo(address.c_str(), NULL, &hint, &addrs);
    }
    
    if (ret != 0)
    {
        std::cout << "Could not resolve inetAddr: " << ret << std::endl;
        return;
    }
    servaddr_ = (struct sockaddr_in *)addrs->ai_addr;
    servaddr_->sin_port = htons(port);
    
    char abuf[INET_ADDRSTRLEN];
    const char* addr = inet_ntop(AF_INET, &servaddr_->sin_addr, abuf,
                     INET_ADDRSTRLEN);
    printf("address:\t%s\n", addr?addr:"unknown"), printf("\t");
    printf("\n port:\t%d\n", ntohs(servaddr_->sin_port)), printf("\t");

//    memset((char*)&servaddr_, 0, sizeof(servaddr_));
//    servaddr_.sin_family = AF_INET;
//    servaddr_.sin_addr.s_addr = inet_addr(address.c_str());
//    servaddr_.sin_port = htons(port);
}

bool TCPClient::Connect(clock_t max_wait)
{
    std::cout << "Connecting to " << address_ << " Port: " << port_ << "..." << std::endl;
    clock_t start_ = clock();
    while (connect(fd_,(struct sockaddr *) servaddr_,sizeof(*servaddr_)) < 0){
        if(clock()-start_ > CLOCKS_PER_SEC*max_wait){
            std::cout << "Error: Client Session Timeout" << std::endl;
            return false;
        }
    }
    
    std::cout << "Connection Established..." << std::endl;
    return true;
}
    
bool TCPClient::Send(void *data, size_t buffer_size)
{
    // send data
    int n = write(fd_,data,buffer_size);
    if (n < 0){
        perror("send failed");
        return false;
    }
    //std::cout << "write done" << std::endl;
    return true;
}
    
bool TCPClient::RcvImage(char* data, size_t buffer_size)
{
    int rcvd_size = 0;
    
    while(rcvd_size < buffer_size){
        int n = read(fd_,data+rcvd_size,buffer_size-rcvd_size);
        //std::cout << "get " << n << " bytes" << std::endl;
        if (n < 0){
            if( errno == EAGAIN){
                continue;
            }

            perror("ERROR reading from socket");
            return false;
        }
        
        rcvd_size += n;
    }
    return true;
}

void TCPClient::Close()
{
    std::cout << "Socket is closing..." << std::endl; close(fd_);
}
