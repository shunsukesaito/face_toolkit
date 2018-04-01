#include "tcp_server.h"

TCPServer::TCPServer(const std::string &address, const short port)
{
    port_ = port;
    // create socket first
    std::cout << "Creating Socket..." << std::endl;
    if ((fd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("cannot create socket");
    }
    int enable = 1;
    if (setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
        perror("setsockopt(SO_REUSEADDR) failed");
    int flags;
#if defined(O_NONBLOCK)
    /* Fixme: O_NONBLOCK is defined but broken on SunOS 4.1.x and AIX 3.2.5. */
    if (-1 == (flags = fcntl(fd, F_GETFL, 0)))
        flags = 0;
   	fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
#else
    /* Otherwise, use the old way of doing it */
    flags = 1;
    ioctl(fd_, FIONBIO, &flags);
#endif
    /* fill in the server's address and data */
    std::cout << "Setting Host..." << std::endl;

    memset((char*)&servaddr_, 0, sizeof(servaddr_));
    servaddr_.sin_family = AF_INET;
    servaddr_.sin_addr.s_addr = inet_addr(address.c_str());
    servaddr_.sin_port = htons(port);
    
    if (::bind(fd_, (struct sockaddr *)&servaddr_, sizeof(servaddr_)) < 0) {
        perror("bind failed");
    }
    
    int true_val = 1;
    setsockopt(fd_,SOL_SOCKET,SO_REUSEADDR,&true_val,sizeof(int));
    setsockopt(newfd_,SOL_SOCKET,SO_REUSEADDR,&true_val,sizeof(int));

}

bool TCPServer::Wait(clock_t max_wait)
{
    listen(fd_,5);
    clilen_ = sizeof(cliaddr_);
    std::cout << "Wainting Port: " <<  port_ << "..." << std::endl;
    
    clock_t start_ = clock();
    do{
        newfd_ = accept(fd_, (struct sockaddr *) &cliaddr_, &clilen_);
        if(clock()-start_ > CLOCKS_PER_SEC*max_wait){
            std::cout << "Error: Client Session Timeout" << std::endl;
            return false;
        }
    }while(newfd_ < 0);
    
    std::cout << "Connection Established..." << std::endl;
    return true;
}
    
    
bool TCPServer::Send(void *data, size_t buffer_size)
{
    // send data
    int n = write(newfd_,data,buffer_size);
    if (n < 0){
        perror("send failed");
        return false;
    }
    return true;
}
    
bool TCPServer::RcvImage(unsigned char* data, size_t buffer_size)
{
    int rcvd_size = 0;
    
    start_idle_ = clock();
    while(rcvd_size < buffer_size){
        int n = read(newfd_,data+rcvd_size,buffer_size-rcvd_size);
        if (n < 0){
            perror("ERROR reading from socket");
            return false;
        }
        else if(clock() - start_idle_ > CLOCKS_PER_SEC*2){
            std::cout << "Disconnected..." << std::endl;
            return false;
        }
        
        rcvd_size += n;
    }
    return true;
}

void TCPServer::Close()
{
    std::cout << "Socket is closing..." << std::endl; close(newfd_); close(fd_);
}

