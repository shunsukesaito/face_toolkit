//
//  sender.h
//  test_socket
//
//  Created by SaitoShunsuke on 1/11/16.
//  Copyright © 2016 SaitoShunsuke. All rights reserved.
//

#ifndef sender_h
#define sender_h

#include <iostream>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>      /* for fprintf */
#include <string.h>     /* for memcpy */
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <pthread.h>
#include <time.h>

#include <time.h>

class TCPServer
{
public:
    TCPServer(const std::string &address, const short port = 0);
    ~TCPServer(){close(newfd_); close(fd_);}
    
    bool Wait(clock_t max_wait = 10);
    bool Send(void *data, size_t buffer_size);
    bool RcvImage(unsigned char* data, size_t buffer_size);
    
    void Close();
    
private:
    int fd_, newfd_;
    clock_t start_idle_ = 0.0;
    struct sockaddr_in cliaddr_;
    struct sockaddr_in servaddr_;
    socklen_t clilen_;
    short port_;
};



#endif /* sender_h */
