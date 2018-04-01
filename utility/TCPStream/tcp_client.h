//
//  sender.h
//  test_socket
//
//  Created by SaitoShunsuke on 1/11/16.
//  Copyright © 2016 SaitoShunsuke. All rights reserved.
//

#ifndef sender_h
#define sender_h

#include <memory>
#include <iostream>

#include <sys/types.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>	/* for fprintf */
#include <string.h>	/* for memcpy */
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <time.h>

class TCPClient
{
public:
    TCPClient(const std::string &address, const short port = 0);
    ~TCPClient(){close(fd_);}

    bool Connect(clock_t max_wait = 10);
    
    bool Send(void *data, size_t buffer_size);
    
    bool RcvImage(char* data, size_t buffer_size);
    
    void Close();
    
private:
    int fd_;
    struct sockaddr_in myaddr_;
    struct sockaddr_in* servaddr_;
    std::string address_;
    short port_;
};

typedef std::shared_ptr<TCPClient> TCPClientPtr;


#endif /* sender_h */
