//
//  tcp_stream.h
//  face_toolkit
//
//  Created by Shunsuke Saito on 9/5/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#pragma once

#include <memory>
#include <string>
#include <iomanip>
#include <iostream>
#include <queue>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

//#include <uv.h>

#include "tcp_client.h"

#define check_uv(status) \
    do { \
        int code = (status); \
        if(code < 0){ \
            fprintf(stderr, "%s: %s\n", uv_err_name(code), uv_strerror(code)); \
            exit(code); \
        } \
    } while(0)

struct Frame{
    bool eof;
    Frame() : eof(false){}
    virtual ~Frame() {}
};

struct LandmarkFrame : public Frame {
    std::vector<Eigen::Vector3f> lands;
};

struct ParamFrame : public Frame {
    std::vector<float> param;
    std::vector<std::pair<std::string, int>> dof;
};

struct ImageFrame : public Frame {
    cv::Mat image;
};

typedef std::shared_ptr<Frame> FramePtr;
typedef std::shared_ptr<ImageFrame> ImageFramePtr;
typedef std::shared_ptr<ParamFrame> ParamFramePtr;
typedef std::shared_ptr<LandmarkFrame> LandmarkFramePtr;

struct TCPStreamSync {
    std::shared_ptr<TCPClient> tcp_client;
    const size_t sf_size;
    const size_t rf_size;
    std::vector<char> send_buffer;
    std::vector<char> recv_buffer;
    std::queue<FramePtr> result;
    
    std::string address;
    int port;
    
    TCPStreamSync(std::string _address, int _port, size_t send_frame_size, size_t recv_frame_size)
    : sf_size(send_frame_size), rf_size(recv_frame_size), address(_address), port(_port) {
        tcp_client = std::make_shared<TCPClient>(_address, _port);
        recv_buffer.resize(rf_size);
        tcp_client->Connect(10);
    }
    
    ~TCPStreamSync() {
        std::cout << "tcp stream destructed" << std::endl;
    }
    
    void write(std::vector<char> buffer) {
        if (buffer.empty()) {
            return;
        }
        else {
            send_buffer = buffer;
            int n_try = 0;
            while(!tcp_client->Send(&send_buffer[0], sf_size)){
                tcp_client = std::make_shared<TCPClient>(address, port);
                tcp_client->Connect(10);
                if(++n_try > 10){
                    throw std::runtime_error("tcp connection cannot be established...");
                }
            }
        }
    }
    
    virtual void setInput(FramePtr frame) {
        write(encode(frame));
    }
    
    virtual FramePtr getOutput(int wait_time = 1000) {
        int n_try = 0;
        while(!tcp_client->RcvImage(&recv_buffer[0], rf_size)){
            tcp_client = std::make_shared<TCPClient>(address, port);
            tcp_client->Connect(10);
            write(send_buffer);
            if(++n_try > 10){
                throw std::runtime_error("tcp connection cannot be established...");
            }
        }
        return decode(recv_buffer);
    }

    /// stream design end
    
    /// export interface begin
    virtual std::vector<char> encode(FramePtr frame) = 0;
    virtual FramePtr decode(std::vector<char> buffer) = 0;
};
//
//struct TCPStream {
//    /// LibUV design begin
//    const size_t sf_size;
//    const size_t rf_size;
//    uv_connect_t connection;
//    uv_tcp_t socket;
//    uv_stream_t* uv_stream = nullptr;
//    std::vector<char> send_buffer;
//    std::vector<char> recv_buffer;
//    std::queue<FramePtr> result;
//    char recv_end;
//    size_t cur_idx = 0;
//
//    uv_getaddrinfo_t resolver;
//    uv_write_t request;
//    std::string address;
//    int port;
//
//    TCPStream(std::string _address, int _port, size_t send_frame_size, size_t recv_frame_size)
//    : sf_size(send_frame_size), rf_size(recv_frame_size), address(_address), port(_port) {
//        recv_buffer.resize(rf_size);
//
//        auto loop = uv_default_loop();
//
//        check_uv(uv_tcp_init(loop, &socket));
//        check_uv(uv_tcp_keepalive(&socket, 1, 60));
//
//        socket.data = this;
//        resolver.data = this;
//
//        struct addrinfo hints; // IPv4 TCP
//        hints.ai_family = AF_INET;
//        hints.ai_socktype = SOCK_STREAM;
//        hints.ai_protocol = IPPROTO_TCP;
//        hints.ai_flags = 0;
//
//        check_uv(uv_getaddrinfo(loop, &resolver, on_resolved, address.c_str(), std::to_string(port).c_str(), &hints));
//    }
//
//    ~TCPStream() {
//        std::cout << "tcp stream destructed" << std::endl;
//    }
//
//    static TCPStream* get_stream(uv_handle_t* handle) {
//        return static_cast<TCPStream*>(handle->data);
//    }
//
//    static void on_resolved(uv_getaddrinfo_t *resolver, int status, struct addrinfo *res) {
//        if (status) {
//            fprintf(stderr, "uv_getaddrinfo error: %s\n", uv_strerror(status));
//            return;
//        }
//        TCPStream* stream = static_cast<TCPStream*>(resolver->data);
//        stream->connect(res->ai_addr);
//        uv_freeaddrinfo(res);
//    }
//
//    static void on_connect(uv_connect_t* connection, int status) {
//        TCPStream* stream = static_cast<TCPStream*>(connection->handle->data);
//        if (status) {
//            fprintf(stderr, "%s:%d - on_connect error: %s\n", stream->address.c_str(), stream->port, uv_strerror(status));
//            return;
//        }
//        stream->uv_stream = connection->handle;
//        stream->try_send();
//    }
//
//    static void on_write(uv_write_t* req, int status)
//    {
//        //std::cout << "write done" << std::endl;
//        if (status) {
//            TCPStream* stream = static_cast<TCPStream*>(req->handle->data);
//            fprintf(stderr, "%s:%d - on_write error: %s\n", stream->address.c_str(), stream->port, uv_strerror(status));
//            return;
//        }
//    }
//
//    static void alloc_cb(uv_handle_t* handle, size_t suggested_size, uv_buf_t* buf) {
//        TCPStream* stream = get_stream(handle);
//        stream->get_avail_buffer(buf);
//    }
//
//    static void on_read(uv_stream_t* handle, ssize_t nread, const uv_buf_t* buf) {
//        TCPStream* stream = get_stream((uv_handle_t*)handle);
//
//        if(nread >= 0) {
//            stream->try_read(nread);
//        }
//        else {
//            if (nread != UV_EOF) {
//                fprintf(stderr, "uv_read_cb error: %s\n", uv_strerror(nread));
//            }
//            uv_close((uv_handle_t*)handle, on_close);
//        }
//    }
//
//    static void on_close(uv_handle_t* handle)
//    {
//        // tcp resource released, safe to close stream -> destruct
//        //printf("closed.\n");
//        TCPStream* stream = get_stream(handle);
//        stream->actual_close();
//    }
//
//    virtual void connect(struct sockaddr* addr) {
//        check_uv(uv_tcp_connect(&connection, &socket, addr, on_connect));
//    }
//
//    virtual void try_send() {
//        if (uv_stream == nullptr || send_buffer.empty())
//            return;
//
//        std::cout << "send buffer" << std::endl;
//        uv_buf_t buffer = {.base = send_buffer.data(), .len = send_buffer.size()};
//
//        uv_write(&request, uv_stream, &buffer, 1, on_write);
//        uv_read_start(uv_stream, alloc_cb, on_read);
//    }
//
//    virtual void try_read(ssize_t nread) {
//
//        cur_idx += nread;
//
//        std::cout << "get " << cur_idx << " bytes" << std::endl;
//        if (cur_idx == recv_buffer.size()) {
//            FramePtr frame = decode(recv_buffer);
//            result.push(frame);
//            cur_idx = 0;
//        }
//    }
//
//    virtual void get_avail_buffer(uv_buf_t* buf) {
//        buf->base = recv_buffer.data() + cur_idx;
//        buf->len = recv_buffer.size() - cur_idx;
//        if (buf->len == 0) {
//            buf->base = &recv_end;
//            buf->len = 1;
//        }
//    }
//
//    void write(std::vector<char> buffer) {
//        if (buffer.empty()) {
//            return;
//        }
//        else {
//            send_buffer = buffer;
//            try_send();
//        }
//    }
//    /// LibUV design end
//
//    /// stream design begin
//    virtual void setInput(FramePtr frame) {
//        write(encode(frame));
//    }
//
//    virtual FramePtr getOutput(int wait_time = 1000) {
//        auto wait = [](std::queue<FramePtr>& q, int ms) {
//            clock_t endwait;
//            endwait = clock () + ms * CLOCKS_PER_SEC  / 1000 ;
//            while (clock() < endwait) {
//                if(q.size() != 0) return true;
//            }
//            return false;};
//        if(!wait(result,wait_time)) return nullptr;
//        auto p = result.front();
//        result.pop();
//        return p;
//    }
//
//    virtual void close() {
//        uv_close((uv_handle_t*)&socket, on_close);
//    }
//
//    void actual_close() {
//
//    }
//    /// stream design end
//
//    /// export interface begin
//    virtual std::vector<char> encode(FramePtr frame) = 0;
//    virtual FramePtr decode(std::vector<char> buffer) = 0;
//
//    /// export interface end
//};

