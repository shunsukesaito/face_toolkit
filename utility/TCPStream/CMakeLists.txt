cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
PinscreenModuleBegin(tcpstream DEPENDS
                     library/LibUV)

add_library(tcpstream tcp_stream.cc tcp_stream.h)

target_link_libraries(tcpstream PSM::LibUV)
target_include_directories(tcpstream INTERFACE .)
PinscreenModuleEnd(tcpstream)