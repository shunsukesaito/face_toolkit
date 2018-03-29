cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
PinscreenModuleBegin(tcpstream DEPENDS
                     library/OpenCV 
                     library/LibUV
                     library/Eigen3)

file(GLOB_RECURSE all_sources *.cc)
file(GLOB_RECURSE all_headers *.h)
add_library(tcpstream ${all_sources} ${all_headers})

target_link_libraries(tcpstream PSM::LibUV PSM::OpenCV PSM::Eigen3)
target_include_directories(tcpstream INTERFACE .)
PinscreenModuleEnd(tcpstream)