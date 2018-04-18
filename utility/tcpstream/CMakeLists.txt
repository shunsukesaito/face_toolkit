cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
FaceToolkitModuleBegin(tcpstream DEPENDS
                     library/OpenCV 
                     library/Eigen3)

file(GLOB_RECURSE all_sources *.cc)
file(GLOB_RECURSE all_headers *.h)
add_library(tcpstream ${all_sources} ${all_headers})

target_link_libraries(tcpstream FTM::OpenCV FTM::Eigen3)
target_include_directories(tcpstream INTERFACE .)
FaceToolkitModuleEnd(tcpstream)