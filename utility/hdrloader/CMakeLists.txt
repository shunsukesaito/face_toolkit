cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
PinscreenModuleBegin(hdrloader)

add_library(hdrloader hdrloader.cc hdrloader.h)

target_include_directories(hdrloader INTERFACE .)
PinscreenModuleEnd(hdrloader)