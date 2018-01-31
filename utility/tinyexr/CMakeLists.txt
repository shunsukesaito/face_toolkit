cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
PinscreenModuleBegin(tinyexr)

add_library(tinyexr tinyexr.cc tinyexr.h)

target_include_directories(tinyexr INTERFACE .)
PinscreenModuleEnd(tinyexr)