cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
FaceToolkitModuleBegin(tinyexr)

add_library(tinyexr tinyexr.cc tinyexr.h)

target_include_directories(tinyexr INTERFACE .)
FaceToolkitModuleEnd(tinyexr)