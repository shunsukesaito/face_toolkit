cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
PinscreenModuleBegin(minitrace)

add_library(minitrace minitrace.c minitrace.h)

target_include_directories(minitrace INTERFACE .)
PinscreenModuleEnd(minitrace)