cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
FaceToolkitModuleBegin(tiny_obj_loader)

add_library(tiny_obj_loader tiny_obj_loader.cc tiny_obj_loader.h)

target_include_directories(tiny_obj_loader INTERFACE .)
FaceToolkitModuleEnd(tiny_obj_loader)