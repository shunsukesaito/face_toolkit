cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
PinscreenModuleBegin(imgui DEPENDS
                    library/GLEW 
                    library/glfw3)

file(GLOB_RECURSE all_sources *.cpp)
file(GLOB_RECURSE all_headers *.h)

add_library(imgui ${all_sources} ${all_headers})

target_link_libraries(imgui PSM::GLEW PSM::glfw3)
target_include_directories(imgui INTERFACE .)
PinscreenModuleEnd(imgui)
