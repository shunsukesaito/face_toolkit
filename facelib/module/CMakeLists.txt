cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
FaceToolkitModuleBegin(module DEPENDS 
                    library/Eigen3 
                    library/OpenCV
                    library/gflags
                    utility/minitrace 
                    utility/face2d-detector
                    utility/tcpstream
                    utility/SPSCQueue
                    facelib/utility 
                    facelib/gl_utility 
                    facelib/shape_model
                    facelib/optimizer
                    facelib/renderer)

file(GLOB_RECURSE all_sources *.cpp)
file(GLOB_RECURSE all_headers *.h)
add_library(module ${all_sources} ${all_headers})

target_link_libraries(module FTM::OpenCV FTM::Eigen3 FTM::gflags FTM::minitrace FTM::face2d-detector FTM::tcpstream FTM::SPSCQueue FTM::utility FTM::gl_utility FTM::shape_model FTM::optimizer FTM::renderer)

if(WITH_IMGUI)
    target_link_libraries(module FTM::imgui)
    target_compile_definitions(module PUBLIC "WITH_IMGUI")
endif()

target_include_directories(module PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<INSTALL_INTERFACE:include/module>)
install(FILES ${all_headers} DESTINATION include/module)
FaceToolkitModuleEnd(module)
