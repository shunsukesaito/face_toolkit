cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
PinscreenModuleBegin(module DEPENDS 
                    library/Eigen3 
                    library/OpenCV
                    library/gflags
                    utility/minitrace 
                    utility/face2d-detector
                    utility/SPSCQueue
                    facelib/utility 
                    facelib/gl_utility 
                    facelib/shape_model
                    facelib/optimizer
                    facelib/renderer
                    facelib/f2f)

file(GLOB_RECURSE all_sources *.cpp)
file(GLOB_RECURSE all_headers *.h)
add_library(module ${all_sources} ${all_headers})

target_link_libraries(module PSM::OpenCV PSM::Eigen3 PSM::gflags PSM::minitrace PSM::face2d-detector PSM::SPSCQueue PSM::utility PSM::gl_utility PSM::shape_model PSM::optimizer PSM::renderer PSM::f2f)

if(WITH_IMGUI)
    target_link_libraries(module PSM::imgui)
    target_compile_definitions(module PUBLIC "WITH_IMGUI")
endif()

target_include_directories(module PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<INSTALL_INTERFACE:include/module>)
install(FILES ${all_headers} DESTINATION include/module)
PinscreenModuleEnd(module)
