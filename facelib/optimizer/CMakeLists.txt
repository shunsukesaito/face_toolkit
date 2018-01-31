cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
PinscreenModuleBegin(optimizer DEPENDS 
                    library/Eigen3 
                    library/OpenCV
                    utility/minitrace 
                    utility/imgui
                    facelib/utility 
                    facelib/gl_utility 
                    facelib/shape_model)

file(GLOB_RECURSE all_sources *.cpp)
file(GLOB_RECURSE all_headers *.h)
add_library(optimizer ${all_sources} ${all_headers})

target_link_libraries(optimizer PSM::OpenCV PSM::Eigen3 PSM::minitrace PSM::utility PSM::gl_utility PSM::shape_model)

if(WITH_IMGUI)
    target_link_libraries(optimizer PSM::imgui)
    target_compile_definitions(optimizer PUBLIC "WITH_IMGUI")
endif()

target_include_directories(optimizer PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<INSTALL_INTERFACE:include/optimizer>)
install(FILES ${all_headers} DESTINATION include/optimizer)
PinscreenModuleEnd(optimizer)