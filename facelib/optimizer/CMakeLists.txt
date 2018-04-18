cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
FaceToolkitModuleBegin(optimizer DEPENDS 
                    library/Eigen3 
                    library/OpenCV
                    library/gflags 
                    utility/minitrace 
                    utility/imgui
                    facelib/utility 
                    facelib/gl_utility 
                    facelib/shape_model)

file(GLOB_RECURSE all_sources *.cpp)
file(GLOB_RECURSE all_headers *.h)
add_library(optimizer ${all_sources} ${all_headers})

target_link_libraries(optimizer FTM::OpenCV FTM::Eigen3 FTM::gflags FTM::minitrace FTM::utility FTM::gl_utility FTM::shape_model)

if(WITH_IMGUI)
    target_link_libraries(optimizer FTM::imgui)
    target_compile_definitions(optimizer PUBLIC "WITH_IMGUI")
endif()

target_include_directories(optimizer PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<INSTALL_INTERFACE:include/optimizer>)
install(FILES ${all_headers} DESTINATION include/optimizer)
FaceToolkitModuleEnd(optimizer)