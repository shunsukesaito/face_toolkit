cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
PinscreenModuleBegin(f2f DEPENDS 
                    library/Eigen3 
                    library/OpenCV
                    utility/minitrace 
                    utility/spdlog 
                    utility/imgui
                    facelib/utility 
                    facelib/gl_utility 
                    facelib/shape_model 
                    facelib/optimizer 
                    facelib/renderer)

file(GLOB_RECURSE all_sources *.cpp)
file(GLOB_RECURSE all_headers *.h)
add_library(f2f ${all_sources} ${all_headers})

target_link_libraries(f2f PSM::OpenCV PSM::Eigen3 PSM::minitrace PSM::spdlog PSM::utility PSM::gl_utility PSM::shape_model PSM::optimizer PSM::renderer)

if(WITH_IMGUI)
    target_link_libraries(f2f PSM::imgui)
    target_compile_definitions(f2f PUBLIC "WITH_IMGUI")
endif()

target_include_directories(f2f PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<INSTALL_INTERFACE:include/f2f>)
install(FILES ${all_headers} DESTINATION include/f2f)
PinscreenModuleEnd(f2f)