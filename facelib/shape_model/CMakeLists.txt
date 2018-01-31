cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
PinscreenModuleBegin(shape_model DEPENDS 
                    library/Eigen3 
                    library/OpenCV
                    utility/imgui
                    facelib/utility
                    facelib/gl_utility)

file(GLOB_RECURSE all_sources *.cpp)
file(GLOB_RECURSE all_headers *.h)
add_library(shape_model ${all_sources} ${all_headers})

target_link_libraries(shape_model PSM::OpenCV PSM::Eigen3 PSM::utility PSM::gl_utility)
if(WITH_IMGUI)
    target_link_libraries(shape_model PSM::imgui)
    target_compile_definitions(shape_model PUBLIC "WITH_IMGUI")
endif()
target_include_directories(shape_model PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<INSTALL_INTERFACE:include/shape_model>)
install(FILES ${all_headers} DESTINATION include/shape_model)
PinscreenModuleEnd(shape_model)