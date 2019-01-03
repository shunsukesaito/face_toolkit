cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
FaceToolkitModuleBegin(renderer DEPENDS 
                    library/Eigen3 
                    library/OpenCV
                    library/gflags 
                    facelib/utility
                    facelib/gl_utility)

file(GLOB_RECURSE all_sources *.cpp)
file(GLOB_RECURSE all_headers *.h)
add_library(renderer ${all_sources} ${all_headers})

target_link_libraries(renderer FTM::OpenCV FTM::Eigen3 FTM::gflags FTM::utility FTM::gl_utility)

if(WITH_IMGUI)
    FaceToolkitModuleBegin(renderer DEPENDS utility/imgui)
    target_link_libraries(renderer FTM::imgui)
    target_compile_definitions(renderer PUBLIC "WITH_IMGUI")
endif()
if(FACE_TOOLKIT)
    FaceToolkitModuleBegin(renderer DEPENDS facelib/shape_model facelib/optimizer)
    target_link_libraries(renderer FTM::shape_model FTM::optimizer)
    target_compile_definitions(renderer PUBLIC "FACE_TOOLKIT")
endif()

target_include_directories(renderer PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<INSTALL_INTERFACE:include/renderer>)
install(FILES ${all_headers} DESTINATION include/renderer)
FaceToolkitModuleEnd(renderer)
