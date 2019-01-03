cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
FaceToolkitModuleBegin(gl_utility DEPENDS 
                    library/Eigen3 
                    library/OpenCV
                    library/GLEW 
                    library/glfw3 
                    utility/imgui
                    utility/glm
                    utility/tinyexr
                    utility/hdrloader
                    facelib/utility)

file(GLOB_RECURSE all_sources *.cpp)
file(GLOB_RECURSE all_headers *.h)
add_library(gl_utility ${all_sources} ${all_headers})

target_link_libraries(gl_utility FTM::OpenCV FTM::Eigen3 FTM::GLEW FTM::glfw3 FTM::glm FTM::tinyexr FTM::hdrloader FTM::utility)

if(WITH_IMGUI)
    FaceToolkitModuleBegin(gl_utility DEPENDS utility/imgui)
    target_link_libraries(gl_utility FTM::imgui)
    target_compile_definitions(gl_utility PUBLIC "WITH_IMGUI")
endif()

target_include_directories(gl_utility PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<INSTALL_INTERFACE:include/gl_utility>)
install(FILES ${all_headers} DESTINATION include/gl_utility)
FaceToolkitModuleEnd(gl_utility)
