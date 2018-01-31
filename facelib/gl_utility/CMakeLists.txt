cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
PinscreenModuleBegin(gl_utility DEPENDS 
                    library/Eigen3 
                    library/OpenCV
                    library/GLEW 
                    library/glfw3 
                    utility/imgui
                    utility/glm
                    utility/tinyexr
                    utility/hdrloader
                    utility/minitrace
                    facelib/utility)

file(GLOB_RECURSE all_sources *.cpp)
file(GLOB_RECURSE all_headers *.h)
add_library(gl_utility ${all_sources} ${all_headers})

target_link_libraries(gl_utility PSM::OpenCV PSM::Eigen3 PSM::GLEW PSM::glfw3 PSM::glm PSM::tinyexr PSM::hdrloader PSM::minitrace PSM::utility)

if(WITH_IMGUI)
    target_link_libraries(gl_utility PSM::imgui)
    target_compile_definitions(gl_utility PUBLIC "WITH_IMGUI")
endif()

target_include_directories(gl_utility PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<INSTALL_INTERFACE:include/gl_utility>)
install(FILES ${all_headers} DESTINATION include/gl_utility)
PinscreenModuleEnd(gl_utility)