cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
FaceToolkitModuleBegin(utility DEPENDS 
                    library/Eigen3 
                    library/OpenCV
                    utility/tiny_obj_loader
                    utility/tinyexr)

file(GLOB_RECURSE all_sources *.cpp)
file(GLOB_RECURSE all_headers *.h)
add_library(utility ${all_sources} ${all_headers})

target_link_libraries(utility FTM::OpenCV FTM::Eigen3 FTM::tiny_obj_loader FTM::tinyexr)
target_include_directories(utility PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<INSTALL_INTERFACE:include/utility>)
install(FILES ${all_headers} DESTINATION include/utility)
FaceToolkitModuleEnd(utility)
