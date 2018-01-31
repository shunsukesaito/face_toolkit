cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
PinscreenModuleBegin(regressor DEPENDS 
                    library/Eigen3 
                    library/OpenCV)

file(GLOB_RECURSE all_sources *.cpp)
file(GLOB_RECURSE all_headers *.h)
add_library(regressor ${all_sources} ${all_headers})

target_link_libraries(regressor PSM::OpenCV PSM::Eigen3)

target_include_directories(regressor PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<INSTALL_INTERFACE:include/regressor>)
install(FILES ${all_headers} DESTINATION include/regressor)
PinscreenModuleEnd(regressor)