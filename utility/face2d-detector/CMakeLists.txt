cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
PinscreenModuleBegin(face2d-detector DEPENDS library/dlib library/OpenCV library/Eigen3 library/gflags)

file(GLOB_RECURSE all_sources *.cc)
file(GLOB_RECURSE all_headers *.h)
add_library(face2d-detector ${all_sources} ${all_headers})

include("${MODULE_MANAGER_ROOT}/cmake-utils/copyFolder.cmake")
copyFolder(COPY_face2d-detector assets assets)
set_property(TARGET COPY_face2d-detector PROPERTY FOLDER "COPY/ASSETS")
add_dependencies(face2d-detector COPY_face2d-detector)

target_link_libraries(face2d-detector PSM::OpenCV PSM::dlib PSM::Eigen3 PSM::gflags)
target_include_directories(face2d-detector PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<INSTALL_INTERFACE:include/face2d-detector>)
install(FILES face2d_detector.h DESTINATION include/face2d-detector)
PinscreenModuleEnd(face2d-detector)