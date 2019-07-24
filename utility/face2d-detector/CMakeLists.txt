cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
FaceToolkitModuleBegin(face2d-detector DEPENDS library/dlib library/OpenCV library/Eigen3 library/gflags utility/tcpstream)

file(GLOB_RECURSE all_sources *.cc)
file(GLOB_RECURSE all_headers *.h)
add_library(face2d-detector ${all_sources} ${all_headers})

include("${MODULE_MANAGER_ROOT}/cmake-utils/copyFolder.cmake")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/assets")
createSymlink(assets assets/facedetector)

target_link_libraries(face2d-detector FTM::OpenCV FTM::dlib FTM::Eigen3 FTM::gflags FTM::tcpstream)
target_include_directories(face2d-detector PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<INSTALL_INTERFACE:include/face2d-detector>)
install(FILES face2d_detector.h DESTINATION include/face2d-detector)
FaceToolkitModuleEnd(face2d-detector)
