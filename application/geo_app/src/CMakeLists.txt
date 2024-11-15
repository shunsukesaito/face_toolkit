cmake_minimum_required(VERSION 3.0)

project(geo_app)

set(ENABLE_CUDA 1 CACHE BOOL "enable CUDA")
if(ENABLE_CUDA)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DENABLE_CUDA=1 ")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_CUDA=1")
endif(ENABLE_CUDA)

#    Threads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

if (UNIX)
	SET(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()

if(ENABLE_CUDA)
  find_package(CUDA)
  
  set(CUDA_DRIVER_LIB "")
  set(CUDA_NVML_LIB "")
  set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -arch=sm_30")
  if(FACETOOLKIT_OS_SUFFIX STREQUAL "linux")
    set(CUDA_DRIVER_LIB "${CUDA_TOOLKIT_ROOT_DIR}/lib64/stubs/libcuda.so")
    set(CUDA_NVML_LIB "${CUDA_TOOLKIT_ROOT_DIR}/lib64/stubs/libnvidia-ml.so")
  endif()

  if(FACETOOLKIT_OS_SUFFIX STREQUAL "mac")
    #set(CMAKE_CUDA_DEVICE_LINK_EXECUTABLE "${CUDA_TOOLKIT_ROOT_DIR}/bin/nvcc")
    set(CUDA_DRIVER_LIB "${CUDA_TOOLKIT_ROOT_DIR}/lib/libcuda.dylib")
  endif()

  include_directories(${CUDA_INCLUDE_DIRS})
endif(ENABLE_CUDA)

set_property(
  DIRECTORY
  APPEND PROPERTY COMPILE_DEFINITIONS _USE_MATH_DEFINES
)

# windows builds require WinSDK
if(WIN32)
    set(WINSDK_LIB "" CACHE PATH "Windows SDK libraries")
    set(CMAKE_PREFIX_PATH ${WINSDK_LIB})
endif(WIN32)

#enable_testing()

# internal projects
include_directories(${CMAKE_SOURCE_DIR})

file(GLOB_RECURSE all_sources *.cpp)
file(GLOB_RECURSE all_headers *.h)

set(BUILD_EXECUTABLE 0 CACHE BOOL "build library if not, it build executable")
if(BUILD_EXECUTABLE)
  add_custom_target(create-folder
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${CMAKE_BINARY_DIR}/$<CONFIG>/cache/"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${CMAKE_BINARY_DIR}/$<CONFIG>/logs/faceview/")

  add_executable(calc_thinkness main.cpp calc_thickness.cpp gui.h)
  set_target_properties(calc_thinkness PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>")

  add_executable(prt_test main.cpp prt_test.cpp gui.h)
  set_target_properties(prt_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>")
  
else(BUILD_EXECUTABLE)
  add_library(calc_thinkness STATIC ${all_sources} ${all_headers})
  add_library(prt_test STATIC ${all_sources} ${all_headers})
endif(BUILD_EXECUTABLE)

set(OPENMP_LIB "")
if(FACETOOLKIT_OS_SUFFIX STREQUAL "mac")
  set(OPENMP_LIB "/usr/local/opt/llvm/lib/libiomp5.dylib")
endif()

target_link_libraries(calc_thinkness
                      FTM::renderer
                      FTM::gl_utility
                      FTM::utility
                      FTM::gflags
                      FTM::OpenCV
                      ${DLIB_LIBRARIES}
                      Threads::Threads
                      ${CMAKE_DL_LIBS}
                      ${OPENMP_LIB})

install(TARGETS calc_thinkness
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib)
    
target_link_libraries(prt_test
                      FTM::renderer
                      FTM::gl_utility
                      FTM::utility
                      FTM::gflags
                      FTM::OpenCV
                      ${DLIB_LIBRARIES}
                      Threads::Threads
                      ${CMAKE_DL_LIBS}
                      ${OPENMP_LIB})

install(TARGETS prt_test
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib)
