cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
FaceToolkitModuleBegin(geo_app DEPENDS 
					library/Eigen3 
					library/GLEW 
					library/glfw3 
					library/gflags 
					library/OpenCV
					library/OpenMesh 
					utility/glm 
					utility/tiny_obj_loader
					utility/tinyexr
					utility/imgui
					facelib/utility
					facelib/gl_utility
					facelib/renderer)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

if(FACETOOLKIT_OS_SUFFIX STREQUAL "mac")
	# This is needed in order to be able to find the omp library when find_package(Threads) is doing it's pre-check
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -L/usr/local/opt/llvm/lib")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -L/usr/local/opt/llvm/lib")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L/usr/local/opt/llvm/lib")
endif()

add_subdirectory(src)

include("${MODULE_MANAGER_ROOT}/cmake-utils/copyFolder.cmake")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/assets")
createSymlink(../../data/camera assets/camera)
createSymlink(../../data/renderppl assets/renderppl)
createSymlink(../../shaders assets/shaders)
createSymlink(../../data/env assets/env)

set(_all_target geo_app)

install(DIRECTORY "${CMAKE_BINARY_DIR}/assets/" USE_SOURCE_PERMISSIONS DESTINATION assets)

if(BUILD_EXECUTABLE)
	install(DIRECTORY "${CMAKE_BINARY_DIR}/lib/" USE_SOURCE_PERMISSIONS DESTINATION lib)

	# link openmp and fix rpath
	if(FACETOOLKIT_OS_SUFFIX STREQUAL "mac")
		set_property(TARGET calc_thinkness APPEND PROPERTY INSTALL_RPATH "@loader_path/../lib")
	endif()
	if(FACETOOLKIT_OS_SUFFIX STREQUAL "linux")
		set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fopenmp")
		set_property(TARGET calc_thinkness APPEND PROPERTY INSTALL_RPATH "\$ORIGIN/../lib")
	endif()
else(BUILD_EXECUTABLE)
	add_library(FTM::geo_app ALIAS geo_app)
endif()