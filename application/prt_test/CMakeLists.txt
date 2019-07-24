cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
FaceToolkitModuleBegin(prt_test DEPENDS 
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
createSymlink(../../data/opt assets/opt)
createSymlink(../../shaders assets/shaders)
createSymlink(../../data/geo_data assets/geo_data)
createSymlink(../../data/FaceWarehouse assets/FaceWarehouse)
createSymlink(../../data/dde assets/dde)
createSymlink(../../data/env assets/env)
createSymlink(../../data/LS assets/LS)
createSymlink(../../data/render assets/render)
createSymlink(../../data/image assets/image)

set(_all_target prt_test)

install(DIRECTORY "${CMAKE_BINARY_DIR}/assets/" USE_SOURCE_PERMISSIONS DESTINATION assets)

if(BUILD_EXECUTABLE)
	install(DIRECTORY "${CMAKE_BINARY_DIR}/lib/" USE_SOURCE_PERMISSIONS DESTINATION lib)

	# link openmp and fix rpath
	if(FACETOOLKIT_OS_SUFFIX STREQUAL "mac")
		set_property(TARGET prt_test APPEND PROPERTY INSTALL_RPATH "@loader_path/../lib")
	endif()
	if(FACETOOLKIT_OS_SUFFIX STREQUAL "linux")
		set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fopenmp")
		set_property(TARGET prt_test APPEND PROPERTY INSTALL_RPATH "\$ORIGIN/../lib")
	endif()
else(BUILD_EXECUTABLE)
	add_library(FTM::prt_test ALIAS prt_test)
endif()