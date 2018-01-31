cmake_minimum_required(VERSION 3.0)

include(../../ModuleGate.cmake)
PinscreenModuleBegin(faceview DEPENDS 
					library/MKL 
					library/Eigen3 
					library/GLEW 
					library/glfw3 
					library/gflags 
					library/OpenCV 
					library/dlib 
					utility/spdlog 
					utility/glm 
					utility/cereal 
					utility/face2d-detector 
					utility/minitrace
					utility/tiny_obj_loader
					utility/SPSCQueue
					utility/tinyexr
					utility/imgui
					facelib/utility
					facelib/gl_utility
					facelib/shape_model
					facelib/regressor
					facelib/optimizer
					facelib/renderer
					facelib/f2f
					facelib/module)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

if(PINSCREEN_OS_SUFFIX STREQUAL "mac")
	# This is needed in order to be able to find the omp library when find_package(Threads) is doing it's pre-check
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -L/usr/local/opt/llvm/lib")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -L/usr/local/opt/llvm/lib")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L/usr/local/opt/llvm/lib")
endif()

add_subdirectory(src)

include("${MODULE_MANAGER_ROOT}/cmake-utils/copyFolder.cmake")
copyFolder(COPY_data ../../data assets)
copyFolder(COPY_shaders ../../shaders assets/shaders)

set_property(TARGET COPY_data PROPERTY FOLDER "COPY/ASSETS")
set_property(TARGET COPY_shaders PROPERTY FOLDER "COPY/ASSETS")

add_dependencies(faceview COPY_data COPY_shaders)

set(_all_target faceview)

install(DIRECTORY "${CMAKE_BINARY_DIR}/assets/" USE_SOURCE_PERMISSIONS DESTINATION assets)

if(BUILD_EXECUTABLE)
	target_compile_definitions(minitrace PUBLIC "MTR_ENABLED")

	install(DIRECTORY "${CMAKE_BINARY_DIR}/lib/" USE_SOURCE_PERMISSIONS DESTINATION lib)

	# link openmp and fix rpath
	if(PINSCREEN_OS_SUFFIX STREQUAL "mac")
		set_property(TARGET faceview APPEND PROPERTY INSTALL_RPATH "@loader_path/../lib")
	endif()
	if(PINSCREEN_OS_SUFFIX STREQUAL "linux")
		set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fopenmp")
		set_property(TARGET faceview APPEND PROPERTY INSTALL_RPATH "\$ORIGIN/../lib")
	endif()
else(BUILD_EXECUTABLE)
	add_library(PSM::faceview ALIAS faceview)
endif()