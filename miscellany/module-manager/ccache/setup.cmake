# https://crascit.com/2016/04/09/using-ccache-with-cmake/

if(CCACHE_PROGRAM)
  # Set up wrapper scripts
  set(C_LAUNCHER   "${CCACHE_PROGRAM}")
  set(CXX_LAUNCHER "${CCACHE_PROGRAM}")
  if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set (C_COMPILER "/usr/local/opt/llvm/bin/clang")
    set (CXX_COMPILER "/usr/local/opt/llvm/bin/clang++")
  else()
    set (C_COMPILER "${CMAKE_C_COMPILER}")
    set (CXX_COMPILER "${CMAKE_CXX_COMPILER}")
  endif()
  configure_file("${MODULE_MANAGER_ROOT}/ccache/launch-c.in" launch-c)
  configure_file("${MODULE_MANAGER_ROOT}/ccache/launch-cxx.in" launch-cxx)
  execute_process(COMMAND chmod a+rx
                 "${CMAKE_BINARY_DIR}/launch-c"
                 "${CMAKE_BINARY_DIR}/launch-cxx"
  )

  if(CMAKE_GENERATOR STREQUAL "Xcode")
    # Set Xcode project attributes to route compilation and linking
    # through our scripts
    set(CMAKE_XCODE_ATTRIBUTE_CC         "${CMAKE_BINARY_DIR}/launch-c")
    set(CMAKE_XCODE_ATTRIBUTE_CXX        "${CMAKE_BINARY_DIR}/launch-cxx")
    set(CMAKE_XCODE_ATTRIBUTE_LD         "${CMAKE_BINARY_DIR}/launch-c")
    set(CMAKE_XCODE_ATTRIBUTE_LDPLUSPLUS "${CMAKE_BINARY_DIR}/launch-cxx")
  else()
    # Support Unix Makefiles and Ninja
    set(CMAKE_C_COMPILER_LAUNCHER   "${CMAKE_BINARY_DIR}/launch-c")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CMAKE_BINARY_DIR}/launch-cxx")
  endif()
else()
  if(CMAKE_GENERATOR STREQUAL "Xcode")
    set(CMAKE_XCODE_ATTRIBUTE_CC         "/usr/local/opt/llvm/bin/clang")
    set(CMAKE_XCODE_ATTRIBUTE_CXX        "/usr/local/opt/llvm/bin/clang++")
  endif()
endif()