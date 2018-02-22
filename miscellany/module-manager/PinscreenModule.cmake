if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set (PINSCREEN_OS_SUFFIX "mac")
endif()
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set (PINSCREEN_OS_SUFFIX "linux")
endif()

# When this is on, nothing internal or the ones with install/ will get built.
# Basically it will just build and install missing external dependencies (default OFF)
option (EXTERNAL_ONLY "compile only external libraries that are not installed" OFF)

# cmake modules
include (ExternalProject)
include(CMakePackageConfigHelpers)
# for cmake < 3.4
include(CMakeParseArguments)
find_program(CCACHE_PROGRAM ccache)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

macro(returnExternals EXTERNALS)
  get_directory_property(_hasParent PARENT_DIRECTORY)
  if(_hasParent)
    set(EXTERNAL_MODULES ${EXTERNALS} PARENT_SCOPE)
  endif()
endmacro(returnExternals)

macro(findModule MODULE)
  get_filename_component(_modulePath "${PINSCREEN_ROOT}/${MODULE}" ABSOLUTE)
  get_filename_component(_moduleName "${MODULE}" NAME)
  get_filename_component(_moduleFolder "${MODULE}" DIRECTORY)
  if(NOT TARGET PSM::${_moduleName})
    add_subdirectory("${_modulePath}" "${CMAKE_BINARY_DIR}/modules/${MODULE}" EXCLUDE_FROM_ALL)
    if(TARGET ${_moduleName}) # cleanup in IDE
      set_property(TARGET ${_moduleName} PROPERTY FOLDER "${_moduleFolder}")
    endif()
  endif()
endmacro(findModule)



macro(PinscreenModuleBegin BASE_MODULE_NAME)
  if(TARGET PSM::${BASE_MODULE_NAME})
    return()
  endif()
  set(_this_EXTERNAL_MODULES ${EXTERNAL_MODULES}) # backup
  set(EXTERNAL_MODULES "") # now list externals of children

  set(multiValueArgs DEPENDS DEFINITIONS LANGUAGES)
  cmake_parse_arguments(PSC_MODULE "" "" "${multiValueArgs}" ${ARGN} )

  # add every child modules first
  foreach(_module ${PSC_MODULE_DEPENDS})
    findModule(${_module})
  endforeach(_module)

  if(EXTERNAL_MODULES AND NOT EXTERNAL_ONLY)
    message(STATUS "External libraries missing. Consider build all libs with EXTERNAL_ONLY=ON first and rerun for actual build")
    message(FATAL_ERROR "Missing modules: ${EXTERNAL_MODULES}")
  endif()

  if(EXTERNAL_ONLY)
    message("${BASE_MODULE_NAME}: depending on ${EXTERNAL_MODULES}")
    ExternalProject_Add (${BASE_MODULE_NAME}
      DEPENDS ${EXTERNAL_MODULES}
      SOURCE_DIR "${MODULE_MANAGER_ROOT}/void-project"
      CMAKE_ARGS -DMODULE_NAME=${BASE_MODULE_NAME}
      BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/void-build"
      CONFIGURE_COMMAND ""
      BUILD_COMMAND ""
      INSTALL_COMMAND ""
      )
    list(APPEND _this_EXTERNAL_MODULES ${BASE_MODULE_NAME})
    returnExternals(${_this_EXTERNAL_MODULES})
    return() 
  endif()

  # returnExternals(${_this_EXTERNAL_MODULES}) # no change of dependencies
  set(MODULE_INSTALL_PATH "${CMAKE_CURRENT_LIST_DIR}/install")
  if (EXISTS "${MODULE_INSTALL_PATH}/cmake")
    message("Using pre-built version of internal module ${BASE_MODULE_NAME}, remove the install folder if this is not desired.")
    find_package(${BASE_MODULE_NAME} REQUIRED
        PATHS "${MODULE_INSTALL_PATH}" NO_DEFAULT_PATH)
    return()
  endif()
  message("configuring ${BASE_MODULE_NAME}")

  if(PSC_MODULE_LANGUAGES)
    project(${BASE_MODULE_NAME} LANGUAGES ${PSC_MODULE_LANGUAGES})
  else()
    project(${BASE_MODULE_NAME})
  endif()
  include("${MODULE_MANAGER_ROOT}/ccache/setup.cmake")
  # here follows the actual code
endmacro(PinscreenModuleBegin)


macro(PinscreenModulePrebuilt BASE_MODULE_NAME)
  if(TARGET PSM::${BASE_MODULE_NAME})
    return()
  endif()
  set(_this_EXTERNAL_MODULES ${EXTERNAL_MODULES}) # backup
  set(EXTERNAL_MODULES "") # now list externals of children

  set(multiValueArgs DEPENDS DEFINITIONS)
  cmake_parse_arguments(PSC_MODULE "" "" "${multiValueArgs}" ${ARGN} )

  # ACTUAL_BUILDING should be added only by itself. If it's OFF we handle dependencies
  if(NOT ACTUAL_BUILDING)
    # add every child modules first
    foreach(_module ${PSC_MODULE_DEPENDS})
      findModule(${_module})
    endforeach(_module)

    set(MODULE_INSTALL_PATH "${CMAKE_CURRENT_LIST_DIR}/install")
    if(EXISTS "${MODULE_INSTALL_PATH}/cmake")
      find_package(${BASE_MODULE_NAME} REQUIRED
          PATHS "${MODULE_INSTALL_PATH}" NO_DEFAULT_PATH)
    elseif(NOT EXTERNAL_ONLY)
      message(STATUS "External libraries missing. Consider build all libs with EXTERNAL_ONLY=ON first and rerun for actual build")
      message(STATUS "When installing ${MODULE_INSTALL_PATH}")
      message(FATAL_ERROR "Missing modules: ${EXTERNAL_MODULES}")
    else()
      ExternalProject_Add (${BASE_MODULE_NAME}
        DEPENDS ${EXTERNAL_MODULES}
        SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}"
        CMAKE_ARGS -DACTUAL_BUILDING=ON "-DCMAKE_INSTALL_PREFIX=${MODULE_INSTALL_PATH}" "-DPINSCREEN_PLATFORM=${PINSCREEN_PLATFORM}"
        BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/actual-build"
        INSTALL_DIR "${MODULE_INSTALL_PATH}"
        )
      list(APPEND _this_EXTERNAL_MODULES ${BASE_MODULE_NAME})
      returnExternals(${_this_EXTERNAL_MODULES})
    endif()
    return() 
  endif()
  set(ACTUAL_BUILDING OFF)
  # add every child modules first
  foreach(_module ${PSC_MODULE_DEPENDS})
    findModule(${_module})
  endforeach(_module)

  project(${BASE_MODULE_NAME})
  include("${MODULE_MANAGER_ROOT}/ccache/setup.cmake")
  # here follows the actual code
endmacro(PinscreenModulePrebuilt)


macro(PinscreenModuleEnd BASE_MODULE_NAME)
  set_target_properties(${BASE_MODULE_NAME} PROPERTIES
    POSITION_INDEPENDENT_CODE ON # always add fPIC
    COMPILE_OPTIONS "$<$<COMPILE_LANGUAGE:CXX>:-std=c++11>"
    XCODE_ATTRIBUTE_CC "${CMAKE_C_COMPILER}"
    XCODE_ATTRIBUTE_CXX "${CMAKE_CXX_COMPILER}")

  add_library(PSM::${BASE_MODULE_NAME} ALIAS ${BASE_MODULE_NAME}) # attach PSM:: namespace for consistency when module is prebuilt

  install(TARGETS ${BASE_MODULE_NAME}
    EXPORT ${BASE_MODULE_NAME}Targets
    ARCHIVE DESTINATION lib/${PINSCREEN_OS_SUFFIX}
    LIBRARY DESTINATION lib/${PINSCREEN_OS_SUFFIX}
    INCLUDES DESTINATION include)
  #install(EXPORT ${BASE_MODULE_NAME}Targets DESTINATION cmake/${PINSCREEN_OS_SUFFIX})
  set(PACKAGE_MODULE_NAME ${BASE_MODULE_NAME})

  configure_package_config_file(${MODULE_MANAGER_ROOT}/TemplateConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/${BASE_MODULE_NAME}Config.cmake
    INSTALL_DESTINATION cmake)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${BASE_MODULE_NAME}Config.cmake
    DESTINATION cmake)
endmacro(PinscreenModuleEnd)

macro(PinscreenHeaderModule BASE_MODULE_NAME)
  if(TARGET PSM::${BASE_MODULE_NAME})
    return()
  endif()
  set(optionArgs HIDE_HEADER)
  set(multiValueArgs DEPENDS)
  set(singleValueArgs LOCATION)
  cmake_parse_arguments(PSC_MODULE "${optionArgs}" "${singleValueArgs}" "${multiValueArgs}" ${ARGN} )
  if(NOT PSC_MODULE_LOCATION)
    message(FATAL_ERROR "need to specify when header files are.")
  endif()
  PinscreenModuleBegin(${BASE_MODULE_NAME} DEPENDS ${PSC_MODULE_DEPENDS})

  add_library(PSM::${BASE_MODULE_NAME} INTERFACE IMPORTED GLOBAL)
  set_target_properties(PSM::${BASE_MODULE_NAME} PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${PSC_MODULE_LOCATION}"
  )

  if(NOT PSC_MODULE_HIDE_HEADER)
    file(GLOB_RECURSE all_files "${CMAKE_CURRENT_LIST_DIR}/*.h" "${CMAKE_CURRENT_LIST_DIR}/*.hpp")
    add_custom_target(${BASE_MODULE_NAME} SOURCES ${all_files})
  endif()

endmacro(PinscreenHeaderModule)
