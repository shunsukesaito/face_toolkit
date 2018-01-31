# entry point of module structure
if(NOT PINSCREEN_ROOT)
	get_filename_component(PINSCREEN_ROOT ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
	get_filename_component(MODULE_MANAGER_ROOT ${PINSCREEN_ROOT}/miscellany/module-manager ABSOLUTE)
	include(${MODULE_MANAGER_ROOT}/PinscreenModule.cmake)
endif()