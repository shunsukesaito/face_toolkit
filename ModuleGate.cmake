# entry point of module structure
if(NOT FACETOOLKIT_ROOT)
	get_filename_component(FACETOOLKIT_ROOT ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
	get_filename_component(MODULE_MANAGER_ROOT ${FACETOOLKIT_ROOT}/miscellany/module-manager ABSOLUTE)
	include(${MODULE_MANAGER_ROOT}/ToolkitModule.cmake)
endif()