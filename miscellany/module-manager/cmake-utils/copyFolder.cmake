macro(copyFolder NAME SOURCE TARGET)
  execute_process(
    COMMAND
      "${CMAKE_COMMAND}" -E remove -f "${NAME}.stamp"
    )


  file(WRITE ${CMAKE_BINARY_DIR}/copyFolderScripts/${NAME}.cmake
  "file(
    COPY \"${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE}/\"
    DESTINATION \"${CMAKE_BINARY_DIR}/${TARGET}\"
    ${ARGN}
  )")

  add_custom_command(
    OUTPUT
      "${NAME}.stamp"
    COMMAND
      ${CMAKE_COMMAND} -P ${CMAKE_BINARY_DIR}/copyFolderScripts/${NAME}.cmake
    COMMAND
      "${CMAKE_COMMAND}" -E touch "${NAME}.stamp"
    COMMENT
      "Copying ${SOURCE} to ${TARGET}"
    VERBATIM
  )

  add_custom_target(${NAME}
    DEPENDS "${NAME}.stamp"
    )
endmacro(copyFolder)

macro(copyData SOURCE TARGET)
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE}/"
     DESTINATION "${CMAKE_BINARY_DIR}/${TARGET}"
     ${ARGN})
endmacro(copyData)

macro(createSymlink SOURCE TARGET)
    execute_process(COMMAND "${CMAKE_COMMAND}" -E create_symlink
                    "${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE}/"
                    "${CMAKE_BINARY_DIR}/${TARGET}")
endmacro(createSymlink)