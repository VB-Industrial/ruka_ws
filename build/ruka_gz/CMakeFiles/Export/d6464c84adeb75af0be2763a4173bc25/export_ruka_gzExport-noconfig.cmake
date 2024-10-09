#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ruka_gz::ruka_gz" for configuration ""
set_property(TARGET ruka_gz::ruka_gz APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ruka_gz::ruka_gz PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libruka_gz.so"
  IMPORTED_SONAME_NOCONFIG "libruka_gz.so"
  )

list(APPEND _cmake_import_check_targets ruka_gz::ruka_gz )
list(APPEND _cmake_import_check_files_for_ruka_gz::ruka_gz "${_IMPORT_PREFIX}/lib/libruka_gz.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
