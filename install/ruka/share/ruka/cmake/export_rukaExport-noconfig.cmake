#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ruka::ruka" for configuration ""
set_property(TARGET ruka::ruka APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ruka::ruka PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libruka.so"
  IMPORTED_SONAME_NOCONFIG "libruka.so"
  )

list(APPEND _cmake_import_check_targets ruka::ruka )
list(APPEND _cmake_import_check_files_for_ruka::ruka "${_IMPORT_PREFIX}/lib/libruka.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
