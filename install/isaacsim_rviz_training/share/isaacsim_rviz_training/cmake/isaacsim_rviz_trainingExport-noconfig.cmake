#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "isaacsim_rviz_training::isaacsim_rviz_training" for configuration ""
set_property(TARGET isaacsim_rviz_training::isaacsim_rviz_training APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(isaacsim_rviz_training::isaacsim_rviz_training PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libisaacsim_rviz_training.so"
  IMPORTED_SONAME_NOCONFIG "libisaacsim_rviz_training.so"
  )

list(APPEND _cmake_import_check_targets isaacsim_rviz_training::isaacsim_rviz_training )
list(APPEND _cmake_import_check_files_for_isaacsim_rviz_training::isaacsim_rviz_training "${_IMPORT_PREFIX}/lib/libisaacsim_rviz_training.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
