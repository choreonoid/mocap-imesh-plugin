option(BUILD_MOCAP_IMESH_PLUGIN "Building MocapIMeshPlugin" OFF)

if(BUILD_MOCAP_IMESH_PLUGIN)
  if(NOT BUILD_MOCAP_PLUGIN)
    message(FATAL_ERROR "BUILD_MOCAP_IMESH_PLUGIN needs to BUILD_MOCAP_PLUGIN")
  endif()
  add_subdirectory(src)
endif()
