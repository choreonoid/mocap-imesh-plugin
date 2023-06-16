if(MSVC)
  option(USE_BUNDLED_TETGEN "Use the tegen library bundled with MocapIMeshPlugin" ON)
else()
  option(USE_BUNDLED_TETGEN "Use the tegen library bundled with MocapIMeshPlugin" OFF)
endif()

if(USE_BUNDLED_TETGEN)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/tetgen)
else()
  set(TETGEN_LIBRARY tet)
endif()

set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS "TETLIBRARY")

include_directories(/usr/include/suitesparse) # UMFPack header location in Ubuntu

set(target CnoidMocapIMeshPlugin)

set(sources
  MocapIMeshPlugin.cpp
  MarkerIMesh.cpp
  MarkerIMeshSolver.cpp
  BodyIMesh.cpp
  BodyIMeshSolver.cpp
  IMeshItem.cpp
  MarkerIMeshItem.cpp
  BodyIMeshItem.cpp
  )

if(USE_BUNDLED_TETGEN)
  set(sources ${sources} tetgen/tetgen.cxx tetgen/predicates.cxx)
endif()

choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles})
target_link_libraries(${target} PUBLIC CnoidMocapPlugin PRIVATE ${TETGEN_LIBRARY} umfpack)
