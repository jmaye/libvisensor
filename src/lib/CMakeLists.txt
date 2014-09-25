remake_find_package(eigen3 CONFIG)
remake_find_package(Boost COMPONENTS thread system
  RESULT_VAR Boost_FOUND)

remake_include()
remake_include(${EIGEN3_INCLUDE_DIRS})
remake_include(${Boost_INCLUDE_DIR})

remake_add_directories(visensor)
remake_add_library(visensor RECURSE PREFIX OFF LINK ${Boost_LIBRARIES})

remake_pkg_config_generate(EXTRA_CFLAGS -std=c++0x)
