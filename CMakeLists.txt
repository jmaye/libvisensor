cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  libvisensor
  VERSION 1.0
  RELEASE alpha
  SUMMARY "A C++-based library for interfacing the VI-sensor."
  AUTHOR "Skybotix AG"
  CONTACT "jerome.maye@mavt.ethz.ch"
  HOME "https://github.com/jmaye/libvisensor"
  LICENSE "Apache License, Version 2.0 (ALv2)"
  INSTALL /usr/
  EXTRA_CXX_FLAGS -std=c++0x
  HEADER_DESTINATION visensor
  PREFIX visensor-
)

remake_doc(
  html
  latex
)

remake_pack_deb(
  DESCRIPTION "Main library"
  DEPENDS libvisensor libboost-system[0-9.]* libboost-thread[0-9.]*
)

remake_pack_deb(
  COMPONENT dev
  DESCRIPTION "Development headers"
  DEPENDS libvisensor libboost-all-dev
)

remake_pack_deb(
  COMPONENT doc
  DESCRIPTION "Documentation"
  DEPENDS libvisensor
)

remake_pack_deb(
  COMPONENT examples
  DESCRIPTION "Examples"
  DEPENDS libvisensor
)

remake_distribute_deb(
  DISTRIBUTION lucid
  SECTION libs
  UPLOAD ppa:jmaye/ethz
  DEPENDS remake doxygen pkg-config libeigen3-dev libboost-all-dev
  PASS CMAKE_BUILD_TYPE LIBVISENSOR_GIT_REVISION
)

remake_distribute_deb(
  DISTRIBUTION precise
  SECTION libs
  UPLOAD ppa:jmaye/ethz
  DEPENDS remake doxygen pkg-config libeigen3-dev libboost-all-dev
  PASS CMAKE_BUILD_TYPE LIBVISENSOR_GIT_REVISION
)

remake_distribute_deb(
  DISTRIBUTION trusty
  SECTION libs
  UPLOAD ppa:jmaye/ethz
  DEPENDS remake doxygen pkg-config libeigen3-dev libboost-all-dev
  PASS CMAKE_BUILD_TYPE LIBVISENSOR_GIT_REVISION
)
