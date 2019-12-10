cmake_minimum_required(VERSION 3.5.0)

find_package(SWIG 4.0.0 QUIET)
if ( NOT ${SWIG_FOUND} )
    include(ExternalProject)

    ExternalProject_Add(SWIG
      URL http://prdownloads.sourceforge.net/swig/swig-4.0.1.tar.gz
      SOURCE_DIR        ${CMAKE_BINARY_DIR}/../swig-src
      BINARY_DIR        ${CMAKE_BINARY_DIR}/../swig-src
      CONFIGURE_COMMAND "./configure" "--prefix" "${CMAKE_INSTALL_PREFIX}"
      BUILD_COMMAND make
      INSTALL_COMMAND make install
    )
endif()

install(FILES package.xml DESTINATION share/swig)