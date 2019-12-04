cmake_minimum_required(VERSION 2.4.3)

add_compile_options(-std=c++11 -w)

# This causes accuracy issues in the EPA algorithm and need to create an issue.
#set(BUILD_PYBULLET ON CACHE BOOL "Enable Bullet Python")
#set(BUILD_PYBULLET_NUMPY OFF CACHE BOOL "Disable Bullet Python Numpy")
#set(USE_DOUBLE_PRECISION ON CACHE BOOL "Enable Bullet Double Precision")

set(BUILD_UNIT_TESTS OFF CACHE BOOL "Disable Bullet Unit Tests" FORCE)
set(BULLET2_MULTITHREADING ON CACHE BOOL "Allows you to use bullet in multithreaded applicaiton" FORCE)

if (NOT WIN32)
	set(BUILD_SHARED_LIBS ON CACHE BOOL "Enable Bullet Shared Libs" FORCE)
else()
	set(BUILD_SHARED_LIBS OFF CACHE BOOL "Enable Bullet Shared Libs" FORCE)
	set(USE_MSVC_RUNTIME_LIBRARY_DLL ON CACHE BOOL "Use MSVC Runtime DLL" FORCE)
	set(CMAKE_RELWITHDEBINFO_POSTFIX "" CACHE STRING "Adds a postfix for ReleaseWithDebug-built libraries." FORCE)
	set(INSTALL_LIBS ON CACHE BOOL "Set when you want to install libraries" FORCE)
	set(USE_MSVC_DISABLE_RTTI OFF CACHE BOOL "Use MSVC /GR- disabled RTTI flags option"	FORCE)
endif()

add_subdirectory(bullet3)

install(FILES package.xml DESTINATION share/bullet3)
