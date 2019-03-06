cmake_minimum_required(VERSION 2.8.3)
project(tesseract_python)

add_compile_options(-std=c++11 -Wall -Wextra -Wno-write-strings)

find_package(catkin REQUIRED COMPONENTS
  tesseract_ros
  tesseract_core
  tesseract_msgs
  tesseract_collision
  tesseract_planning
  roscpp
  urdf
  srdfdom
  geometric_shapes
  kdl_parser
  eigen_conversions
  pluginlib
  trajopt
)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED)
find_package(octomap REQUIRED)
find_package(orocos_kdl REQUIRED)

find_package(PythonInterp REQUIRED)
find_package(PythonLibs REQUIRED)

INCLUDE(FindSWIG)
FIND_PACKAGE(SWIG REQUIRED)
INCLUDE(UseSWIG)

catkin_package(      
  CATKIN_DEPENDS
    tesseract_ros
    tesseract_core
    tesseract_msgs
    tesseract_collision    
    roscpp
    urdf
    srdfdom
    geometric_shapes
    kdl_parser
    eigen_conversions
    pluginlib
  DEPENDS
    EIGEN3
    Boost
    OCTOMAP
    orocos_kdl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${Boost_INCLUDE_DIRS}
  SYSTEM ${OCTOMAP_INCLUDE_DIRS}
  SYSTEM ${orocos_kdl_INCLUDE_DIRS}
  SYSTEM ${PYTHON_INCLUDE_DIRS}
)


SET(SWIG_CXX_EXTENSION cxx)
set_property(SOURCE swig/tesseract_python.i PROPERTY CPLUSPLUS ON)
set_property(SOURCE swig/tesseract_python.i PROPERTY SWIG_FLAGS -relativeimport -threads)

set(CMAKE_SWIG_OUTDIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/tesseract)

SWIG_ADD_MODULE(tesseract_python python swig/tesseract_python.i)
SWIG_LINK_LIBRARIES(tesseract_python ${PYTHON_LIBRARIES} ${EIGEN3_LIBRARIES} ${PYTHON_LIBRARIES} ${catkin_LIBRARIES})

set_target_properties(${SWIG_MODULE_tesseract_python_REAL_NAME}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/tesseract)

catkin_python_setup()

install(TARGETS ${SWIG_MODULE_tesseract_python_REAL_NAME}
  LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}/tesseract
)

