#
# Project settings
#

# Minimum required version of cmake 
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

# Project name and programming languages used
PROJECT( discrete_double_integrator CXX )

# puts executables into bin directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

# Add extra cmake files to module path
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#
# Prerequisites
#
FIND_PACKAGE( ACADO REQUIRED )
FIND_PACKAGE(PythonLibs 2.7 REQUIRED)
FIND_PACKAGE(Boost COMPONENTS iostreams python thread filesystem system numpy)
FIND_PACKAGE(Eigen)

#
# Include directories
#
INCLUDE_DIRECTORIES( . ${ACADO_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${PYTHON_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

#
# Link directories
#
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS} ${PYTHON_LIBRARIES})

#
# Build an executable
#
ADD_EXECUTABLE( discrete_double_integrator discrete_double_integrator.cpp )
TARGET_LINK_LIBRARIES( discrete_double_integrator ${ACADO_SHARED_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
SET_TARGET_PROPERTIES( discrete_double_integrator PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} )