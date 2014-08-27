cmake_minimum_required (VERSION 2.6.0)
project (plot_sst)

# puts executables into bin directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

message("CMAKE_BUILD_TYPE is ${CMAKE_BUILD_TYPE}")

#---- Now check the compiler
if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall -Wno-sign-compare -g -std=c++0x")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# Add extra cmake files to module path
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# find_package fills in required includes/libraries
find_package(PythonLibs 2.7 REQUIRED)
find_package(Boost COMPONENTS iostreams python thread filesystem system numpy)
find_package(Eigen)

include_directories(${Boost_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${PYTHON_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

# all link directories
link_directories(${Boost_LIBRARY_DIRS} ${PYTHON_LIBRARIES})

# create executable and link to necessary subset of libraries
add_executable(plot_sst plot_sst.cpp)
target_link_libraries(plot_sst ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} ${MY_LIBRARIES}) 
