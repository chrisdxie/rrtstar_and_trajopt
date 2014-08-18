cmake_minimum_required (VERSION 2.6.0)
project (double_integrator_no_CD)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

#---- Now check the compiler
if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall -Wno-sign-compare -Wno-enum-compare -Wno-reorder -std=c++11")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# Add extra cmake files to module path
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT COMPONENTS system program_options)
find_package(Eigen)

set(TIMER_LIBRARIES "dl")

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${Boost_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})

add_executable(double-integrator-noCD-test  double_integrator_noCD_sqp.cpp double_integrator_QP_solver_noCD.c)
target_link_libraries(double-integrator-noCD-test ${Boost_LIBRARIES} ${TIMER_LIBRARIES})
