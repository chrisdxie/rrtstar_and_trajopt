cmake_minimum_required (VERSION 2.6.0)
project (cartpole)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

#---- Now check the compiler
if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall -Wno-sign-compare -Wno-enum-compare -Wno-reorder -std=c++11")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# Add extra cmake files to module path
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT COMPONENTS system program_options)
find_package(Eigen)

set(TIMER_LIBRARIES "dl;rt")

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${Boost_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})

#add_executable(cartpole-test test_dynamics.cpp)
#target_link_libraries(cartpole-test ${Boost_LIBRARIES} ${TIMER_LIBRARIES})

add_executable(cartpole-test cartpole_sqp.cpp cartpole_QP_solver.c)
target_link_libraries(cartpole-test ${Boost_LIBRARIES} ${TIMER_LIBRARIES})

