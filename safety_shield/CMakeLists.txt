cmake_minimum_required(VERSION 3.0.2)
project(safety_shield)

## Compile as C++17
add_compile_options(-std=c++17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

##Find XBOT2
find_package(xbot2 REQUIRED)
find_package(roscpp REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  geometry_msgs
  visualization_msgs
  gazebo_ros
  concert_msgs
  trajectory_msgs
)


## Find catkin macros and libraries
#find_package(Eigen3 3.4 REQUIRED NO_MODULE)
# INCLUDE_DIRECTORIES ( "$ENV{EIGEN3_INCLUDE_DIR}" )
INCLUDE_DIRECTORIES ( ${EIGEN3_INCLUDE_DIR})
# Find Python
# IF YOU DON'T WANT TO USE CONDA, DISABLE THESE THREE LINES
#set(PYTHON_EXECUTABLE "$ENV{CONDA_PREFIX}/bin/python")
#set(PYTHON_INCLUDE_DIR "$ENV{CONDA_PREFIX}/include/python3.8")
#set(PYTHON_LIBRARY "$ENV{CONDA_PREFIX}/lib/libpython3.8.so")
#set(PYTHON_LIBRARY_DEBUG "$ENV{CONDA_PREFIX}/lib/libpython3.8.so")
#find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/safety_shield.cc 
    src/long_term_traj.cc 
    src/motion.cc 
    src/path.cc 
    src/robot_reach.cc 
    src/human_reach.cc
    src/pedestrian_reach.cc
    src/verify_iso.cc 
    #src/advanced_verify_iso.cc
    )

# Add external subdirectories
add_subdirectory(external/pybind11)
add_subdirectory(external/SaRA/ReachLib)
add_subdirectory(external/LongTermPlanner)

#add_subdirectory(external/spdlog)
#add_subdirectory(external/yaml-cpp)

target_include_directories(${PROJECT_NAME}
    PUBLIC external/pybind11/include
    PUBLIC external/SaRA/ReachLib/include
    PUBLIC external/LongTermPlanner/include
    # PUBLIC /home/user/concert_ws/install/include
#    PUBLIC external/spdlog/include
#    PUBLIC external/yaml-cpp/include
)

target_link_directories(${PROJECT_NAME}
    PUBLIC external/ReachLib/src
    PUBLIC external/LongTermPlanner/src
#    PUBLIC external/spdlog/src
#    PUBLIC external/yaml-cpp/src
)

target_link_libraries(${PROJECT_NAME} 
    PUBLIC SaRA 
    PUBLIC long_term_planner 
    PUBLIC yaml-cpp
    #PUBLIC ${catkin_EXPORTED_TARGETS}
)


## sara_shield_xbot2 ##
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sara_shield_xbot2
#  CATKIN_DEPENDS roscpp tf2 tf2_ros
#)

#include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
#)

add_xbot2_plugin(sara_shield_xbot2 src/sara_shield_xbot2.cc)

target_include_directories(sara_shield_xbot2
  PUBLIC /opt/ros/noetic/lib
  )

target_link_libraries(sara_shield_xbot2
	PUBLIC ${PROJECT_NAME}
	PUBLIC XBotInterface::XBotInterface
	PUBLIC ${roscpp_LIBRARIES}
	PUBLIC ${catkin_LIBRARIES}
  )

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
#install(TARGETS ${PROJECT_NAME} DESTINATION /opt/xbot/lib)
install(TARGETS sara_shield_xbot2 DESTINATION lib)
#set_target_properties(sara_shield_xbot2 PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(debug src/debug.cc)
target_link_libraries(debug ${PROJECT_NAME} long_term_planner yaml-cpp)


#############
## GTest ##
#############
enable_testing()
find_package(GTest REQUIRED)
include(GoogleTest)

add_executable(long_term_traj_tests tests/src/long_term_traj_tests.cc)
target_include_directories(long_term_traj_tests PUBLIC tests/include)
target_link_libraries(long_term_traj_tests ${PROJECT_NAME} GTest::GTest GTest::Main)
add_executable(human_reach_tests tests/src/human_reach_tests.cc)
target_include_directories(human_reach_tests PUBLIC tests/include)
target_link_libraries(human_reach_tests ${PROJECT_NAME} GTest::GTest GTest::Main)
add_executable(motion_tests tests/src/motion_tests.cc)
target_include_directories(motion_tests PUBLIC tests/include)
target_link_libraries(motion_tests ${PROJECT_NAME} GTest::GTest GTest::Main)
add_executable(path_tests tests/src/path_tests.cc)
target_include_directories(path_tests PUBLIC tests/include)
target_link_libraries(path_tests ${PROJECT_NAME} GTest::GTest GTest::Main)
add_executable(robot_reach_tests tests/src/robot_reach_tests.cc)
target_include_directories(robot_reach_tests PUBLIC tests/include)
target_link_libraries(robot_reach_tests ${PROJECT_NAME} GTest::GTest GTest::Main)
add_executable(verify_iso_tests tests/src/verify_iso_tests.cc)
target_include_directories(verify_iso_tests PUBLIC tests/include)
target_link_libraries(verify_iso_tests ${PROJECT_NAME} GTest::GTest GTest::Main)
add_executable(safety_shield_tests tests/src/safety_shield_tests.cc)
target_include_directories(safety_shield_tests PUBLIC tests/include)
target_link_libraries(safety_shield_tests ${PROJECT_NAME} GTest::GTest GTest::Main)
gtest_discover_tests(long_term_traj_tests)
gtest_discover_tests(human_reach_tests)
gtest_discover_tests(motion_tests)
gtest_discover_tests(path_tests)
gtest_discover_tests(robot_reach_tests)
gtest_discover_tests(verify_iso_tests)
gtest_discover_tests(safety_shield_tests)

#catkin_add_gtest(verify_iso_test tests/capsule_tests.cc tests/verify_iso_fixture.h)
#target_link_libraries(verify_iso_test ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)

#catkin_add_gtest(long_term_traj_tests tests/long_term_traj_tests.cc tests/long_term_traj_fixture.h)
#target_link_libraries(long_term_traj_tests ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)

##############
## Bindings ##
##############
pybind11_add_module(safety_shield_py MODULE bindings/safety_shield_bindings.cpp)
target_link_libraries(safety_shield_py PRIVATE ${PROJECT_NAME} long_term_planner yaml-cpp)