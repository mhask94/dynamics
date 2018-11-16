cmake_minimum_required(VERSION 3.4)
project(DynamicsLib)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    rosflight_msgs
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
    )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 1)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

IF (NOT TARGET gtest)
    # Prevent GoogleTest from overriding our compiler/linker options
    # when building with Visual Studio
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    add_subdirectory(../googletest)
ENDIF()

add_library(Dynamics STATIC
    include/rk4.hpp
    src/rk4.cpp
    )

set(TEST_NAME ${PROJECT_NAME}_UnitTest)

add_executable(${TEST_NAME}
    src/dynamics_unittests.cpp
    )

target_link_libraries(${TEST_NAME}
    ${catkin_LIBRARIES}
    Dynamics
    gtest
    gtest_main
    )
