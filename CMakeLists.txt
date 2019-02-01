cmake_minimum_required(VERSION 3.4)
project(DynamicsLib)

find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

if (NOT TARGET geometry)
    add_subdirectory(lib/geometry)
    include_directories(lib/geometry/include)
endif()

set(DYNAMICS_INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIRS}
    include
    lib/cvxgen
    lib/geometry/include
    )

include_directories(
    ${DYNAMICS_INCLUDE_DIRS}
    )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 1)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(Dynamics STATIC
    lib/geometry/include/geometry/quat.h
    lib/cvxgen/solver.h
    lib/cvxgen/solver.c
    lib/cvxgen/util.c
    lib/cvxgen/matrix_support.c
    lib/cvxgen/ldl.c
    include/types.hpp
    include/params.hpp
    include/fixedwing.hpp
    include/drone.hpp
    include/controller.hpp
    include/dynamics.hpp
    src/fixedwing.cpp
    src/drone.cpp
    src/controller.cpp
    src/dynamics.cpp
    )

if (${GTEST_FOUND})
        include_directories(include ${GTEST_INCLUDE_DIRS})
        set(TEST_NAME ${PROJECT_NAME}_UnitTest)
        add_executable(${TEST_NAME}
            test/dynamics_unittests.cpp
            )
        target_link_libraries(${TEST_NAME}
            ${GTEST_LIBRARIES}
            ${CATKIN_LIBRARIES}
            Dynamics
            gtest
            gtest_main
            pthread
            )
endif(${GTEST_FOUND})
