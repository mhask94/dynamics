cmake_minimum_required(VERSION 3.4)
project(DynamicsLib)

find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

if (NOT TARGET geometry)
    add_subdirectory(include/geometry)
    include_directories(include/geometry/include)
endif()

set(DYNAMICS_INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIRS}
    include
    include/cvxgen
    include/geometry/include
    )

include_directories(
    ${DYNAMICS_INCLUDE_DIRS}
    )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 1)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(Dynamics STATIC
    include/geometry/include/geometry/quat.h
    include/cvxgen/solver.h
    include/cvxgen/solver.c
    include/cvxgen/util.c
    include/cvxgen/matrix_support.c
    include/cvxgen/ldl.c
    include/dynamics/types.hpp
    include/dynamics/params.hpp
    include/dynamics/fixedwing.hpp
    include/dynamics/drone.hpp
    include/dynamics/controller.hpp
    include/dynamics/dynamics.hpp
    include/dynamics/input.hpp
    include/dynamics/winddynamics.hpp
    src/fixedwing.cpp
    src/drone.cpp
    src/controller.cpp
    src/dynamics.cpp
    src/winddynamics.cpp
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
