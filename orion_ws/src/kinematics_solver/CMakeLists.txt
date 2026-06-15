cmake_minimum_required(VERSION 3.10)

project(kinematics_solver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

include_directories(
    include
)

add_executable(solver
    src/ik_test.cpp
    src/inverse_kinematics.cpp
    src/forward_kinematics.cpp
    src/robot_model.cpp
    src/trajectory_optimiser.cpp
)

add_executable(unit_test
    src/unit_test.cpp
    src/inverse_kinematics.cpp
    src/forward_kinematics.cpp
    src/robot_model.cpp
    src/trajectory_optimiser.cpp
)

target_link_libraries(solver Eigen3::Eigen)
target_link_libraries(unit_test Eigen3::Eigen)