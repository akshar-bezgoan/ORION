cmake_minimum_required(VERSION 3.10)

project(kinematics_solver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

include_directories(
    include
)

add_executable(solver
    src/test_fk.cpp
    src/forward_kinematics.cpp
    src/robot_model.cpp
)

target_link_libraries(solver Eigen3::Eigen)