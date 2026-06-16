cmake_minimum_required(VERSION 3.10)

project(kinematics_solver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

include_directories(
    include
)

add_executable(solver
    src/main.cpp
    src/inverse_kinematics.cpp
    src/forward_kinematics.cpp
    src/robot_model.cpp
    src/trajectory_optimiser.cpp
)

add_executable(ik_test
    src/ik_test.cpp
    src/inverse_kinematics.cpp
    src/forward_kinematics.cpp
    src/robot_model.cpp
)

add_executable(trajectory_optimiser_test
    src/trajectory_optimiser_test.cpp
    src/trajectory_optimiser.cpp
    src/robot_model.cpp
    src/forward_kinematics.cpp
)

add_executable(pwm_output_test
    src/pwm_output_test.cpp
    src/pwm_output.cpp
    src/robot_model.cpp
)

target_link_libraries(solver Eigen3::Eigen)
target_link_libraries(ik_test Eigen3::Eigen)
target_link_libraries(trajectory_optimiser_test Eigen3::Eigen)
target_link_libraries(pwm_output_test Eigen3::Eigen)
