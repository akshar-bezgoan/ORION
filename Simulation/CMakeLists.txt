cmake_minimum_required(VERSION 3.15)
project(ORION_Simulation_CPP)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Paths to kinematics_solver (adjust if folder structure differs)
set(KINEMATICS_SRC "../orion_ws/src/kinematics_solver/src")
set(KINEMATICS_INC "../orion_ws/src/kinematics_solver/include")

# Find dependencies
if(APPLE)
    # Add Homebrew prefix for Apple Silicon Macs
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")
endif()

find_package(Eigen3 REQUIRED)
find_package(SFML 3 REQUIRED COMPONENTS System Window Graphics)

# Define the executable and include kinematics source files directly
add_executable(gui_sim 
    gui_main.cpp 
    ${KINEMATICS_SRC}/forward_kinematics.cpp
    ${KINEMATICS_SRC}/robot_model.cpp 
    ${KINEMATICS_SRC}/inverse_kinematics.cpp
    ${KINEMATICS_SRC}/trajectory_optimiser.cpp
)

target_include_directories(gui_sim PRIVATE 
    ${KINEMATICS_INC} 
)

target_link_libraries(gui_sim SFML::Graphics SFML::Window SFML::System Eigen3::Eigen)