cmake_minimum_required(VERSION 3.10)
project(rocket_cpp VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_executable(rocket_cpp
    src/main.cpp
    src/rocket_kinematics.cpp
    src/csv_loader.cpp
    include/rocket_types.hpp
    include/rocket_kinematics.hpp
    include/csv_loader.hpp
)

target_include_directories(rocket_cpp PRIVATE include)

target_link_libraries(rocket_cpp PRIVATE Eigen3::Eigen)