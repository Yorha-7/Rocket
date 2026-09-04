cmake_minimum_required(VERSION 3.10)
project(rocket_cpp VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

# The system's libzip CMake package is broken (references a missing
# zipcmp binary), so both zip and XML parsing are located via pkg-config
# instead of find_package.
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBZIP REQUIRED IMPORTED_TARGET libzip)
pkg_check_modules(TINYXML2 REQUIRED IMPORTED_TARGET tinyxml2)

add_executable(rocket_cpp
    src/main.cpp
    src/rocket_kinematics.cpp
    src/pitch_dynamics.cpp
    src/aerodynamics.cpp
    src/atmosphere.cpp
    src/mass_properties_model.cpp
    src/ork_archive.cpp
    src/ork_geometry.cpp
    src/ork_flightdata.cpp
    src/ork_mass_components.cpp
    src/ork_loader.cpp
    include/rocket_types.hpp
    include/rocket_kinematics.hpp
    include/aerodynamics.hpp
    include/mass_properties_model.hpp
    include/ork_archive.hpp
    include/ork_geometry.hpp
    include/ork_flightdata.hpp
    include/ork_mass_components.hpp
    include/ork_loader.hpp
)

target_include_directories(rocket_cpp PRIVATE include)

target_link_libraries(rocket_cpp PRIVATE
    Eigen3::Eigen
    PkgConfig::LIBZIP
    PkgConfig::TINYXML2
)
