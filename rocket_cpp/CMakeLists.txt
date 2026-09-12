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
    src/cli_args.cpp
    include/cli_args.hpp

    # sim -- the physics engine (ground-truth simulation)
    src/sim/rocket_kinematics.cpp
    src/sim/pitch_dynamics.cpp
    src/sim/yaw_dynamics.cpp
    src/sim/aerodynamics.cpp
    src/sim/atmosphere.cpp
    src/sim/mass_properties_model.cpp
    include/sim/rocket_types.hpp
    include/sim/rocket_kinematics.hpp
    include/sim/aerodynamics.hpp
    include/sim/mass_properties_model.hpp

    # ork -- rocket-parameter extraction from OpenRocket .ork design files
    src/ork/ork_archive.cpp
    src/ork/ork_geometry.cpp
    src/ork/ork_flightdata.cpp
    src/ork/ork_mass_components.cpp
    src/ork/ork_loader.cpp
    include/ork/ork_archive.hpp
    include/ork/ork_geometry.hpp
    include/ork/ork_flightdata.hpp
    include/ork/ork_mass_components.hpp
    include/ork/ork_loader.hpp

    # gnc -- Guidance/Navigation/Control: the "flight computer" stack
    # (sensors feed Navigation, which decides direction; TVC executes it)
    src/gnc/thrust_vector_control.cpp
    src/gnc/sensors.cpp
    src/gnc/navigation.cpp
    include/gnc/thrust_vector_control.hpp
    include/gnc/sensors.hpp
    include/gnc/navigation.hpp
)

target_include_directories(rocket_cpp PRIVATE include)

target_link_libraries(rocket_cpp PRIVATE
    Eigen3::Eigen
    PkgConfig::LIBZIP
    PkgConfig::TINYXML2
)
