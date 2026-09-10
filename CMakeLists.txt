cmake_minimum_required(VERSION 3.16)
project(nav_dynamics VERSION 1.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler warning flags
if(MSVC)
    add_compile_options(/W4 /EHsc)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Find yaml-cpp for configuration parsing
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

# Core Library Sources
set(NAV_DYNAMICS_SOURCES
    src/dof_config.cpp
    src/dof_transformer.cpp
    src/config_loader.cpp
    src/mass_matrix.cpp
    src/coriolis_matrix.cpp
    src/damping_matrix.cpp
    src/restoring_force.cpp
    src/thruster_allocation.cpp
    src/dynamic_model.cpp
    src/ros_adapter.cpp
)

# Core Library Headers
set(NAV_DYNAMICS_HEADERS
    include/nav_dynamics/types.hpp
    include/nav_dynamics/dof_config.hpp
    include/nav_dynamics/dof_transformer.hpp
    include/nav_dynamics/config_loader.hpp
    include/nav_dynamics/mass_matrix.hpp
    include/nav_dynamics/coriolis_matrix.hpp
    include/nav_dynamics/damping_matrix.hpp
    include/nav_dynamics/restoring_force.hpp
    include/nav_dynamics/thruster_allocation.hpp
    include/nav_dynamics/dynamic_model.hpp
    include/nav_dynamics/ros_adapter.hpp
)

add_library(nav_dynamics STATIC
    ${NAV_DYNAMICS_SOURCES}
    ${NAV_DYNAMICS_HEADERS}
)

target_include_directories(nav_dynamics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(nav_dynamics PUBLIC
    Eigen3::Eigen
    yaml-cpp
)

# Enable testing
enable_testing()
add_subdirectory(tests)

# Build examples
add_subdirectory(examples)
