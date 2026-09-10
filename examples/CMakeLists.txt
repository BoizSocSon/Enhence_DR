cmake_minimum_required(VERSION 3.16)

add_executable(rov_simulation_example rov_simulation_example.cpp)
target_link_libraries(rov_simulation_example PRIVATE nav_dynamics Eigen3::Eigen)
