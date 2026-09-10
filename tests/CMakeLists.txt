cmake_minimum_required(VERSION 3.16)

# Test executables
macro(add_nav_test test_name)
    add_executable(${test_name} ${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE nav_dynamics Eigen3::Eigen yaml-cpp)
    add_test(NAME ${test_name} COMMAND ${test_name})
endmacro()

add_nav_test(test_mass_matrix)
add_nav_test(test_coriolis_matrix)
add_nav_test(test_damping_matrix)
add_nav_test(test_restoring_force)
add_nav_test(test_dof_reduction)
add_nav_test(test_dynamic_model)
add_nav_test(test_dof_transformer)
add_nav_test(test_config_loader)
