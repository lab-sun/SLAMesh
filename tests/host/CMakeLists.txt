cmake_minimum_required(VERSION 3.10)
project(slamesh_host_tests LANGUAGES CXX)

find_package(Eigen3 REQUIRED NO_MODULE)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(SANITIZE "Enable address and undefined behavior sanitizers" OFF)

add_executable(test_point_matrix test_point_matrix.cpp)
target_link_libraries(test_point_matrix PRIVATE Eigen3::Eigen)
target_compile_options(test_point_matrix PRIVATE -Wall -Wextra)

if(SANITIZE)
    target_compile_options(test_point_matrix PRIVATE
        -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_libraries(test_point_matrix PRIVATE -fsanitize=address,undefined)
endif()

enable_testing()
add_test(NAME test_point_matrix COMMAND test_point_matrix)
