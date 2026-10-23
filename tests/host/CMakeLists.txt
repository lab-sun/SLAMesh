cmake_minimum_required(VERSION 3.10)
project(slamesh_host_tests LANGUAGES CXX)

find_package(Eigen3 REQUIRED NO_MODULE)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(SANITIZE "Enable address and undefined behavior sanitizers" OFF)
option(FUZZ "Build the libFuzzer harness (Clang only)" OFF)

add_executable(test_point_matrix test_point_matrix.cpp)
target_link_libraries(test_point_matrix PRIVATE Eigen3::Eigen)
target_compile_options(test_point_matrix PRIVATE -Wall -Wextra)

add_executable(stress_point_matrix stress_point_matrix.cpp)
target_link_libraries(stress_point_matrix PRIVATE Eigen3::Eigen)
target_compile_options(stress_point_matrix PRIVATE -Wall -Wextra)

if(SANITIZE)
    target_compile_options(test_point_matrix PRIVATE
        -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_libraries(test_point_matrix PRIVATE -fsanitize=address,undefined)
    target_compile_options(stress_point_matrix PRIVATE
        -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_libraries(stress_point_matrix PRIVATE -fsanitize=address,undefined)
endif()

enable_testing()
add_test(NAME test_point_matrix COMMAND test_point_matrix)
add_test(NAME stress_point_matrix COMMAND stress_point_matrix 1 20000)

# libFuzzer needs Clang. Other compilers skip this target.
if(FUZZ)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        add_executable(fuzz_point_matrix fuzz_point_matrix.cpp)
        target_link_libraries(fuzz_point_matrix PRIVATE Eigen3::Eigen)
        target_compile_options(fuzz_point_matrix PRIVATE
            -Wall -Wextra
            -fsanitize=fuzzer,address,undefined
            -fno-omit-frame-pointer)
        target_link_libraries(fuzz_point_matrix PRIVATE
            -fsanitize=fuzzer,address,undefined)
    else()
        message(STATUS "FUZZ=ON but the compiler is not Clang, so fuzz_point_matrix is not built")
    endif()
endif()
