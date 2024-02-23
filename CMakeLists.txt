cmake_minimum_required(VERSION 3.12)
project(fast_kinematics LANGUAGES CXX CUDA)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 14)

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g3 -Wall -Werror -fsized-deallocation -Wno-deprecated-declarations")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)

# eigen include directory
get_target_property(inc_dir Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
include_directories(${inc_dir})
# urdfdom include directory
include_directories(${urdfdom_INCLUDE_DIRS})
include_directories(include)

set_source_files_properties(tests/test_client.cpp PROPERTIES LANGUAGE CUDA)
add_executable(test_client tests/test_client.cpp)
set_target_properties(test_client PROPERTIES CUDA_ARCHITECTURES 52)
target_compile_options(test_client PRIVATE --expt-relaxed-constexpr -diag-suppress 20012,20236)
target_link_libraries(test_client ${urdfdom_LIBRARIES})

# include pybind11
set_source_files_properties(pybind/pybind_fast_kinematics.cpp PROPERTIES LANGUAGE CUDA)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/extern/pybind11")
pybind11_add_module(fast_kinematics pybind/pybind_fast_kinematics.cpp)
set_target_properties(fast_kinematics PROPERTIES CUDA_ARCHITECTURES 52)
target_compile_options(fast_kinematics PRIVATE --expt-relaxed-constexpr -diag-suppress 20012,20236)
target_link_libraries(fast_kinematics PRIVATE ${urdfdom_LIBRARIES})
