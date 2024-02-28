cmake_minimum_required(VERSION 3.12)
project(fast_kinematics LANGUAGES CXX CUDA)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 14)

# set(CMAKE_CXX_COMPILER "/usr/bin/g++-10")

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g3 -Wall -Werror -fsized-deallocation -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -fPIC")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O2 -fPIC")

find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)
find_package(Torch REQUIRED PATH "${TORCH_INSTALL_PREFIX}")
find_library(TORCH_PYTHON_LIBRARY torch_python PATH "${TORCH_INSTALL_PREFIX}/lib")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

# eigen include directory
get_target_property(inc_dir Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
include_directories(${inc_dir})
# urdfdom include directory
include_directories(${urdfdom_INCLUDE_DIRS})
# torch include directory
include_directories(${TORCH_INCLUDE_DIRS})
include_directories(include)

# first make the parsing library
add_library(parser src/parser.cpp)
target_link_libraries(parser PRIVATE ${urdfdom_LIBRARIES})

# then make core kinematics library
add_library(kinematics src/kinematics.cu src/fast_kinematics.cu)
set_target_properties(kinematics PROPERTIES CUDA_ARCHITECTURES 52)
target_compile_options(kinematics PRIVATE --expt-relaxed-constexpr -diag-suppress 20012,20236)
target_link_libraries(kinematics PRIVATE "${TORCH_LIBRARIES}" parser)

add_executable(test_client tests/test_client.cu)
set_target_properties(test_client PROPERTIES CUDA_ARCHITECTURES 52)
target_compile_options(test_client PRIVATE --expt-relaxed-constexpr -diag-suppress 20012,20236)
target_link_libraries(test_client PRIVATE kinematics)

# include pybind11
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/extern/pybind11")
pybind11_add_module(fast_kinematics pybind/pybind_fast_kinematics.cu)
set_target_properties(fast_kinematics PROPERTIES CUDA_ARCHITECTURES 52)
target_compile_options(fast_kinematics PRIVATE --expt-relaxed-constexpr -diag-suppress 20012,20236)
target_link_libraries(fast_kinematics PRIVATE "${TORCH_PYTHON_LIBRARY}" kinematics)
