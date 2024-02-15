cmake_minimum_required(VERSION 3.12)
project(fast_kinematics LANGUAGES CXX CUDA)
set(CMAKE_CXX_STANDARD 17)

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
target_compile_options(test_client PRIVATE --expt-relaxed-constexpr -diag-suppress 20012)
target_link_libraries(test_client ${urdfdom_LIBRARIES})
