cmake_minimum_required(VERSION 3.12)
project(fast_kinematics LANGUAGES CXX CUDA)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g3 -Wall -Werror -fsized-deallocation -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -fPIC")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O2 -fPIC")
set(CMAKE_CUDA_RUNTIME_LIBRARY Static)

set(TORCH_CUDA_ARCH_LIST "5.0 6.1 8.0 8.6 8.9 9.0")

# find torch if no explicit path is given
if(NOT DEFINED CMAKE_PREFIX_PATH)
  if(NOT DEFINED PYTHON_EXECUTABLE)
    find_program(PYTHON_EXECUTABLE python3)
  endif()
  execute_process(
    COMMAND python3.10 -c "import torch;print(torch.utils.cmake_prefix_path)"
    OUTPUT_VARIABLE CMAKE_PREFIX_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
endif()

# torch requires pre-C++11 ABI
# may or may not be necessary here but doesn't hurt
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=0)

find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)
find_package(Torch REQUIRED)
find_library(TORCH_PYTHON_LIBRARY torch_python PATH "${TORCH_INSTALL_PREFIX}/lib")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

# urdfdom links statically to these two libraries, but for some reason,
# on my system I still need to link these two separately
find_package(TinyXML2 REQUIRED)
find_package(console_bridge REQUIRED)

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
target_link_libraries(parser PRIVATE urdfdom_model tinyxml2 console_bridge)

# then make core kinematics library
add_library(kinematics src/kinematics.cu src/fast_kinematics.cu)
set_target_properties(kinematics PROPERTIES CUDA_ARCHITECTURES "52;61")
target_compile_options(kinematics PRIVATE --expt-relaxed-constexpr -diag-suppress 20012,20236)
target_link_libraries(kinematics PRIVATE "${TORCH_LIBRARIES}" parser)

add_executable(test_client tests/test_client.cu)
set_target_properties(test_client PROPERTIES CUDA_ARCHITECTURES "52;61")
target_compile_options(test_client PRIVATE --expt-relaxed-constexpr -diag-suppress 20012,20236)
target_link_libraries(test_client PRIVATE kinematics)

# include pybind11
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/extern/pybind11")
pybind11_add_module(fast_kinematics pybind/pybind_fast_kinematics.cpp)
target_link_libraries(fast_kinematics PRIVATE "${TORCH_PYTHON_LIBRARY}" kinematics)
