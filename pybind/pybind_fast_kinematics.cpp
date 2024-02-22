#include <pybind11/pybind11.h>
// #include <fast_kinematics.h>

namespace py = pybind11;

int add(int i, int j) {
    return i + j;
}

PYBIND11_MODULE(fast_kinematics, m) {
  m.doc() = "fast kinematics python bindings";
  m.def("add", &add, "A function which adds two numbers");
}
