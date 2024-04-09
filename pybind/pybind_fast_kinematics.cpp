#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <torch/extension.h>
#include <fast_kinematics.h>

namespace py = pybind11;

PYBIND11_MODULE(fast_kinematics, m) {
  m.doc() = "fast kinematics python bindings";

  auto PyFastKinematics = py::class_<FastKinematics>(m, "FastKinematics");
  PyFastKinematics
    .def(py::init<std::string, size_t, std::string, bool>(), py::arg("urdf_path"),
                                                             py::arg("num_of_robots"),
                                                             py::arg("eef_name"),
                                                             py::arg("verbose")=false)
    .def("forward_kinematics", &FastKinematics::forward_kinematics,
         py::arg("h_angs"), py::arg("block_size")=256)
    .def("jacobian_mixed_frame", &FastKinematics::jacobian_mixed_frame,
         py::arg("h_angs"), py::arg("block_size")=256)
    .def("jacobian_world_frame", &FastKinematics::jacobian_world_frame,
         py::arg("h_angs"), py::arg("block_size")=256)
    .def("forward_kinematics_raw_ptr", &FastKinematics::forward_kinematics_raw_ptr,
         py::arg("t_angs"), py::arg("block_size")=256)
    .def("jacobian_mixed_frame_raw_ptr", &FastKinematics::jacobian_mixed_frame_raw_ptr,
         py::arg("h_angs"), py::arg("block_size")=256)
    .def("jacobian_world_frame_raw_ptr", &FastKinematics::jacobian_world_frame_raw_ptr,
         py::arg("h_angs"), py::arg("block_size")=256)
    .def("get_num_of_active_joints", &FastKinematics::get_num_of_active_joints)
    .def("get_num_of_joints", &FastKinematics::get_num_of_joints);
}
