#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <torch/torch.h>
#include <torch/extension.h>
#include <fast_kinematics.h>

namespace py = pybind11;

std::string constructor_doc = R"(
    :param urdf_path: path to the URDF file
    :param num_of_robots: number of robots in the URDF file
    :param eef_name: name of the end-effector
    :param verbose: whether to print debug information
)";

std::string forward_kinematics_doc = R"(
    forward kinematics in numpy

    :param h_angs: joint angles in 1d numpy array [q1, q2, q3, ...]
    :param block_size: block size for parallel computation
    :return: end-effector pose in 1d numpy array [x, y, z, qw, qx, qy, qz, ...]
)";

std::string jacobian_mixed_frame_doc = R"(
    mixed frame means the velocity is expressed in the end-effector frame
    but the angular velocity is expressed in the base frame

    :param h_angs: joint angles in 1d numpy array [q1, q2, q3, ...]
    :param block_size: block size for parallel computation
    :return: Jacobian matrix in 2d numpy array
)";

std::string jacobian_world_frame_doc = R"(
    both the velocity and angular velocity are expressed in the base frame

    :param h_angs: joint angles in 1d numpy array [q1, q2, q3, ...]
    :param block_size: block size for parallel computation
    :return: Jacobian matrix in 2d numpy array
)";

std::string forward_kinematics_pytorch_doc = R"(
    forward kinematics in PyTorch

    :param t_angs: joint angles in 1d torch tensor [q1, q2, q3, ...]
    :param block_size: block size for parallel computation
    :return: end-effector pose in 1d torch tensor [x, y, z, qw, qx, qy, qz, ...]
)";

std::string jacobian_mixed_frame_pytorch_doc = R"(
    mixed frame means the velocity is expressed in the end-effector frame
    but the angular velocity is expressed in the base frame

    :param t_angs: joint angles in 1d torch tensor [q1, q2, q3, ...]
    :param block_size: block size for parallel computation
    :return: Jacobian matrix in 2d torch tensor
)";

std::string jacobian_world_frame_pytorch_doc = R"(
    both the velocity and angular velocity are expressed in the base frame

    :param t_angs: joint angles in 1d torch tensor [q1, q2, q3, ...]
    :param block_size: block size for parallel computation
    :return: Jacobian matrix in 2d torch tensor
)";

std::string get_num_of_active_joints_doc = R"(
    get the number of joints that are not fixed in a single kinematic chain,
)";

std::string get_num_of_joints_doc = R"(
    get the total number of joints in a single kinematic chain,
)";

std::string get_num_of_robots_doc = R"(
    get the number of parallel robots
)";


PYBIND11_MODULE(fast_kinematics, m) {
  m.doc() = "fast kinematics python bindings";

  auto PyFastKinematics = py::class_<FastKinematics>(m, "FastKinematics");
  PyFastKinematics
    .def(py::init<std::string, size_t, std::string, bool>(), py::arg("urdf_path"),
                                                             py::arg("num_of_robots"),
                                                             py::arg("eef_name"),
                                                             py::arg("verbose")=false,
                                                             constructor_doc.c_str())
    .def("forward_kinematics", &FastKinematics::forward_kinematics,
         py::arg("h_angs"), py::arg("block_size")=256, forward_kinematics_doc.c_str())
    .def("jacobian_mixed_frame", &FastKinematics::jacobian_mixed_frame,
         py::arg("h_angs"), py::arg("block_size")=256, jacobian_mixed_frame_doc.c_str())
    .def("jacobian_world_frame", &FastKinematics::jacobian_world_frame,
         py::arg("h_angs"), py::arg("block_size")=256, jacobian_world_frame_doc.c_str())
    .def("forward_kinematics_pytorch", [](FastKinematics &fk, torch::Tensor t_angs, size_t block_size=256) {
      auto options = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA, 0).pinned_memory(true);
      float *d_result = fk.forward_kinematics_raw_ptr(t_angs.data_ptr<float>(), block_size);
      long int sz = fk.get_num_of_robots() * 7;
      return torch::from_blob(d_result, {sz}, options);
    }, py::arg("t_angs"), py::arg("block_size")=256, forward_kinematics_pytorch_doc.c_str())
    .def("jacobian_mixed_frame_pytorch", [](FastKinematics &fk, torch::Tensor t_angs, size_t block_size=256) {
      auto options = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA, 0).pinned_memory(true);
      float *d_result = fk.jacobian_mixed_frame_raw_ptr(t_angs.data_ptr<float>(), block_size);
      long int sz = fk.get_num_of_robots() * fk.get_num_of_active_joints() * 6;
      return torch::from_blob(d_result, {sz}, options);
    }, py::arg("t_angs"), py::arg("block_size")=256, jacobian_mixed_frame_pytorch_doc.c_str())
    .def("jacobian_world_frame_pytorch", [](FastKinematics &fk, torch::Tensor t_angs, size_t block_size=256) {
      auto options = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA, 0).pinned_memory(true);
      float *d_result = fk.jacobian_world_frame_raw_ptr(t_angs.data_ptr<float>(), block_size);
      long int sz = fk.get_num_of_robots() * fk.get_num_of_active_joints() * 6;
      return torch::from_blob(d_result, {sz}, options);
    }, py::arg("t_angs"), py::arg("block_size")=256, jacobian_world_frame_pytorch_doc.c_str())
    .def("get_num_of_active_joints", &FastKinematics::get_num_of_active_joints, get_num_of_active_joints_doc.c_str())
    .def("get_num_of_joints", &FastKinematics::get_num_of_joints, get_num_of_joints_doc.c_str())
    .def("get_num_of_robots", &FastKinematics::get_num_of_robots, get_num_of_robots_doc.c_str());
}
