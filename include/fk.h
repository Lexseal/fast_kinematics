#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <urdf_parser/urdf_parser.h>

// data structure
// [type (1 float), axis (3 floats), rotation (4 floats), translation (3 floats)]

Eigen::Matrix<float, 7, 1> forward_kinematics(std::vector<float> data, std::vector<float> angs) {
  Eigen::Quaternionf r = Eigen::Quaternionf::Identity();
  Eigen::Quaternionf t = Eigen::Quaternionf(0, 0, 0, 0);
  size_t num_of_joints = angs.size();
  size_t idx = 0;
  while (idx < data.size()) {
    float type = data[idx++];
    Eigen::Vector3f axis = Eigen::Vector3f(data[idx], data[idx + 1], data[idx + 2]);
    idx += 3;
    Eigen::Quaternionf rotation = Eigen::Quaternionf(data[idx], data[idx + 1], data[idx + 2], data[idx + 3]);
    idx += 4;
    Eigen::Quaternionf translation = Eigen::Quaternionf(0, data[idx], data[idx + 1], data[idx + 2]);
    idx += 3;
    std::cout << "type: " << type << std::endl;
    if (type == urdf::Joint::REVOLUTE) {
      // convert Eigen::AngleAxisd(joint_values[--num_of_joints], walker->axis) to quaternion
      Eigen::Quaternionf change = Eigen::Quaternionf(Eigen::AngleAxisf(angs[--num_of_joints], axis));
      r = change * r;
      t = change * t * change.inverse();
      std::cout << change.coeffs().transpose() << std::endl;
    } else if (type == urdf::Joint::PRISMATIC) {
      auto translation = angs[--num_of_joints] * axis;
      Eigen::Quaternionf change = Eigen::Quaternionf(0, translation.x(), translation.y(), translation.z());
      t.coeffs() += change.coeffs();
      std::cout << change.coeffs().transpose() << std::endl;
    }
    std::cout << rotation.coeffs().transpose() << std::endl;
    std::cout << translation.coeffs().transpose() << std::endl;
    r = rotation * r;
    t = rotation * t * rotation.inverse();
    t.coeffs() += translation.coeffs();
  }
  Eigen::Matrix<float, 7, 1> result;
  result << t.x(), t.y(), t.z(), r.w(), r.x(), r.y(), r.z();
  return result;
}