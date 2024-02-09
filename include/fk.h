#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <urdf_parser/urdf_parser.h>

// data structure
// [type (1 float), axis (3 floats), rotation (4 floats), translation (3 floats)]

Eigen::Matrix<float, 7, 1> forward_kinematics(float* data, float* angs,
  size_t num_of_joints, size_t num_of_active_joints, size_t idx) {
  Eigen::Quaternionf r = Eigen::Quaternionf::Identity();
  Eigen::Quaternionf t = Eigen::Quaternionf(0, 0, 0, 0);
  size_t end_idx = 11*num_of_joints*(idx+1);
  size_t end_ang_idx = num_of_active_joints*(idx+1);
  idx = num_of_joints*idx;
  while (idx < end_idx) {
    float type = data[idx++];
    Eigen::Vector3f axis = Eigen::Vector3f(data[idx], data[idx + 1], data[idx + 2]);
    idx += 3;
    Eigen::Quaternionf rotation = Eigen::Quaternionf(data[idx], data[idx + 1], data[idx + 2], data[idx + 3]);
    idx += 4;
    Eigen::Quaternionf translation = Eigen::Quaternionf(0, data[idx], data[idx + 1], data[idx + 2]);
    idx += 3;
    if (type == urdf::Joint::REVOLUTE) {
      // convert Eigen::AngleAxisd(joint_values[--end_ang_idx], walker->axis) to quaternion
      Eigen::Quaternionf change = Eigen::Quaternionf(Eigen::AngleAxisf(angs[--end_ang_idx], axis));
      r = change * r;
      t = change * t * change.inverse();
    } else if (type == urdf::Joint::PRISMATIC) {
      auto translation = angs[--end_ang_idx] * axis;
      Eigen::Quaternionf change = Eigen::Quaternionf(0, translation.x(), translation.y(), translation.z());
      t.coeffs() += change.coeffs();
    }
    r = rotation * r;
    t = rotation * t * rotation.inverse();
    t.coeffs() += translation.coeffs();
  }
  Eigen::Matrix<float, 7, 1> result;
  result << t.x(), t.y(), t.z(), r.w(), r.x(), r.y(), r.z();
  return result;
}