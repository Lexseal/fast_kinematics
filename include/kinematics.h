#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <urdf_parser/urdf_parser.h>

// data structure
// [translation (3 floats), rotation (4 floats), type (1 float), axis (3 floats)]

Eigen::Matrix<float, 7, 1> forward_kinematics(float *data, float *angs,
  size_t *num_joints_cum, size_t *num_of_active_joints_cum, size_t idx) {
  size_t end_idx = num_joints_cum[idx];
  size_t ang_idx = idx == 0 ? 0 : num_of_active_joints_cum[idx - 1];
  if (idx > 0) idx = num_joints_cum[idx - 1];
  Eigen::Quaternionf r = Eigen::Quaternionf::Identity();
  Eigen::Quaternionf t(0, data[idx], data[idx+1], data[idx+2]);
  idx += 3;
  while (idx < end_idx-11) {
    Eigen::Quaternionf rotation(data[idx], data[idx + 1], data[idx + 2], data[idx + 3]);
    idx += 4;
    float type = data[idx++];
    Eigen::Vector3f axis = Eigen::Vector3f(data[idx], data[idx + 1], data[idx + 2]);
    idx += 3;
    Eigen::Quaternionf nxt_translation(0, data[idx], data[idx + 1], data[idx + 2]);
    idx += 3;

    r = r * rotation;
    if (type == urdf::Joint::REVOLUTE) {
      // convert Eigen::AngleAxisd(joint_values[ang_idx++], walker->axis) to quaternion
      Eigen::Quaternionf change = Eigen::Quaternionf(Eigen::AngleAxisf(angs[ang_idx++], axis));
      r = r * change;
    } else if (type == urdf::Joint::PRISMATIC) {
      auto translation = angs[ang_idx++] * axis;
      Eigen::Quaternionf change = Eigen::Quaternionf(0, translation.x(), translation.y(), translation.z());
      nxt_translation.coeffs() += change.coeffs();
    }
    nxt_translation = r * nxt_translation * r.inverse();
    t.coeffs() += nxt_translation.coeffs();
  }

  Eigen::Quaternionf rotation(data[idx], data[idx + 1], data[idx + 2], data[idx + 3]);
  idx += 4;
  float type = data[idx++];
  Eigen::Vector3f axis = Eigen::Vector3f(data[idx], data[idx + 1], data[idx + 2]);
  idx += 3;
  Eigen::Quaternionf nxt_translation(0,0,0,0);

  r = r * rotation;
  if (type == urdf::Joint::REVOLUTE) {
    // convert Eigen::AngleAxisd(joint_values[ang_idx++], walker->axis) to quaternion
    Eigen::Quaternionf change = Eigen::Quaternionf(Eigen::AngleAxisf(angs[ang_idx++], axis));
    r = r * change;
  } else if (type == urdf::Joint::PRISMATIC) {
    auto translation = angs[ang_idx++] * axis;
    Eigen::Quaternionf change = Eigen::Quaternionf(0, translation.x(), translation.y(), translation.z());
    nxt_translation.coeffs() += change.coeffs();
  }
  nxt_translation = r * nxt_translation * r.inverse();
  t.coeffs() += nxt_translation.coeffs();
  
  Eigen::Matrix<float, 7, 1> result;
  result << t.x(), t.y(), t.z(), r.w(), r.x(), r.y(), r.z();
  return result;
}

Eigen::MatrixXf jacobian(float *data, float *angs,
  size_t *num_joints_cum, size_t *num_of_active_joints_cum, size_t idx) {
  size_t end_idx = num_joints_cum[idx];
  size_t start_ang_idx = idx == 0 ? 0 : num_of_active_joints_cum[idx - 1];
  size_t ang_idx = start_ang_idx;
  Eigen::MatrixXf result = Eigen::MatrixXf::Zero(6, num_of_active_joints_cum[idx] - ang_idx);
  if (idx > 0) idx = num_joints_cum[idx - 1];  // idx is getting reused here!
  Eigen::Quaternionf r = Eigen::Quaternionf::Identity();
  Eigen::Quaternionf t(0, data[idx], data[idx+1], data[idx+2]);
  idx += 3;
  while (idx < end_idx-11) {
    Eigen::Quaternionf rotation(data[idx], data[idx + 1], data[idx + 2], data[idx + 3]);
    idx += 4;
    float type = data[idx++];
    Eigen::Vector3f axis = Eigen::Vector3f(data[idx], data[idx + 1], data[idx + 2]);
    idx += 3;
    Eigen::Quaternionf nxt_translation(0, data[idx], data[idx + 1], data[idx + 2]);
    idx += 3;

    r = r * rotation;
    if (type == urdf::Joint::REVOLUTE) {
      Eigen::Quaternionf axis_quat(0, axis.x(), axis.y(), axis.z());
      axis_quat = r * axis_quat * r.inverse();
      result.block(3, ang_idx-start_ang_idx, 3, 1) = axis_quat.vec();
      result.block(0, ang_idx-start_ang_idx, 3, 1) = -axis_quat.vec().cross(t.vec());
      // convert Eigen::AngleAxisd(joint_values[ang_idx++], walker->axis) to quaternion
      Eigen::Quaternionf change = Eigen::Quaternionf(Eigen::AngleAxisf(angs[ang_idx++], axis));
      r = r * change;
    } else if (type == urdf::Joint::PRISMATIC) {
      Eigen::Quaternionf axis_quat(0, axis.x(), axis.y(), axis.z());
      axis_quat = r * axis_quat * r.inverse();
      result.block(0, ang_idx-start_ang_idx, 3, 1) = axis_quat.vec();
      Eigen::Vector3f translation = angs[ang_idx++] * axis;
      Eigen::Quaternionf change = Eigen::Quaternionf(0, translation.x(), translation.y(), translation.z());
      nxt_translation.coeffs() += change.coeffs();
    }
    nxt_translation = r * nxt_translation * r.inverse();
    t.coeffs() += nxt_translation.coeffs();
  }

  Eigen::Quaternionf rotation(data[idx], data[idx + 1], data[idx + 2], data[idx + 3]);
  idx += 4;
  float type = data[idx++];
  Eigen::Vector3f axis = Eigen::Vector3f(data[idx], data[idx + 1], data[idx + 2]);
  idx += 3;

  r = r * rotation;
  if (type == urdf::Joint::REVOLUTE) {
    Eigen::Quaternionf axis_quat(0, axis.x(), axis.y(), axis.z());
    axis_quat = r * axis_quat * r.inverse();
    result.block(3, ang_idx-start_ang_idx, 3, 1) = axis_quat.vec();
    result.block(0, ang_idx-start_ang_idx, 3, 1) = -axis_quat.vec().cross(t.vec());
  } else if (type == urdf::Joint::PRISMATIC) {
    Eigen::Quaternionf axis_quat(0, axis.x(), axis.y(), axis.z());
    axis_quat = r * axis_quat * r.inverse();
    result.block(0, ang_idx-start_ang_idx, 3, 1) = axis_quat.vec();
  }
  return result;
}
