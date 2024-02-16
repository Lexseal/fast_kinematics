#pragma once

#include <vector>
#include <urdf_parser/urdf_parser.h>
#include <cuda_quat.h>

// data structure
// [translation (3 floats), rotation (4 floats), type (1 float), axis (3 floats)]

__global__ void forward_kinematics(float *data, float *angs, size_t *num_data_cum,
                        size_t *num_of_active_joints_cum, float *result, size_t num_of_robots) {
  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_of_robots) return;
  size_t end_idx = num_data_cum[idx];
  size_t ang_idx = idx == 0 ? 0 : num_of_active_joints_cum[idx - 1];
  size_t data_idx = 0;
  if (idx > 0) data_idx = num_data_cum[idx - 1];
  Quaternion r = Quaternion::Identity();
  Quaternion t(0, data[data_idx], data[data_idx+1], data[data_idx+2]);
  data_idx += 3;
  while (data_idx < end_idx-11) {
    Quaternion rotation(data[data_idx], data[data_idx + 1], data[data_idx + 2], data[data_idx + 3]);
    data_idx += 4;
    float type = data[data_idx++];
    float axis[3] = {data[data_idx], data[data_idx + 1], data[data_idx + 2]};
    data_idx += 3;
    Quaternion nxt_translation(0, data[data_idx], data[data_idx + 1], data[data_idx + 2]);
    data_idx += 3;

    r = r * rotation;
    if (type == urdf::Joint::REVOLUTE) {
      Quaternion change = Quaternion::FromAngleAxis(angs[ang_idx++], axis[0], axis[1], axis[2]);
      r = r * change;
    } else if (type == urdf::Joint::PRISMATIC) {
      float displacement = angs[ang_idx++];
      Quaternion change(0, displacement*axis[0], displacement*axis[1], displacement*axis[2]);
      nxt_translation += change;
    }
    nxt_translation = r * nxt_translation * r.inverse();
    t += nxt_translation;
  }

  Quaternion rotation(data[data_idx], data[data_idx + 1], data[data_idx + 2], data[data_idx + 3]);
  data_idx += 4;
  float type = data[data_idx++];
  float axis[3] = {data[data_idx], data[data_idx + 1], data[data_idx + 2]};
  data_idx += 3;
  Quaternion nxt_translation(0,0,0,0);

  r = r * rotation;
  if (type == urdf::Joint::REVOLUTE) {
    Quaternion change = Quaternion::FromAngleAxis(angs[ang_idx++], axis[0], axis[1], axis[2]);
    r = r * change;
  } else if (type == urdf::Joint::PRISMATIC) {
    float displacement = angs[ang_idx++];
    Quaternion change(0, displacement*axis[0], displacement*axis[1], displacement*axis[2]);
    nxt_translation += change;
  }
  nxt_translation = r * nxt_translation * r.inverse();
  t += nxt_translation;
  
  result[idx*7+0] = t.x;
  result[idx*7+1] = t.y;
  result[idx*7+2] = t.z;
  result[idx*7+3] = r.w;
  result[idx*7+4] = r.x;
  result[idx*7+5] = r.y;
  result[idx*7+6] = r.z;
}

__global__ void jacobian(float *data, float *angs, size_t *num_data_cum,
              size_t *num_of_active_joints_cum, float *result, size_t num_of_robots) {
  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_of_robots) return;
  size_t end_idx = num_data_cum[idx];
  size_t ang_idx = idx == 0 ? 0 : num_of_active_joints_cum[idx - 1];
  size_t data_idx = 0;
  if (idx > 0) data_idx = num_data_cum[idx - 1];
  Quaternion r = Quaternion::Identity();
  Quaternion t(0, data[data_idx], data[data_idx+1], data[data_idx+2]);
  data_idx += 3;
  while (data_idx < end_idx-11) {
    Quaternion rotation(data[data_idx], data[data_idx + 1], data[data_idx + 2], data[data_idx + 3]);
    data_idx += 4;
    float type = data[data_idx++];
    float axis[3] = {data[data_idx], data[data_idx + 1], data[data_idx + 2]};
    data_idx += 3;
    Quaternion nxt_translation(0, data[data_idx], data[data_idx + 1], data[data_idx + 2]);
    data_idx += 3;

    r = r * rotation;
    if (type == urdf::Joint::REVOLUTE) {
      Quaternion axis_quat(0, axis[0], axis[1], axis[2]);
      axis_quat = r * axis_quat * r.inverse();
      result[6*ang_idx+0] = -axis_quat.y*t.z + axis_quat.z*t.y;
      result[6*ang_idx+1] = axis_quat.x*t.z - axis_quat.z*t.x;
      result[6*ang_idx+2] = -axis_quat.x*t.y + axis_quat.y*t.x;
      result[6*ang_idx+3] = axis_quat.x;
      result[6*ang_idx+4] = axis_quat.y;
      result[6*ang_idx+5] = axis_quat.z;
      Quaternion change = Quaternion::FromAngleAxis(angs[ang_idx++], axis[0], axis[1], axis[2]);
      r = r * change;
    } else if (type == urdf::Joint::PRISMATIC) {
      Quaternion axis_quat(0, axis[0], axis[1], axis[2]);
      Quaternion rotated_axis = r * axis_quat * r.inverse();
      result[6*ang_idx+0] = rotated_axis.x;
      result[6*ang_idx+1] = rotated_axis.y;
      result[6*ang_idx+2] = rotated_axis.z;
      float displacement = angs[ang_idx++];
      nxt_translation += axis_quat*displacement;
    }
    nxt_translation = r * nxt_translation * r.inverse();
    t += nxt_translation;
  }

  Quaternion rotation(data[data_idx], data[data_idx + 1], data[data_idx + 2], data[data_idx + 3]);
  data_idx += 4;
  float type = data[data_idx++];
  float axis[3] = {data[data_idx], data[data_idx + 1], data[data_idx + 2]};
  data_idx += 3;
  Quaternion nxt_translation(0,0,0,0);

  r = r * rotation;
  if (type == urdf::Joint::REVOLUTE) {
    Quaternion axis_quat(0, axis[0], axis[1], axis[2]);
    axis_quat = r * axis_quat * r.inverse();
    result[6*ang_idx+0] = -axis_quat.y*t.z + axis_quat.z*t.y;
    result[6*ang_idx+1] = axis_quat.x*t.z - axis_quat.z*t.x;
    result[6*ang_idx+2] = -axis_quat.x*t.y + axis_quat.y*t.x;
    result[6*ang_idx+3] = axis_quat.x;
    result[6*ang_idx+4] = axis_quat.y;
    result[6*ang_idx+5] = axis_quat.z;
    Quaternion change = Quaternion::FromAngleAxis(angs[ang_idx++], axis[0], axis[1], axis[2]);
    r = r * change;
  } else if (type == urdf::Joint::PRISMATIC) {
    Quaternion axis_quat(0, axis[0], axis[1], axis[2]);
    Quaternion rotated_axis = r * axis_quat * r.inverse();
    result[6*ang_idx+0] = rotated_axis.x;
    result[6*ang_idx+1] = rotated_axis.y;
    result[6*ang_idx+2] = rotated_axis.z;
    float displacement = angs[ang_idx++];
    nxt_translation += axis_quat*displacement;
  }
  nxt_translation = r * nxt_translation * r.inverse();
  t += nxt_translation;
}
