#include <urdf_parser/urdf_parser.h>
#include <kinematics.h>
#include <cuda_quat.h>

__global__ void _forward_kinematics(float *data, float *angs, size_t *cum_data_idx,
  size_t *cum_active_joint_idx, float *result, size_t num_of_robots) {
  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_of_robots) return;
  size_t end_idx = cum_data_idx[idx];
  size_t ang_idx = idx == 0 ? 0 : cum_active_joint_idx[idx - 1];
  size_t data_idx = 0;
  if (idx > 0) data_idx = cum_data_idx[idx - 1];
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

__global__ void _jacobian(float *data, float *angs, size_t *cum_data_idx,
  size_t *cum_active_joint_idx, float *result, size_t num_of_robots) {
  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_of_robots) return;
  size_t end_idx = cum_data_idx[idx];
  size_t ang_idx = idx == 0 ? 0 : cum_active_joint_idx[idx - 1];
  size_t data_idx = 0;
  if (idx > 0) data_idx = cum_data_idx[idx - 1];
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
      result[6*ang_idx+3] = 0;
      result[6*ang_idx+4] = 0;
      result[6*ang_idx+5] = 0;
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
    result[6*ang_idx+3] = 0;
    result[6*ang_idx+4] = 0;
    result[6*ang_idx+5] = 0;
    float displacement = angs[ang_idx++];
    nxt_translation += axis_quat*displacement;
  }
  nxt_translation = r * nxt_translation * r.inverse();
  t += nxt_translation;
}

__global__ void _jacobian_mixed_frame(float *data, float *angs, size_t *cum_data_idx,
  size_t *cum_active_joint_idx, float *result, size_t num_of_robots) {
  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_of_robots) return;
  size_t end_idx = cum_data_idx[idx];
  size_t start_ang_idx = idx == 0 ? 0 : cum_active_joint_idx[idx - 1];
  size_t ang_idx = start_ang_idx;
  size_t data_idx = 0;
  if (idx > 0) data_idx = cum_data_idx[idx - 1];
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
      // temporarily store the translation in the jacobian matrix
      result[6*ang_idx+0] = t.x;
      result[6*ang_idx+1] = t.y;
      result[6*ang_idx+2] = t.z;
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
      result[6*ang_idx+3] = 0;
      result[6*ang_idx+4] = 0;
      result[6*ang_idx+5] = 0;
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
    result[6*ang_idx+0] = t.x;
    result[6*ang_idx+1] = t.y;
    result[6*ang_idx+2] = t.z;
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
    result[6*ang_idx+3] = 0;
    result[6*ang_idx+4] = 0;
    result[6*ang_idx+5] = 0;
    float displacement = angs[ang_idx++];
    nxt_translation += axis_quat*displacement;
  }
  nxt_translation = r * nxt_translation * r.inverse();
  t += nxt_translation;

  for (size_t i = start_ang_idx; i < ang_idx; ++i) {
    if (result[6*i+3] == 0 && result[6*i+4] == 0 && result[6*i+5] == 0) continue;
    float dx = result[6*i+4]*(t.z-result[6*i+2]) - result[6*i+5]*(t.y-result[6*i+1]);
    float dy = -result[6*i+3]*(t.z-result[6*i+2]) + result[6*i+5]*(t.x-result[6*i+0]);
    float dz = result[6*i+3]*(t.y-result[6*i+1]) - result[6*i+4]*(t.x-result[6*i+0]);
    result[6*i+0] = dx;
    result[6*i+1] = dy;
    result[6*i+2] = dz;
  }
}