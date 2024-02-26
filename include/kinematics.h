#pragma once

#include <urdf_parser/urdf_parser.h>
#include <cuda_quat.h>

// data structure
// [translation (3 floats), rotation (4 floats), type (1 float), axis (3 floats)]

__global__ void _forward_kinematics(float *data, float *angs, size_t *cum_data_idx,
  size_t *cum_active_joint_idx, float *result, size_t num_of_robots);

__global__ void _jacobian(float *data, float *angs, size_t *cum_data_idx,
  size_t *cum_active_joint_idx, float *result, size_t num_of_robots);

__global__ void _jacobian_mixed_frame(float *data, float *angs, size_t *cum_data_idx,
  size_t *cum_active_joint_idx, float *result, size_t num_of_robots);