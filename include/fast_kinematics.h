#pragma once

#include <iostream>
#include <vector>
#include <unordered_map>
#include <kinematics.h>
#include <parser.h>

class FastKinematics {
public:
  FastKinematics(std::string urdf_path, size_t num_of_robots, std::string eef_name)
  : urdf_path(urdf_path)
  , num_of_robots(num_of_robots)
  , eef_name(eef_name) {
    root = Parser::parse(urdf_path);
    tip = Parser::find_link_parent(root, eef_name);
    num_of_active_joints = Parser::find_num_of_active_joints(tip);
    num_of_joints = Parser::find_num_of_joints(tip);
    Parser::prepare_repr(num_of_robots, tip, &h_data, &h_cum_data_idx);
    allocate_memory();
    copy_memory();
  }

  ~FastKinematics() {
    delete[] h_data;
    delete[] h_cum_active_joint_idx;
    delete[] h_cum_data_idx;
    delete[] h_result;
    cudaFree(d_data);
    cudaFree(d_angs);
    cudaFree(d_result);
    cudaFree(d_num_of_joints_cum);
    cudaFree(d_num_of_active_joints_cum);
  }

  float* forward_kinematics(float *h_angs, size_t block_size=256) {
    cudaMemcpy(d_angs, h_angs, h_cum_active_joint_idx[num_of_robots-1] * sizeof(float), cudaMemcpyHostToDevice);
    dim3 grid((num_of_robots+(block_size-1))/block_size);
    dim3 block(block_size);
    _forward_kinematics<<<grid, block>>>(d_data, d_angs, d_num_of_joints_cum, d_num_of_active_joints_cum, d_result, num_of_robots);
    cudaMemcpy(h_result, d_result, num_of_robots * 7 * sizeof(float), cudaMemcpyDeviceToHost);
    return h_result;
  }

  float* jacobian_mixed_frame(float *h_angs, size_t block_size=256) {
    cudaMemcpy(d_angs, h_angs, h_cum_active_joint_idx[num_of_robots-1] * sizeof(float), cudaMemcpyHostToDevice);
    dim3 grid((num_of_robots+(block_size-1))/block_size);
    dim3 block(block_size);
    _jacobian_mixed_frame<<<grid, block>>>(d_data, d_angs, d_num_of_joints_cum, d_num_of_active_joints_cum, d_result, num_of_robots);
    cudaMemcpy(h_result, d_result, h_cum_active_joint_idx[num_of_robots-1] * 6 * sizeof(float), cudaMemcpyDeviceToHost);
    return h_result;
  }

  float* jacobian_world_frame(float *h_angs, size_t block_size=256) {
    cudaMemcpy(d_angs, h_angs, h_cum_active_joint_idx[num_of_robots-1] * sizeof(float), cudaMemcpyHostToDevice);
    dim3 grid((num_of_robots+(block_size-1))/block_size);
    dim3 block(block_size);
    _jacobian<<<grid, block>>>(d_data, d_angs, d_num_of_joints_cum, d_num_of_active_joints_cum, d_result, num_of_robots);
    cudaMemcpy(h_result, d_result, h_cum_active_joint_idx[num_of_robots-1] * 6 * sizeof(float), cudaMemcpyDeviceToHost);
    return h_result;
  }

  size_t get_num_of_active_joints() {
    return num_of_active_joints;
  }

  size_t get_num_of_joints() {
    return num_of_joints;
  }

private:
  JointTreePtr root, tip;
  size_t num_of_active_joints, num_of_joints;
  float *h_data, *h_result;
  float *d_data, *d_angs, *d_result;
  size_t *h_cum_data_idx, *d_num_of_joints_cum;
  size_t *h_cum_active_joint_idx, *d_num_of_active_joints_cum;
  size_t num_of_robots;
  std::string eef_name;
  std::string urdf_path;

  void allocate_memory() {
    size_t result_max_size = std::max(7*num_of_robots, 6*num_of_robots*num_of_active_joints);
    h_cum_active_joint_idx = new size_t[num_of_robots];
    h_cum_active_joint_idx[0] = num_of_active_joints;
    for (size_t i = 1; i < num_of_robots; ++i) {
      h_cum_active_joint_idx[i] = h_cum_active_joint_idx[i - 1] + num_of_active_joints;
    }
    h_result = new float[result_max_size];
    cudaMalloc(&d_data, h_cum_data_idx[num_of_robots-1] * sizeof(float));
    cudaMalloc(&d_angs, h_cum_active_joint_idx[num_of_robots-1] * sizeof(float));
    cudaMalloc(&d_result, result_max_size * sizeof(float));
    cudaMalloc(&d_num_of_joints_cum, num_of_robots * sizeof(size_t));
    cudaMalloc(&d_num_of_active_joints_cum, num_of_robots * sizeof(size_t));
  }

  void copy_memory() {
    cudaMemcpy(d_data, h_data, h_cum_data_idx[num_of_robots-1] * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_num_of_joints_cum, h_cum_data_idx, num_of_robots * sizeof(size_t), cudaMemcpyHostToDevice);
    cudaMemcpy(d_num_of_active_joints_cum, h_cum_active_joint_idx, num_of_robots * sizeof(size_t), cudaMemcpyHostToDevice);
  }
};
