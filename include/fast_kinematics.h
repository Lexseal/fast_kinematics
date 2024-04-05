#pragma once

#include <string>
#include <Eigen/Core>
#include <parser.h>

class FastKinematics {
public:
  FastKinematics(std::string urdf_path, size_t num_of_robots, std::string eef_name, bool verbose=false);
  ~FastKinematics();

  Eigen::Ref<Eigen::VectorXf> forward_kinematics(Eigen::Ref<Eigen::VectorXf> h_angs, size_t block_size=256);
  Eigen::Ref<Eigen::VectorXf> jacobian_mixed_frame(Eigen::Ref<Eigen::VectorXf> h_angs, size_t block_size=256);
  Eigen::Ref<Eigen::VectorXf> jacobian_world_frame(Eigen::Ref<Eigen::VectorXf> h_angs, size_t block_size=256);
  float *forward_kinematics_pytorch(float *t_angs, size_t block_size=256);
  float *jacobian_mixed_frame_pytorch(float *t_angs, size_t block_size=256);
  float *jacobian_world_frame_pytorch(float *t_angs, size_t block_size=256);
  size_t get_num_of_active_joints();
  size_t get_num_of_joints();

private:
  JointTreePtr root, tip;
  size_t num_of_active_joints, num_of_joints;
  float *h_data;
  Eigen::VectorXf h_fk_result, h_jac_result;
  float *d_data, *d_angs, *d_result;
  size_t *h_cum_data_idx, *d_num_of_joints_cum;
  size_t *h_cum_active_joint_idx, *d_num_of_active_joints_cum;
  size_t num_of_robots;
  std::string eef_name;
  std::string urdf_path;

  void allocate_memory();
  void copy_memory();
};
