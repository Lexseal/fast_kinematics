#include <fast_kinematics.h>
#include <kinematics.h>

FastKinematics::FastKinematics(std::string urdf_path, size_t num_of_robots, std::string eef_name)
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

FastKinematics::~FastKinematics() {
  delete[] h_data;
  delete[] h_cum_active_joint_idx;
  delete[] h_cum_data_idx;
  cudaFree(d_data);
  cudaFree(d_angs);
  cudaFree(d_result);
  cudaFree(d_num_of_joints_cum);
  cudaFree(d_num_of_active_joints_cum);
}

Eigen::Ref<Eigen::VectorXf> FastKinematics::forward_kinematics(Eigen::Ref<Eigen::VectorXf> h_angs, size_t block_size) {
  assert(h_angs.size() == h_cum_active_joint_idx[num_of_robots-1]);  // check if the number of joints is correct
  cudaMemcpy(d_angs, h_angs.data(), h_cum_active_joint_idx[num_of_robots-1] * sizeof(float), cudaMemcpyHostToDevice);
  dim3 grid((num_of_robots+(block_size-1))/block_size);
  dim3 block(block_size);
  _forward_kinematics<<<grid, block>>>(d_data, d_angs, d_num_of_joints_cum, d_num_of_active_joints_cum, d_result, num_of_robots);
  cudaMemcpy(h_fk_result.data(), d_result, 7*num_of_robots * sizeof(float), cudaMemcpyDeviceToHost);
  return h_fk_result;
}

Eigen::Ref<Eigen::VectorXf> FastKinematics::jacobian_mixed_frame(Eigen::Ref<Eigen::VectorXf> h_angs, size_t block_size) {
  assert(h_angs.size() == h_cum_active_joint_idx[num_of_robots-1]);  // check if the number of joints is correct
  cudaMemcpy(d_angs, h_angs.data(), h_cum_active_joint_idx[num_of_robots-1] * sizeof(float), cudaMemcpyHostToDevice);
  dim3 grid((num_of_robots+(block_size-1))/block_size);
  dim3 block(block_size);
  _jacobian_mixed_frame<<<grid, block>>>(d_data, d_angs, d_num_of_joints_cum, d_num_of_active_joints_cum, d_result, num_of_robots);
  cudaMemcpy(h_jac_result.data(), d_result,
              h_cum_active_joint_idx[num_of_robots-1] * 6 * sizeof(float), cudaMemcpyDeviceToHost);
  return h_jac_result;
}

Eigen::Ref<Eigen::VectorXf> FastKinematics::jacobian_world_frame(Eigen::Ref<Eigen::VectorXf> h_angs, size_t block_size) {
  assert(h_angs.size() == h_cum_active_joint_idx[num_of_robots-1]);  // check if the number of joints is correct
  cudaMemcpy(d_angs, h_angs.data(), h_cum_active_joint_idx[num_of_robots-1] * sizeof(float), cudaMemcpyHostToDevice);
  dim3 grid((num_of_robots+(block_size-1))/block_size);
  dim3 block(block_size);
  _jacobian<<<grid, block>>>(d_data, d_angs, d_num_of_joints_cum, d_num_of_active_joints_cum, d_result, num_of_robots);
  cudaMemcpy(h_jac_result.data(), d_result,
              h_cum_active_joint_idx[num_of_robots-1] * 6 * sizeof(float), cudaMemcpyDeviceToHost);
  return h_jac_result;
}

torch::Tensor FastKinematics::forward_kinematics_pytorch(torch::Tensor t_angs, size_t block_size) {
  // first check the shape of the input tensor
  assert(t_angs.size(0) == h_cum_active_joint_idx[num_of_robots-1]);
  dim3 grid((num_of_robots+(block_size-1))/block_size);
  dim3 block(block_size);
  _forward_kinematics<<<grid, block>>>(d_data, t_angs.data_ptr<float>(), d_num_of_joints_cum, d_num_of_active_joints_cum, d_result, num_of_robots);
  // now convert d_result to a tensor
  auto options = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA, 0).pinned_memory(true);
  torch::Tensor t_result = torch::from_blob(d_result, {h_fk_result.size()}, options);
  return t_result;
}

torch::Tensor FastKinematics::jacobian_mixed_frame_pytorch(torch::Tensor t_angs, size_t block_size) {
  // first check the shape of the input tensor
  assert(t_angs.size(0) == h_cum_active_joint_idx[num_of_robots-1]);
  dim3 grid((num_of_robots+(block_size-1))/block_size);
  dim3 block(block_size);
  _jacobian_mixed_frame<<<grid, block>>>(d_data, t_angs.data_ptr<float>(), d_num_of_joints_cum, d_num_of_active_joints_cum, d_result, num_of_robots);
  // now convert d_result to a tensor
  auto options = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA, 0).pinned_memory(true);
  torch::Tensor t_result = torch::from_blob(d_result, {h_jac_result.size()}, options);
  return t_result;
}

torch::Tensor FastKinematics::jacobian_world_frame_pytorch(torch::Tensor t_angs, size_t block_size) {
  // first check the shape of the input tensor
  assert(t_angs.size(0) == h_cum_active_joint_idx[num_of_robots-1]);
  dim3 grid((num_of_robots+(block_size-1))/block_size);
  dim3 block(block_size);
  _jacobian<<<grid, block>>>(d_data, t_angs.data_ptr<float>(), d_num_of_joints_cum, d_num_of_active_joints_cum, d_result, num_of_robots);
  // now convert d_result to a tensor
  auto options = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA, 0).pinned_memory(true);
  torch::Tensor t_result = torch::from_blob(d_result, {h_jac_result.size()}, options);
  return t_result;
}

size_t FastKinematics::get_num_of_active_joints() {
  return num_of_active_joints;
}

size_t FastKinematics::get_num_of_joints() {
  return num_of_joints;
}

void FastKinematics::allocate_memory() {
  h_cum_active_joint_idx = new size_t[num_of_robots];
  h_cum_active_joint_idx[0] = num_of_active_joints;
  for (size_t i = 1; i < num_of_robots; ++i) {
    h_cum_active_joint_idx[i] = h_cum_active_joint_idx[i - 1] + num_of_active_joints;
  }
  size_t result_max_size = std::max(7*num_of_robots, 6*h_cum_active_joint_idx[num_of_robots-1]);
  // resize the eigen xd vector
  h_fk_result.resize(7*num_of_robots);
  h_jac_result.resize(6*h_cum_active_joint_idx[num_of_robots-1]);
  cudaMalloc(&d_data, h_cum_data_idx[num_of_robots-1] * sizeof(float));
  cudaMalloc(&d_angs, h_cum_active_joint_idx[num_of_robots-1] * sizeof(float));
  cudaMalloc(&d_result, result_max_size * sizeof(float));
  cudaMalloc(&d_num_of_joints_cum, num_of_robots * sizeof(size_t));
  cudaMalloc(&d_num_of_active_joints_cum, num_of_robots * sizeof(size_t));
}

void FastKinematics::copy_memory() {
  cudaMemcpy(d_data, h_data, h_cum_data_idx[num_of_robots-1] * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(d_num_of_joints_cum, h_cum_data_idx, num_of_robots * sizeof(size_t), cudaMemcpyHostToDevice);
  cudaMemcpy(d_num_of_active_joints_cum, h_cum_active_joint_idx, num_of_robots * sizeof(size_t), cudaMemcpyHostToDevice);
}
