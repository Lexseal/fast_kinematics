#include <queue>
#include <vector>
#include <iomanip>
#include <iostream>
#include <fast_kinematics.h>
#include <torch/torch.h>
#include <chrono>

int main() {
  size_t num_of_robots = 1000;
  FastKinematics fk("../kuka_iiwa.urdf", num_of_robots, "lbr_iiwa_link_7");
  Eigen::VectorXf h_angs(num_of_robots * fk.get_num_of_active_joints());
  h_angs[0] = 0.0;
  h_angs[1] = -M_PI / 4.0;
  h_angs[2] = 0.0;
  h_angs[3] = M_PI / 2.0;
  h_angs[4] = 0.0;
  h_angs[5] = M_PI / 4.0;
  h_angs[6] = 0.0;
  for (size_t i = 1; i < num_of_robots; ++i) {
    std::memcpy(h_angs.data() + i * fk.get_num_of_active_joints(), h_angs.data(),
                fk.get_num_of_active_joints() * sizeof(float));
  }

  float *d_angs;
  cudaMalloc(&d_angs, h_angs.size() * sizeof(float));
  cudaMemcpy(d_angs, h_angs.data(), h_angs.size() * sizeof(float), cudaMemcpyHostToDevice);
  torch::Tensor t_angs = torch::from_blob(d_angs, {h_angs.size()});
  float *result_ptr = fk.forward_kinematics_raw_ptr(t_angs.data_ptr<float>(), num_of_robots);
  auto options = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA, 0).pinned_memory(true);
  long sz = fk.get_num_of_active_joints()*6;
  torch::Tensor result = torch::from_blob(result_ptr, {sz}, options);
  std::cout << result << std::endl;

  // auto start = std::chrono::high_resolution_clock::now();
  // for (size_t i = 0; i < 1000; ++i) {
  //   // auto h_result = fk.jacobian_mixed_frame(h_angs);
  // }
  // auto end = std::chrono::high_resolution_clock::now();
  // std::cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;

  // for (size_t i = 0; i < 6; ++i) {
  //   for (size_t j = 0; j < fk.get_num_of_active_joints(); ++j) {
  //     std::cout << std::fixed << std::setprecision(3) << h_result[j*6+i] << " ";
  //   }
  //   std::cout << std::endl;
  // }
  return 0;
}
