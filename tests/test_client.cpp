#include <queue>
#include <parser.h>
#include <kinematics.h>
#include <iomanip>

int main() {
  JointTreePtr root = Parser::parse("../kuka_iiwa.urdf");
  std::string eef_name = "lbr_iiwa_link_7";
  JointTreePtr tip = Parser::find_link_parent(root, eef_name);
  size_t num_of_active_joints = Parser::find_num_of_active_joints(tip),
         num_of_joints = Parser::find_num_of_joints(tip);

  size_t num_of_robots = 1000;
  float *h_data;
  size_t *h_cum_data_idx;
  Parser::prepare_repr(num_of_robots, tip, &h_data, &h_cum_data_idx);
  
  float *h_angs = new float[num_of_robots*num_of_active_joints];
  h_angs[0] = 0.0;
  h_angs[1] = -M_PI / 4.0;
  h_angs[2] = 0.0;
  h_angs[3] = M_PI / 2.0;
  h_angs[4] = 0.0;
  h_angs[5] = M_PI / 4.0;
  h_angs[6] = 0.0;
  for (size_t i = 0; i < num_of_robots; ++i) {
    memcpy(h_angs + i*num_of_active_joints, h_angs, num_of_active_joints*sizeof(float));
  }
  size_t *h_cum_active_joint_idx = new size_t[num_of_robots];
  h_cum_active_joint_idx[0] = num_of_active_joints;
  for (size_t i = 1; i < num_of_robots; ++i) {
    h_cum_active_joint_idx[i] = h_cum_active_joint_idx[i - 1] + num_of_active_joints;
  }

  float *d_data, *d_angs, *d_result;
  size_t *d_num_of_joints_cum, *d_num_of_active_joints_cum;
  cudaMalloc(&d_data, h_cum_data_idx[num_of_robots-1] * sizeof(float));
  cudaMalloc(&d_angs, h_cum_active_joint_idx[num_of_robots-1] * sizeof(float));
  cudaMalloc(&d_result, 6*h_cum_active_joint_idx[num_of_robots-1] * sizeof(float));
  cudaMalloc(&d_num_of_joints_cum, num_of_robots * sizeof(size_t));
  cudaMalloc(&d_num_of_active_joints_cum, num_of_robots * sizeof(size_t));

  cudaMemcpy(d_data, h_data, h_cum_data_idx[num_of_robots-1] * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(d_angs, h_angs, h_cum_active_joint_idx[num_of_robots-1] * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(d_num_of_joints_cum, h_cum_data_idx, num_of_robots * sizeof(size_t), cudaMemcpyHostToDevice);
  cudaMemcpy(d_num_of_active_joints_cum, h_cum_active_joint_idx, num_of_robots * sizeof(size_t), cudaMemcpyHostToDevice);
  cudaMemset(d_result, 0, 6*h_cum_active_joint_idx[num_of_robots-1] * sizeof(float));

  dim3 grid((num_of_robots+255)/256);
  dim3 block(256);
  jacobian_mixed_frame<<<grid, block>>>(d_data, d_angs, d_num_of_joints_cum, d_num_of_active_joints_cum, d_result, num_of_robots);

  float *h_result = new float[6*h_cum_active_joint_idx[num_of_robots-1]]{0};
  cudaMemcpy(h_result, d_result, 6*h_cum_active_joint_idx[num_of_robots-1] * sizeof(float), cudaMemcpyDeviceToHost);

  for (size_t i = 0; i < 6; ++i) {
    for (size_t j = h_cum_active_joint_idx[0]; j < h_cum_active_joint_idx[1]; ++j) {
      std::cout << std::fixed << std::setprecision(3) << h_result[j*6+i] << " ";
    }
    std::cout << std::endl;
  }
  return 0;
}
