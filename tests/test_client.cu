#include <queue>
#include <vector>
#include <iomanip>
#include <iostream>
#include <fast_kinematics.h>
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

  // fk.do_nothing(h_angs);

  auto start = std::chrono::high_resolution_clock::now();
  for (size_t i = 0; i < 1000; ++i) {
    auto h_result = fk.jacobian_mixed_frame(h_angs);
  }
  auto end = std::chrono::high_resolution_clock::now();
  std::cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;

  // for (size_t i = 0; i < 6; ++i) {
  //   for (size_t j = 0; j < fk.get_num_of_active_joints(); ++j) {
  //     std::cout << std::fixed << std::setprecision(3) << h_result[j*6+i] << " ";
  //   }
  //   std::cout << std::endl;
  // }
  return 0;
}
