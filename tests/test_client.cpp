#include <queue>
#include <parser.h>
#include <kinematics.h>
#include <iomanip>
#include <iostream>
#include <fast_kinematics.h>

int main() {
  size_t num_of_robots = 1;
  FastKinematics fk("../kuka_iiwa.urdf", num_of_robots, "lbr_iiwa_link_7");
  float *h_angs = new float[num_of_robots*fk.get_num_of_active_joints()];
  h_angs[0] = 0.0;
  h_angs[1] = -M_PI / 4.0;
  h_angs[2] = 0.0;
  h_angs[3] = M_PI / 2.0;
  h_angs[4] = 0.0;
  h_angs[5] = M_PI / 4.0;
  h_angs[6] = 0.0;
  for (size_t i = 0; i < num_of_robots; ++i) {
    memcpy(h_angs + i*fk.get_num_of_active_joints(), h_angs, fk.get_num_of_active_joints()*sizeof(float));
  }

  float *h_result = fk.jacobian_mixed_frame(h_angs);

  for (size_t i = 0; i < 6; ++i) {
    for (size_t j = 0; j < fk.get_num_of_active_joints(); ++j) {
      std::cout << std::fixed << std::setprecision(3) << h_result[j*6+i] << " ";
    }
    std::cout << std::endl;
  }
  return 0;
}
