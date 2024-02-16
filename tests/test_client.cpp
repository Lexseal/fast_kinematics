#include <queue>
#include <parser.h>
#include <kinematics.h>
#include <thread>
#include <chrono>

size_t find_num_of_active_joints(JointTreePtr tip) {
  size_t num_of_active_joints = 0;
  JointTreePtr walker = tip;
  while (walker) {
    num_of_active_joints += (walker->type != urdf::Joint::FIXED);
    walker = walker->parent;
  }
  return num_of_active_joints;
}

size_t find_num_of_joints(JointTreePtr tip) {
  size_t num_of_joints = 0;
  JointTreePtr walker = tip;
  while (walker) {
    ++num_of_joints;
    walker = walker->parent;
  }
  return num_of_joints;
}

int main() {
  std::string eef_name = "panda_leftfinger";
  JointTreePtr root = Parser::parse("../panda.urdf"), tip;
  // print the joint tree using BFS
  std::queue<JointTreePtr> q;
  q.push(root);
  while (!q.empty()) {
    JointTreePtr joint_tree = q.front(); q.pop();
    // std::cout << "Joint name: " << joint_tree->name << std::endl;
    // std::cout << "Joint type: " << joint_tree->type << std::endl;
    // std::cout << "Joint rotation: " << joint_tree->rotation << std::endl;
    // std::cout << "Joint position: " << joint_tree->position << std::endl;
    // std::cout << "Joint axis: " << joint_tree->axis.transpose() << std::endl;
    // if (joint_tree->parent) {
    //   std::cout << "Parent joint name: " << joint_tree->parent->name << std::endl;
    // }
    for (auto &child : joint_tree->children) {
      q.push(child);
    }
    if (std::find(joint_tree->children_links.begin(),
                  joint_tree->children_links.end(),
                  eef_name) != joint_tree->children_links.end()) {
      tip = joint_tree;
    }
  }

  // sanity check do the chain transformation from the tip to the root
  // making all the joint values to be zero
  // We start with two quaternions, v representing the position we are at right now, and q representing the rotation
  // for each joint, we update v and q.
  // for rotation, we just multiply the quaternion
  // for the transformation, we have to rotate the position by the quaternion, and then add the translation
  Eigen::Quaternionf r = Eigen::Quaternionf::Identity();
  Eigen::Quaternionf t = Eigen::Quaternionf(0, 0, 0, 0);
  JointTreePtr walker = tip;
  // size_t num_of_active_joints = find_num_of_active_joints(tip);
  // std::vector<float> joint_values{0, 0.2, 0, -2.6, 0, 3.0, 0.8, 0.03};
  // while (walker) {
  //   if (walker->type == urdf::Joint::REVOLUTE) {
  //     // convert Eigen::AngleAxisf(joint_values[--num_of_active_joints], walker->axis) to quaternion
  //     Eigen::Quaternionf change = Eigen::Quaternionf(Eigen::AngleAxisf(joint_values[--num_of_active_joints], walker->axis));
  //     r = change * r;
  //     t = change * t * change.inverse();
  //     std::cout << change.coeffs().transpose() << std::endl;
  //   } else if (walker->type == urdf::Joint::PRISMATIC) {
  //     auto translation = joint_values[--num_of_active_joints] * walker->axis;
  //     Eigen::Quaternionf change = Eigen::Quaternionf(0, translation.x(), translation.y(), translation.z());
  //     t.coeffs() += change.coeffs();
  //     std::cout << change.coeffs().transpose() << std::endl;
  //   }
  //   std::cout << walker->rotation.coeffs().transpose() << std::endl;
  //   std::cout << walker->position.coeffs().transpose() << std::endl;
  //   r = walker->rotation * r;
  //   t = walker->rotation * t * walker->rotation.inverse();
  //   t.coeffs() += walker->position.coeffs();
  //   walker = walker->parent;
  // }
  // std::cout << "The transformation from the tip to the root is: " << std::endl;
  // std::cout << "Rotation: " << r.coeffs().transpose() << std::endl;
  // std::cout << "Position: " << t.coeffs().transpose() << std::endl;

  // data structure
  // [translation (3 floats), rotation (4 floats), type (1 float), axis (3 floats)]
  
  size_t num_of_robots = 10000;

  float *h_data = new float[11 * find_num_of_joints(tip) * num_of_robots];
  walker = tip;
  size_t idx = 11 * find_num_of_joints(tip) * num_of_robots;
  while (walker) {
    h_data[--idx] = walker->axis.z();
    h_data[--idx] = walker->axis.y();
    h_data[--idx] = walker->axis.x();
    h_data[--idx] = walker->type;
    h_data[--idx] = walker->rotation.coeffs().z();
    h_data[--idx] = walker->rotation.coeffs().y();
    h_data[--idx] = walker->rotation.coeffs().x();
    h_data[--idx] = walker->rotation.coeffs().w();
    h_data[--idx] = walker->position.z();
    h_data[--idx] = walker->position.y();
    h_data[--idx] = walker->position.x();
    walker = walker->parent;
  }
  for (size_t i = 0; i < num_of_robots-1; ++i) {
    memcpy(h_data + i*11*find_num_of_joints(tip),
           h_data + (num_of_robots-1)*11*find_num_of_joints(tip), 11*find_num_of_joints(tip)*sizeof(float));
  }
  float *h_angs = new float[8*num_of_robots];
  //  = {0, 0.2, 0, -2.6, 0, 3.0, 0.8, 0.03}
  h_angs[0] = 0;
  h_angs[1] = 0.2;
  h_angs[2] = 0;
  h_angs[3] = -2.6;
  h_angs[4] = 0;
  h_angs[5] = 3.0;
  h_angs[6] = 0.8;
  h_angs[7] = 0.03;
  for (size_t i = 0; i < num_of_robots; ++i) {
    memcpy(h_angs + i*8, h_angs, 8*sizeof(float));
  }
  size_t *h_num_of_data_cum = new size_t[num_of_robots];
  h_num_of_data_cum[0] = find_num_of_joints(tip)*11;
  for (size_t i = 1; i < num_of_robots; ++i) {
    h_num_of_data_cum[i] = h_num_of_data_cum[i - 1] + find_num_of_joints(tip)*11;
  }
  size_t *h_num_of_active_joints_cum = new size_t[num_of_robots];
  h_num_of_active_joints_cum[0] = find_num_of_active_joints(tip);
  for (size_t i = 1; i < num_of_robots; ++i) {
    h_num_of_active_joints_cum[i] = h_num_of_active_joints_cum[i - 1] + find_num_of_active_joints(tip);
  }
  
  float *h_result = new float[6*h_num_of_active_joints_cum[num_of_robots-1]]{0};

  float *d_data, *d_angs, *d_result;
  size_t *d_num_of_joints_cum, *d_num_of_active_joints_cum;
  cudaMalloc(&d_data, h_num_of_data_cum[num_of_robots-1] * sizeof(float));
  cudaMalloc(&d_angs, h_num_of_active_joints_cum[num_of_robots-1] * sizeof(float));
  cudaMalloc(&d_result, 6*h_num_of_active_joints_cum[num_of_robots-1] * sizeof(float));
  cudaMalloc(&d_num_of_joints_cum, num_of_robots * sizeof(size_t));
  cudaMalloc(&d_num_of_active_joints_cum, num_of_robots * sizeof(size_t));

  std::cout << "memory assigned" << std::endl;
  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

  cudaMemcpy(d_data, h_data, h_num_of_data_cum[num_of_robots-1] * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(d_angs, h_angs, h_num_of_active_joints_cum[num_of_robots-1] * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(d_num_of_joints_cum, h_num_of_data_cum, num_of_robots * sizeof(size_t), cudaMemcpyHostToDevice);
  cudaMemcpy(d_num_of_active_joints_cum, h_num_of_active_joints_cum, num_of_robots * sizeof(size_t), cudaMemcpyHostToDevice);
  cudaMemcpy(d_result, h_result, 6*h_num_of_active_joints_cum[num_of_robots-1] * sizeof(float), cudaMemcpyHostToDevice);

  std::cout << "memory copied" << std::endl;

  // jacobian(d_data, d_angs, d_num_of_joints_cum, d_num_of_active_joints_cum, 3, d_result);
  dim3 grid((num_of_robots+1023)/1024);
  dim3 block(1024);
  jacobian<<<grid, block>>>(d_data, d_angs, d_num_of_joints_cum, d_num_of_active_joints_cum, d_result, num_of_robots);

  cudaMemcpy(h_result, d_result, 6*h_num_of_active_joints_cum[num_of_robots-1] * sizeof(float), cudaMemcpyDeviceToHost);

  std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
  std::cout << "The time to copy data and for the kernel to run on a size 10000 is: " << time_span.count() << " seconds." << std::endl;
  for (size_t i = 0; i < 6; ++i) {
    for (size_t j = h_num_of_active_joints_cum[9998]; j < h_num_of_active_joints_cum[9999]; ++j) {
      std::cout << h_result[j*6+i] << " ";
    }
    std::cout << std::endl;
  }

  return 0;
}
