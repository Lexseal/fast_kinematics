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
  
  size_t num_of_robots = 4;

  float *data = new float[11 * find_num_of_joints(tip) * num_of_robots];
  walker = tip;
  size_t idx = 11 * find_num_of_joints(tip) * num_of_robots;
  while (walker) {
    data[--idx] = walker->axis.z();
    data[--idx] = walker->axis.y();
    data[--idx] = walker->axis.x();
    data[--idx] = walker->type;
    data[--idx] = walker->rotation.coeffs().z();
    data[--idx] = walker->rotation.coeffs().y();
    data[--idx] = walker->rotation.coeffs().x();
    data[--idx] = walker->rotation.coeffs().w();
    data[--idx] = walker->position.z();
    data[--idx] = walker->position.y();
    data[--idx] = walker->position.x();
    walker = walker->parent;
  }
  for (size_t i = 0; i < num_of_robots-1; ++i) {
    memcpy(data + i*11*find_num_of_joints(tip),
           data + (num_of_robots-1)*11*find_num_of_joints(tip), 11*find_num_of_joints(tip)*sizeof(float));
  }
  float *angs = new float[8*num_of_robots];
  //  = {0, 0.2, 0, -2.6, 0, 3.0, 0.8, 0.03}
  angs[0] = 0;
  angs[1] = 0.2;
  angs[2] = 0;
  angs[3] = -2.6;
  angs[4] = 0;
  angs[5] = 3.0;
  angs[6] = 0.8;
  angs[7] = 0.03;
  for (size_t i = 0; i < num_of_robots; ++i) {
    memcpy(angs + i*8, angs, 8*sizeof(float));
  }
  size_t *num_of_joints_cum = new size_t[num_of_robots];
  num_of_joints_cum[0] = find_num_of_joints(tip)*11;
  for (size_t i = 1; i < num_of_robots; ++i) {
    num_of_joints_cum[i] = num_of_joints_cum[i - 1] + find_num_of_joints(tip)*11;
  }
  size_t *num_of_active_joints_cum = new size_t[num_of_robots];
  num_of_active_joints_cum[0] = find_num_of_active_joints(tip);
  for (size_t i = 1; i < num_of_robots; ++i) {
    num_of_active_joints_cum[i] = num_of_active_joints_cum[i - 1] + find_num_of_active_joints(tip);
  }
  
  // for (size_t i = 0; i < 11 * find_num_of_joints(tip) * num_of_robots; ++i) {
  //   std::cout << data[i] << std::endl;
  // }
  float **result = new float*[6];
  for (size_t i = 0; i < 6; ++i) {
    result[i] = new float[num_of_active_joints_cum[num_of_robots-1]]{0};
  }
  jacobian(data, angs, num_of_joints_cum, num_of_active_joints_cum, 3, result);
  // for (size_t i = 0; i < 7; ++i) {
  //   std::cout << result[i] << std::endl;
  // }

  for (size_t i = 0; i < 6; ++i) {
    for (size_t j = num_of_active_joints_cum[2]; j < num_of_active_joints_cum[3]; ++j) {
      std::cout << result[i][j] << " ";
    }
    std::cout << std::endl;
  }

  // use multiple threads to do the forward kinematics
  // std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
  // std::vector<std::thread> threads;
  // std::vector<Eigen::Matrix<float, 7, 1>> results(num_of_robots);
  // for (size_t i = 0; i < num_of_robots; ++i) {
  //   threads.push_back(std::thread([=, &results](){
  //     results[i] = forward_kinematics(data, angs, num_of_joints_cum, num_of_active_joints_cum, i);
  //   }));
  // }

  // for (auto &thread : threads) {
  //   thread.join();
  // }
  // std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
  // std::cout << "The time it takes to do the forward kinematics for " << num_of_robots << " robots is: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;

  // // use single thread to do the forward kinematics
  // start = std::chrono::high_resolution_clock::now();
  // for (size_t i = 0; i < num_of_robots; ++i) {
  //   results[i] = forward_kinematics(data, angs, num_of_joints_cum, num_of_active_joints_cum, i);
  // }
  // end = std::chrono::high_resolution_clock::now();
  // std::cout << "The time it takes to do the forward kinematics for " << num_of_robots << " robots is: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;

  return 0;
}
