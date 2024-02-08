#include <queue>
#include <parser.h>
#include <fk.h>

size_t find_num_of_joints(JointTreePtr tip) {
  size_t num_of_joints = 0;
  JointTreePtr walker = tip;
  while (walker) {
    num_of_joints += (walker->type != urdf::Joint::FIXED);
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
  size_t num_of_joints = find_num_of_joints(tip);
  std::vector<float> joint_values{0, 0.2, 0, -2.6, 0, 3.0, 0.8, 0.03};
  while (walker) {
    if (walker->type == urdf::Joint::REVOLUTE) {
      // convert Eigen::AngleAxisf(joint_values[--num_of_joints], walker->axis) to quaternion
      Eigen::Quaternionf change = Eigen::Quaternionf(Eigen::AngleAxisf(joint_values[--num_of_joints], walker->axis));
      r = change * r;
      t = change * t * change.inverse();
      std::cout << change.coeffs().transpose() << std::endl;
    } else if (walker->type == urdf::Joint::PRISMATIC) {
      auto translation = joint_values[--num_of_joints] * walker->axis;
      Eigen::Quaternionf change = Eigen::Quaternionf(0, translation.x(), translation.y(), translation.z());
      t.coeffs() += change.coeffs();
      std::cout << change.coeffs().transpose() << std::endl;
    }
    std::cout << walker->rotation.coeffs().transpose() << std::endl;
    std::cout << walker->position.coeffs().transpose() << std::endl;
    r = walker->rotation * r;
    t = walker->rotation * t * walker->rotation.inverse();
    t.coeffs() += walker->position.coeffs();
    walker = walker->parent;
  }
  std::cout << "The transformation from the tip to the root is: " << std::endl;
  std::cout << "Rotation: " << r.coeffs().transpose() << std::endl;
  std::cout << "Position: " << t.coeffs().transpose() << std::endl;

  // data structure
  // [type (1 float), axis (3 floats), rotation (4 floats), translation (3 floats)]
  std::vector<float> data;
  walker = tip;
  while (walker) {
    data.push_back(walker->type);
    data.push_back(walker->axis.x());
    data.push_back(walker->axis.y());
    data.push_back(walker->axis.z());
    data.push_back(walker->rotation.coeffs().w());
    data.push_back(walker->rotation.coeffs().x());
    data.push_back(walker->rotation.coeffs().y());
    data.push_back(walker->rotation.coeffs().z());
    data.push_back(walker->position.x());
    data.push_back(walker->position.y());
    data.push_back(walker->position.z());
    walker = walker->parent;
  }

  // std::vector<float> joint_values{0, 0.2, 0, -2.6, 0, 3.0, 0.8, 0.03};
  std::cout << forward_kinematics(data, joint_values) << std::endl;

  return 0;
}
