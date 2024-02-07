#include <queue>
#include <parser.h>

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
  std::string tip_name = "panda_hand_joint";
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
    if (joint_tree->name == tip_name) {
      tip = joint_tree;
    }
  }

  // sanity check do the chain transformation from the tip to the root
  // making all the joint values to be zero
  // We start with two quaternions, v representing the position we are at right now, and q representing the rotation
  // for each joint, we update v and q.
  // for rotation, we just multiply the quaternion
  // for the transformation, we have to rotate the position by the quaternion, and then add the translation
  Eigen::Quaterniond r = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond t = Eigen::Quaterniond(0, 0, 0, 0);
  JointTreePtr walker = tip;
  size_t num_of_joints = find_num_of_joints(tip);
  std::vector<double> joint_values{0, 0.2, 0, -2.6, 0, 3.0, 0.8};
  while (walker) {
    if (walker->type == urdf::Joint::REVOLUTE) {
      // convert Eigen::AngleAxisd(joint_values[--num_of_joints], walker->axis) to quaternion
      Eigen::Quaterniond change = Eigen::Quaterniond(Eigen::AngleAxisd(joint_values[--num_of_joints], walker->axis));
      r = change * r;
      t = change * t * change.inverse();
    }

    r = walker->rotation * r;
    t = walker->rotation * t * walker->rotation.inverse();
    t.coeffs() += walker->position.coeffs();
    walker = walker->parent;
  }
  std::cout << "The transformation from the tip to the root is: " << std::endl;
  std::cout << "Rotation: " << r.coeffs().transpose() << std::endl;
  std::cout << "Position: " << t.coeffs().transpose() << std::endl;

  return 0;
}
