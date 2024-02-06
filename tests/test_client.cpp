#include <queue>
#include <parser.h>

int main() {
  JointTreePtr root = Parser::parse("../panda.urdf");
  // print the joint tree using BFS
  std::queue<JointTreePtr> q;
  q.push(root);
  while (!q.empty()) {
    JointTreePtr joint_tree = q.front(); q.pop();
    std::cout << "Joint name: " << joint_tree->name << std::endl;
    std::cout << "Joint type: " << joint_tree->type << std::endl;
    std::cout << "Joint rotation: " << joint_tree->rotation << std::endl;
    std::cout << "Joint position: " << joint_tree->position << std::endl;
    std::cout << "Joint axis: " << joint_tree->axis.transpose() << std::endl;
    if (joint_tree->parent) {
      std::cout << "Parent joint name: " << joint_tree->parent->name << std::endl;
    }
    for (auto &child : joint_tree->children) {
      q.push(child);
    }
  }

  // sanity check add up all the 

  return 0;
}
