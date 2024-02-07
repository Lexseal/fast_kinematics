#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <urdf_parser/urdf_parser.h>
#include <iostream>
#include <unordered_map>

struct JointTree {
  std::string name;
  int type;
  Eigen::Quaterniond rotation;
  Eigen::Quaterniond position;
  Eigen::Vector3d axis;
  std::shared_ptr<JointTree> parent;
  std::vector<std::shared_ptr<JointTree>> children;
  std::vector<std::string> children_links;
};
using JointTreePtr = std::shared_ptr<JointTree>;

class Parser {
public:
  static JointTreePtr parse(std::string urdf_path, bool verbose = false) {
    urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(urdf_path);
    if (verbose) {
      std::cout << "Model name: " << model->getName() << std::endl;
      std::cout << "Root link name: " << model->getRoot()->name << std::endl;
      std::cout << "Number of links: " << model->links_.size() << std::endl;
      std::cout << "Number of joints: " << model->joints_.size() << std::endl;
      std::cout << "Number of materials: " << model->materials_.size() << std::endl;
    }

    std::unordered_map<std::string, JointTreePtr> link_to_parent;
    std::unordered_map<std::string, std::vector<JointTreePtr>> link_to_children;
    for (auto joint : model->joints_) {
      // create a joint tree of the link
      auto joint_tree = std::make_shared<JointTree>();
      joint_tree->name = joint.second->name;
      joint_tree->type = joint.second->type;
      auto &rotation = joint.second->parent_to_joint_origin_transform.rotation;
      auto &position = joint.second->parent_to_joint_origin_transform.position;
      auto &axis = joint.second->axis;
      joint_tree->rotation = Eigen::Quaterniond(rotation.w, rotation.x, rotation.y, rotation.z);
      joint_tree->position = Eigen::Quaterniond(0, position.x, position.y, position.z);
      joint_tree->axis = Eigen::Vector3d(axis.x, axis.y, axis.z);
      std::string parent = joint.second->parent_link_name, child = joint.second->child_link_name;
      link_to_children[parent].push_back(joint_tree);
      assert(link_to_parent.count(child) == 0);  // do not support a link having multiple parents
      link_to_parent[child] = joint_tree;
      joint_tree->children_links.push_back(child);
    }

    // connect the joint trees
    for (auto &itm : link_to_parent) {
      const std::string &link_name = itm.first;
      JointTreePtr &parent = itm.second;
      for (auto &child : link_to_children[link_name]) {
        parent->children.push_back(child);
        child->parent = parent;
      }
    }

    // find the tree root
    std::string root_link_name = model->getRoot()->name;
    std::vector<JointTreePtr> root_joints = link_to_children[root_link_name];
    assert(root_joints.size() == 1);  // do not support a link having multiple root links yet
    JointTreePtr root = root_joints[0];
  
    return root;
  }
};
