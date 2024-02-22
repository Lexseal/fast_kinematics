#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <urdf_parser/urdf_parser.h>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>

struct JointTree {
  std::string name;
  int type;
  Eigen::Quaternionf rotation;
  Eigen::Quaternionf position;
  Eigen::Vector3f axis;
  std::shared_ptr<JointTree> parent;
  std::vector<std::shared_ptr<JointTree>> children;
  std::vector<std::string> children_links;
};
using JointTreePtr = std::shared_ptr<JointTree>;

// representation
// [translation (3 floats), rotation (4 floats), type (1 float), axis (3 floats), ...]
constexpr size_t float_per_joint = 11;

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
      joint_tree->rotation = Eigen::Quaternionf(rotation.w, rotation.x, rotation.y, rotation.z);
      joint_tree->position = Eigen::Quaternionf(0, position.x, position.y, position.z);
      joint_tree->axis = Eigen::Vector3f(axis.x, axis.y, axis.z);
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

  static size_t find_num_of_active_joints(JointTreePtr tip) {
    size_t num_of_active_joints = 0;
    JointTreePtr walker = tip;
    while (walker) {
      num_of_active_joints += (walker->type != urdf::Joint::FIXED);
      walker = walker->parent;
    }
    return num_of_active_joints;
  }

  static size_t find_num_of_joints(JointTreePtr tip) {
    size_t num_of_joints = 0;
    JointTreePtr walker = tip;
    while (walker) {
      ++num_of_joints;
      walker = walker->parent;
    }
    return num_of_joints;
  }

  static JointTreePtr find_link_parent(JointTreePtr root, std::string eef_name, bool verbose = false) {
    JointTreePtr tip;
    std::queue<JointTreePtr> q;
    q.push(root);
    while (!q.empty()) {
      JointTreePtr node = q.front(); q.pop();
      if (verbose) {
        std::cout << "Joint name: " << node->name << std::endl;
        std::cout << "Joint type: " << node->type << std::endl;
        std::cout << "Joint rotation: " << node->rotation << std::endl;
        std::cout << "Joint position: " << node->position << std::endl;
        std::cout << "Joint axis: " << node->axis.transpose() << std::endl;
        if (node->parent) {
          std::cout << "Parent joint name: " << node->parent->name << std::endl;
        }
      }
      for (auto &child : node->children) q.push(child);
      if (std::find(node->children_links.begin(),
                    node->children_links.end(),
                    eef_name) != node->children_links.end()) {
        tip = node;
        break;  // found
      }
    }
    return tip;
  }

  static void prepare_repr(size_t num_of_robots, JointTreePtr tip, float **data, size_t **cum_idx) {
    size_t num_of_joints = find_num_of_joints(tip);
    size_t idx = num_of_robots * float_per_joint * num_of_joints;
    *data = new float[idx];
    *cum_idx = new size_t[num_of_robots];
    (*cum_idx)[0] = num_of_joints * float_per_joint;
    JointTreePtr walker = tip;
    while (walker) {
      (*data)[--idx] = walker->axis.z();
      (*data)[--idx] = walker->axis.y();
      (*data)[--idx] = walker->axis.x();
      (*data)[--idx] = walker->type;
      (*data)[--idx] = walker->rotation.coeffs().z();
      (*data)[--idx] = walker->rotation.coeffs().y();
      (*data)[--idx] = walker->rotation.coeffs().x();
      (*data)[--idx] = walker->rotation.coeffs().w();
      (*data)[--idx] = walker->position.z();
      (*data)[--idx] = walker->position.y();
      (*data)[--idx] = walker->position.x();
      walker = walker->parent;
    }
    for (size_t i = 0; i < num_of_robots-1; ++i) {
      memcpy((*data) + i*float_per_joint*num_of_joints,
             (*data) + (num_of_robots-1)*float_per_joint*num_of_joints, float_per_joint*num_of_joints*sizeof(float));
      (*cum_idx)[i+1] = (*cum_idx)[i] + num_of_joints * float_per_joint;
    }
  }
};
