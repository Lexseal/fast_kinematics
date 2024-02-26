#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

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
  static JointTreePtr parse(std::string urdf_path, bool verbose = false);

  static size_t find_num_of_active_joints(JointTreePtr tip);

  static size_t find_num_of_joints(JointTreePtr tip);

  static JointTreePtr find_link_parent(JointTreePtr root, std::string eef_name, bool verbose = false);

  static void prepare_repr(size_t num_of_robots, JointTreePtr tip, float **data, size_t **cum_idx);
};
