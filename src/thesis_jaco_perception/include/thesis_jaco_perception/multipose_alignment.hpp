#ifndef THESIS_JACO_PERCEPTION__MULTIPOSE_ALIGNMENT_HPP_
#define THESIS_JACO_PERCEPTION__MULTIPOSE_ALIGNMENT_HPP_
#include <array>
#include <string>
#include <vector>
#include "thesis_jaco_perception/multipose_association.hpp"
namespace thesis_jaco_perception { struct MultiPoseCapture {std::vector<Point3> optical_cloud;std::vector<Capsule3> world_capsules;}; struct MultiPoseResult {bool accepted{false},converged{false},limits_active{false};std::string reason;std::array<double,6> pose{};double initial_cost{0},final_cost{0},stability{0};std::vector<std::size_t> associations_per_round,associations_per_pose;std::vector<std::size_t> covered_capsules;std::vector<std::vector<MultiPoseCorrespondence>> rounds;}; class MultiPoseAlignmentEstimator {public: MultiPoseResult estimate(const std::vector<MultiPoseCapture>&,const std::array<double,6>&,const std::array<double,6>&) const;}; }
#endif
