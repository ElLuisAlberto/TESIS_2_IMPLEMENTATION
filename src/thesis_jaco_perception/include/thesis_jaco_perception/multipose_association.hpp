#ifndef THESIS_JACO_PERCEPTION__MULTIPOSE_ASSOCIATION_HPP_
#define THESIS_JACO_PERCEPTION__MULTIPOSE_ASSOCIATION_HPP_
#include <array>
#include <cstddef>
#include <vector>
#include "thesis_jaco_perception/self_filter_geometry.hpp"
namespace thesis_jaco_perception {
struct MultiPoseCorrespondence {std::size_t pose_index,point_index,capsule_index,round_index; Point3 optical_point; double first_distance,second_distance;};
struct AssociationRound {double gate,margin;};
std::vector<MultiPoseCorrespondence> associate_multipose(const std::vector<std::vector<Point3>>&,const std::vector<std::vector<Capsule3>>&,const std::array<double,6>&,const AssociationRound&,std::size_t);
}
#endif
