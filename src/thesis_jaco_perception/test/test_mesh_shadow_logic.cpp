#include "thesis_jaco_perception/self_mesh_geometry.hpp"
#include "gtest/gtest.h"
#include <limits>
namespace g=thesis_jaco_perception;
TEST(MeshShadowLogic, ClassifiesExternalSelfOccludedAndRejectsInvalid){g::MeshTriangle t{{1,-1,-1},{1,1,-1},{1,0,1},0};EXPECT_EQ(g::classify_mesh_observation({t},{0,0,0},{.8,0,0},.07),g::MeshClassification::EXTERNAL_CONFIRMED);EXPECT_EQ(g::classify_mesh_observation({t},{0,0,0},{1.02,0,0},.07),g::MeshClassification::SELF_CONFIRMED);EXPECT_EQ(g::classify_mesh_observation({t},{0,0,0},{1.2,0,0},.07),g::MeshClassification::OCCLUDED_UNKNOWN);
EXPECT_THROW(g::classify_mesh_observation({t},{0,0,0},{1,0,0},-1),std::invalid_argument);
EXPECT_THROW(g::first_mesh_intersection({t},{0,0,0},{std::numeric_limits<double>::quiet_NaN(),0,0}),std::invalid_argument);}
TEST(MeshShadowLogic, PreservesSafetyProvenanceAndNearestLink){auto p=g::collision_mesh_provenance();EXPECT_TRUE(p.session_only);EXPECT_TRUE(p.not_for_safety);EXPECT_EQ(p.geometry_kind,"collision_mesh");g::MeshInstance a{"a","collision",{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1},{{{1,-1,-1},{1,1,-1},{1,0,1},0}},{},{},p};g::MeshInstance b=a;b.link_frame="b";b.triangles[0].a.x=2;b.triangles[0].b.x=2;b.triangles[0].c.x=2;g::build_mesh_bvh(&a);g::build_mesh_bvh(&b);auto h=g::first_world_mesh_intersection({a,b},{{"a",a.local_collision_transform},{"b",b.local_collision_transform}},{0,0,0},{3,0,0});ASSERT_TRUE(h);EXPECT_EQ(h->link_frame,"a");}
