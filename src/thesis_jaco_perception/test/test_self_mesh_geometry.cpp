#include "thesis_jaco_perception/self_mesh_geometry.hpp"
#include "gtest/gtest.h"
#include <array>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <sstream>
#include <unordered_set>
#include <functional>
namespace g=thesis_jaco_perception;
const g::MeshTriangle a{{0,-1,-1},{0,1,-1},{0,0,1},7};
TEST(Mesh, FirstAndLink){auto h=g::first_mesh_intersection({{{2,-1,-1},{2,1,-1},{2,0,1},9},a},{-1,0,0},{3,0,0});ASSERT_TRUE(h);EXPECT_NEAR(h->distance,1,1e-12);EXPECT_EQ(h->link_index,7U);}
TEST(Mesh, TransformAndClasses){double m[16]={1,0,0,1,0,1,0,0,0,0,1,0,0,0,0,1};auto t=g::transform_triangle(a,m);auto c=g::MeshPoint3{0,0,0};EXPECT_EQ(g::classify_mesh_observation({t},c,{0.7,0,0},.2),g::MeshClassification::EXTERNAL_CONFIRMED);EXPECT_EQ(g::classify_mesh_observation({t},c,{1.0,0,0},.01),g::MeshClassification::SELF_CONFIRMED);EXPECT_EQ(g::classify_mesh_observation({t},c,{1.2,0,0},.01),g::MeshClassification::OCCLUDED_UNKNOWN);}
TEST(Mesh, Rejects){
  EXPECT_THROW(g::first_mesh_intersection({}, {0,0,0},{1,0,0}),std::invalid_argument);
  EXPECT_THROW(g::first_mesh_intersection({{{0,0,0},{0,0,0},{0,0,0},0}}, {0,0,0},{1,0,0}),std::invalid_argument);
  EXPECT_FALSE(g::first_mesh_intersection({a},{-1,0,0},{-1,2,0}));
}
TEST(Mesh, Provenance){EXPECT_EQ(g::collision_mesh_provenance().geometry_kind,"collision_mesh");EXPECT_EQ(g::visual_shadow_only_provenance().geometry_kind,"visual_shadow_only");}
std::string expanded_jaco() {
  std::string command = std::string("xacro ") + THESIS_DESCRIPTION_XACRO;
  FILE * pipe = popen(command.c_str(), "r"); if (!pipe) throw std::runtime_error("xacro unavailable");
  std::string xml; char buffer[4096]; while (fgets(buffer, sizeof(buffer), pipe)) xml += buffer;
  EXPECT_EQ(pclose(pipe), 0); return xml;
}
TEST(Mesh, LoadsRealCollisionSet) {
  auto meshes=g::load_collision_meshes_from_urdf_xml(expanded_jaco());
  ASSERT_EQ(meshes.size(),13U); std::size_t triangles=0;std::ostringstream inventory;
  for(const auto&m:meshes){EXPECT_FALSE(m.link_frame.empty());EXPECT_FALSE(m.uri.empty());EXPECT_FALSE(m.triangles.empty());EXPECT_FALSE(m.bvh.empty());triangles+=m.triangles.size();inventory<<m.link_frame<<":"<<m.triangles.size()<<";";for(const auto&t:m.triangles)EXPECT_TRUE(g::is_non_degenerate(t));}
  RecordProperty("F8B_MESH_INVENTORY",inventory.str());
  EXPECT_GT(triangles,0U); EXPECT_NE(g::resolve_thesis_collision_uri(meshes.front().uri).find("thesis_description/meshes/"),std::string::npos);
}
TEST(Mesh, BvhStructuralContractForEveryRealCollisionMesh) {
  auto meshes=g::load_collision_meshes_from_urdf_xml(expanded_jaco());
  std::ostringstream structure;for(const auto&m:meshes){ASSERT_GT(m.triangles.size(),8U);ASSERT_GT(m.bvh.size(),1U);std::unordered_set<std::size_t> seen;std::size_t leaves=0,max_depth=0,max_leaf=0;std::function<void(int,std::size_t)> walk=[&](int id,std::size_t depth){ASSERT_GE(id,0);ASSERT_LT(static_cast<std::size_t>(id),m.bvh.size());const auto&n=m.bvh[id];max_depth=std::max(max_depth,depth);if(n.count){++leaves;max_leaf=std::max(max_leaf,n.count);ASSERT_LE(n.count,8U);ASSERT_EQ(n.left,-1);ASSERT_EQ(n.right,-1);for(std::size_t i=n.first;i<n.first+n.count;++i)ASSERT_TRUE(seen.insert(m.bvh_indices[i]).second);}else{ASSERT_GE(n.left,0);ASSERT_GE(n.right,0);walk(n.left,depth+1);walk(n.right,depth+1);}};walk(0,0);EXPECT_GT(leaves,1U);EXPECT_GE(max_depth,2U);EXPECT_EQ(seen.size(),m.triangles.size());structure<<m.link_frame<<":n"<<m.bvh.size()<<",l"<<leaves<<",d"<<max_depth<<",m"<<max_leaf<<";";}RecordProperty("F8D_BVH_STRUCTURE",structure.str());
}
TEST(Mesh, BvhMatchesBruteAndWorldSelectsNearest) {
  g::MeshInstance m{"a","package://thesis_description/meshes/base.dae",{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1},{a},{},{},g::collision_mesh_provenance()};g::build_mesh_bvh(&m);
  auto brute=g::first_mesh_intersection(m.triangles,{-1,0,0},{3,0,0}); auto bvh=g::first_bvh_intersection(m,{-1,0,0},{3,0,0});ASSERT_TRUE(brute);ASSERT_TRUE(bvh);EXPECT_NEAR(brute->distance,bvh->distance,1e-12);
  auto n=m;n.link_frame="b";std::array<double,16> ta{1,0,0,2,0,1,0,0,0,0,1,0,0,0,0,1},tb{1,0,0,1,0,1,0,0,0,0,1,0,0,0,0,1};auto h=g::first_world_mesh_intersection({m,n},{{"a",ta},{"b",tb}},{0,0,0},{5,0,0});ASSERT_TRUE(h);EXPECT_EQ(h->link_frame,"b");
}
TEST(Mesh, RejectsInvalidLoaderInputs) {
  EXPECT_THROW(g::resolve_thesis_collision_uri("package://other/mesh.dae"),std::invalid_argument);
  EXPECT_THROW(g::resolve_thesis_collision_uri("package://thesis_description/nope.dae"),std::invalid_argument);
  EXPECT_THROW(g::first_world_mesh_intersection({}, {}, {std::numeric_limits<double>::quiet_NaN(),0,0},{1,0,0}),std::invalid_argument);
  g::MeshInstance m{"a","x",{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1},{a},{},{},g::collision_mesh_provenance()};g::build_mesh_bvh(&m);std::array<double,16> bad{2,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};EXPECT_THROW(g::first_world_mesh_intersection({m},{{"a",bad}},{0,0,0},{1,0,0}),std::invalid_argument);
}
TEST(Mesh, OfflineDeterministicBenchmark) {
  const auto load_start=std::chrono::steady_clock::now(); auto meshes=g::load_collision_meshes_from_urdf_xml(expanded_jaco());
  const auto build_end=std::chrono::steady_clock::now(); std::vector<g::MeshLinkTransform> transforms;
  const std::array<double,16> identity{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}; for(const auto&m:meshes) transforms.push_back({m.link_frame,identity});
  std::size_t triangles=0,hits=0;for(const auto&m:meshes)triangles+=m.triangles.size();
  const auto query_start=std::chrono::steady_clock::now(); for(int i=0;i<1300;++i){const double x=((i*37)%101-50)*0.002,y=((i*17)%97-48)*0.002;auto h=g::first_world_mesh_intersection(meshes,transforms,{x,y,1.5},{x,y,-1.5});if(h)++hits;}
  const auto query_end=std::chrono::steady_clock::now(); const auto build_ms=std::chrono::duration<double,std::milli>(build_end-load_start).count();const auto query_ms=std::chrono::duration<double,std::milli>(query_end-query_start).count();
  std::cout<<"F8B_BENCH instances="<<meshes.size()<<" triangles="<<triangles<<" build_ms="<<build_ms<<" query_1300_ms="<<query_ms<<" per_ray_us="<<(query_ms*1000.0/1300.0)<<" hits="<<hits<<std::endl;EXPECT_GT(triangles,0U);
  std::ostringstream metrics;metrics<<"instances="<<meshes.size()<<" triangles="<<triangles<<" build_ms="<<build_ms<<" query_1300_ms="<<query_ms<<" per_ray_us="<<(query_ms*1000.0/1300.0)<<" hits="<<hits;RecordProperty("F8B_BENCH",metrics.str());
}
TEST(Mesh, HierarchicalExactEquivalenceAndBenchmark) {
  auto meshes=g::load_collision_meshes_from_urdf_xml(expanded_jaco());const std::array<double,16> id{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};std::vector<g::MeshLinkTransform> transforms;for(const auto&m:meshes)transforms.push_back({m.link_frame,id});
  transforms[0].world_link[3]=0.15; transforms[4].world_link[7]=-0.10; g::MeshQueryProfile profile;auto instances=g::update_world_mesh_instances(meshes,transforms,&profile);RecordProperty("F8C_AABB_UPDATE_MS",std::to_string(profile.world_aabb_update_ms));
  const std::vector<std::pair<g::MeshPoint3,g::MeshPoint3>> cases{{{5,5,2},{5,5,-2}},{{0,0,1.5},{0,0,-1.5}},{{0.02,0,1.5},{0.02,0,-1.5}},{{0,0,1.5},{0,0,0.01}}};
  for(const auto&ray:cases){auto brute=g::first_world_mesh_intersection_brute_force(instances,ray.first,ray.second);auto opt=g::first_world_mesh_intersection_optimized(instances,ray.first,ray.second,.07,&profile);ASSERT_EQ(static_cast<bool>(brute),static_cast<bool>(opt));if(brute){EXPECT_EQ(brute->link_frame,opt->link_frame);EXPECT_NEAR(brute->distance,opt->distance,1e-9);auto cls=[&](const auto&h){return h.delta_ray<-.07?g::MeshClassification::EXTERNAL_CONFIRMED:std::abs(h.delta_ray)<=.07?g::MeshClassification::SELF_CONFIRMED:g::MeshClassification::OCCLUDED_UNKNOWN;};EXPECT_EQ(cls(*brute),cls(*opt));}}
  auto run=[&](int near_count,const char*name){std::vector<std::pair<g::MeshPoint3,g::MeshPoint3>> rays;for(int i=0;i<1300;++i){bool near=i<near_count;double x=near?((i*37)%101-50)*.002:5.0+((i*13)%17)*.01;double y=near?((i*17)%97-48)*.002:5.0;rays.push_back({{x,y,1.5},{x,y,-1.5}});}auto b0=std::chrono::steady_clock::now();std::vector<std::optional<g::MeshWorldRayResult>> brute;for(auto&r:rays)brute.push_back(g::first_world_mesh_intersection_brute_force(instances,r.first,r.second));double bm=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-b0).count();g::MeshQueryProfile p;auto o0=std::chrono::steady_clock::now();std::size_t oh=0;for(std::size_t i=0;i<rays.size();++i){auto opt=g::first_world_mesh_intersection_optimized(instances,rays[i].first,rays[i].second,.07,&p);ASSERT_EQ(static_cast<bool>(brute[i]),static_cast<bool>(opt));if(opt){++oh;EXPECT_EQ(brute[i]->link_frame,opt->link_frame);EXPECT_NEAR(brute[i]->distance,opt->distance,1e-9);}}double om=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-o0).count();std::ostringstream v;v<<"brute_ms="<<bm<<" optimized_ms="<<om<<" speedup="<<(bm/om)<<" hits="<<oh<<" aabb="<<p.link_aabbs<<" reject="<<p.links_rejected<<" bvh_links="<<p.links_to_bvh<<" nodes="<<p.bvh_nodes<<" leaves="<<p.bvh_leaves<<" node_aabbs="<<p.node_aabbs<<" triangles="<<p.triangles;RecordProperty(std::string("F8C_")+name,v.str());};
  run(0,"BACKGROUND");run(1300,"NEAR");run(250,"MIXED");
}
