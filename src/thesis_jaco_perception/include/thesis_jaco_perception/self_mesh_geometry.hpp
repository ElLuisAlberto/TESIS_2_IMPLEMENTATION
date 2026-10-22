#ifndef THESIS_JACO_PERCEPTION__SELF_MESH_GEOMETRY_HPP_
#define THESIS_JACO_PERCEPTION__SELF_MESH_GEOMETRY_HPP_

#include <cstddef>
#include <array>
#include <optional>
#include <string>
#include <vector>

namespace thesis_jaco_perception
{
struct MeshPoint3 {double x; double y; double z;};
struct MeshTriangle {MeshPoint3 a; MeshPoint3 b; MeshPoint3 c; std::size_t link_index;};
struct MeshRayResult {double distance; double delta_ray; std::size_t link_index;};
enum class MeshClassification {EXTERNAL_CONFIRMED, SELF_CONFIRMED, OCCLUDED_UNKNOWN};
struct MeshProvenance {bool session_only{true}; bool not_for_safety{true}; std::string geometry_kind;};
struct MeshAabb {MeshPoint3 min; MeshPoint3 max;};
struct MeshBvhNode {MeshAabb bounds; int left{-1}; int right{-1}; std::size_t first{0}; std::size_t count{0};};
struct MeshInstance {
  std::string link_frame; std::string uri; std::array<double, 16> local_collision_transform;
  std::vector<MeshTriangle> triangles; std::vector<std::size_t> bvh_indices;
  std::vector<MeshBvhNode> bvh; MeshProvenance provenance;
};
struct MeshLinkTransform {std::string link_frame; std::array<double, 16> world_link;};
struct MeshWorldRayResult {double distance; double delta_ray; std::string link_frame; std::string uri;};
struct MeshWorldInstance {const MeshInstance * mesh; MeshLinkTransform transform; MeshAabb world_aabb;};
struct MeshQueryProfile {std::size_t rays{0}; std::size_t link_aabbs{0}; std::size_t links_rejected{0}; std::size_t links_to_bvh{0}; std::size_t bvh_nodes{0}; std::size_t bvh_leaves{0}; std::size_t node_aabbs{0}; std::size_t triangles{0}; double world_aabb_update_ms{0.0}; double query_ms{0.0};};

bool is_finite(const MeshPoint3 & point);
bool is_non_degenerate(const MeshTriangle & triangle);
MeshTriangle transform_triangle(const MeshTriangle & triangle, const double transform[16]);
std::optional<MeshRayResult> first_mesh_intersection(
  const std::vector<MeshTriangle> & triangles, const MeshPoint3 & camera, const MeshPoint3 & observed);
MeshClassification classify_mesh_observation(
  const std::vector<MeshTriangle> & triangles, const MeshPoint3 & camera,
  const MeshPoint3 & observed, double epsilon);
MeshProvenance collision_mesh_provenance();
MeshProvenance visual_shadow_only_provenance();
std::string resolve_thesis_collision_uri(const std::string & uri);
std::vector<MeshInstance> load_collision_meshes_from_urdf_xml(const std::string & urdf_xml);
void build_mesh_bvh(MeshInstance * mesh);
std::optional<MeshRayResult> first_bvh_intersection(
  const MeshInstance & mesh, const MeshPoint3 & camera_link, const MeshPoint3 & observed_link);
std::optional<MeshWorldRayResult> first_world_mesh_intersection(
  const std::vector<MeshInstance> & meshes, const std::vector<MeshLinkTransform> & transforms,
  const MeshPoint3 & camera_world, const MeshPoint3 & observed_world);
std::vector<MeshWorldInstance> update_world_mesh_instances(
  const std::vector<MeshInstance> & meshes, const std::vector<MeshLinkTransform> & transforms,
  MeshQueryProfile * profile = nullptr);
std::optional<MeshWorldRayResult> first_world_mesh_intersection_brute_force(
  const std::vector<MeshWorldInstance> & instances, const MeshPoint3 & camera_world,
  const MeshPoint3 & observed_world);
std::optional<MeshWorldRayResult> first_world_mesh_intersection_optimized(
  const std::vector<MeshWorldInstance> & instances, const MeshPoint3 & camera_world,
  const MeshPoint3 & observed_world, double epsilon, MeshQueryProfile * profile = nullptr);
}  // namespace thesis_jaco_perception
#endif
