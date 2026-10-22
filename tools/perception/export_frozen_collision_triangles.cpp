#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <array>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
template<class T> T read_value()
{
  T value{};
  std::cin.read(reinterpret_cast<char *>(&value), sizeof(value));
  if (!std::cin) throw std::runtime_error("truncated input");
  return value;
}

std::array<double, 3> transform(const std::array<double, 16> & matrix,
  const std::array<double, 3> & point)
{
  return {matrix[0]*point[0]+matrix[1]*point[1]+matrix[2]*point[2]+matrix[3],
    matrix[4]*point[0]+matrix[5]*point[1]+matrix[6]*point[2]+matrix[7],
    matrix[8]*point[0]+matrix[9]*point[1]+matrix[10]*point[2]+matrix[11]};
}

struct Triangle {std::uint16_t link; std::array<float, 9> xyz;};
}

int main()
{
  try {
    const auto count = read_value<std::uint32_t>();
    if (count == 0 || count > 100) throw std::runtime_error("invalid mesh count");
    std::vector<Triangle> triangles;
    for (std::uint32_t index = 0; index < count; ++index) {
      auto length = read_value<std::uint16_t>();
      if (length == 0 || length > 4096) throw std::runtime_error("invalid path length");
      std::string path(length, '\0');
      std::cin.read(path.data(), length);
      if (!std::cin) throw std::runtime_error("truncated mesh path");
      std::array<double, 16> world{}, origin{};
      std::array<double, 3> scale{};
      for (auto & value : world) value = read_value<double>();
      for (auto & value : origin) value = read_value<double>();
      for (auto & value : scale) value = read_value<double>();
      Assimp::Importer importer;
      const auto * scene = importer.ReadFile(path,
        aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_PreTransformVertices);
      if (!scene || !scene->HasMeshes()) throw std::runtime_error("Assimp load failed: " + path);
      for (unsigned mesh_index = 0; mesh_index < scene->mNumMeshes; ++mesh_index) {
        const auto * mesh = scene->mMeshes[mesh_index];
        for (unsigned face_index = 0; face_index < mesh->mNumFaces; ++face_index) {
          const auto & face = mesh->mFaces[face_index];
          if (face.mNumIndices != 3) throw std::runtime_error("non-triangle face");
          Triangle triangle{static_cast<std::uint16_t>(index), {}};
          for (unsigned vertex = 0; vertex < 3; ++vertex) {
            const auto & v = mesh->mVertices[face.mIndices[vertex]];
            auto local = transform(origin, {v.x*scale[0], v.y*scale[1], v.z*scale[2]});
            auto world_point = transform(world, local);
            for (unsigned axis = 0; axis < 3; ++axis) {
              triangle.xyz[3*vertex+axis] = static_cast<float>(world_point[axis]);
            }
          }
          triangles.push_back(triangle);
        }
      }
    }
    std::cout.write("TMSH8R1", 7);
    const auto total = static_cast<std::uint32_t>(triangles.size());
    std::cout.write(reinterpret_cast<const char *>(&total), sizeof(total));
    for (const auto & triangle : triangles) {
      std::cout.write(reinterpret_cast<const char *>(&triangle.link), sizeof(triangle.link));
      std::cout.write(reinterpret_cast<const char *>(triangle.xyz.data()),
        sizeof(float) * triangle.xyz.size());
    }
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FROZEN_MESH_EXPORT_ERROR " << error.what() << '\n';
    return 2;
  }
}
