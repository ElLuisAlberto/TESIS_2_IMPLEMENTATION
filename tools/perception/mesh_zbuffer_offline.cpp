// Offline pinhole triangle z-buffer for frozen collision meshes. No ROS interfaces.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <vector>

struct Triangle {
  std::uint16_t link;
  float xyz[9];
};

template<class T> bool read_exact(T & value) {
  return std::fread(&value, sizeof(T), 1, stdin) == 1;
}

template<class T> bool read_many(T * values, std::size_t count) {
  return std::fread(values, sizeof(T), count, stdin) == count;
}

int main() {
  std::uint32_t width, height, count;
  double intrinsic[4];
  if (!read_exact(width) || !read_exact(height) || !read_exact(count) ||
      !read_many(intrinsic, 4) || width == 0 || height == 0 || count > 1000000) return 2;
  std::vector<Triangle> triangles(count);
  if (!read_many(triangles.data(), count)) return 3;
  const std::size_t pixels = static_cast<std::size_t>(width) * height;
  std::vector<float> depth(pixels);
  std::vector<std::uint8_t> labels(pixels);
  double camera[16];
  while (read_many(camera, 16)) {
    std::fill(depth.begin(), depth.end(), std::numeric_limits<float>::infinity());
    std::fill(labels.begin(), labels.end(), 255);
    for (const auto & triangle : triangles) {
      double u[3], v[3], invz[3];
      bool visible = true;
      for (int k = 0; k < 3; ++k) {
        const double x = triangle.xyz[3*k], y = triangle.xyz[3*k+1], z = triangle.xyz[3*k+2];
        const double cx = camera[0]*x + camera[1]*y + camera[2]*z + camera[3];
        const double cy = camera[4]*x + camera[5]*y + camera[6]*z + camera[7];
        const double cz = camera[8]*x + camera[9]*y + camera[10]*z + camera[11];
        if (!std::isfinite(cz) || cz <= 0.02) {visible = false; break;}
        invz[k] = 1.0/cz;
        u[k] = intrinsic[0]*cx*invz[k] + intrinsic[2];
        v[k] = intrinsic[1]*cy*invz[k] + intrinsic[3];
      }
      if (!visible) continue;
      const double det = (v[1]-v[2])*(u[0]-u[2]) + (u[2]-u[1])*(v[0]-v[2]);
      if (std::abs(det) < 1e-9 || !std::isfinite(det)) continue;
      const int left = std::max(0, static_cast<int>(std::floor(std::min({u[0],u[1],u[2]}))));
      const int right = std::min(static_cast<int>(width)-1,
        static_cast<int>(std::ceil(std::max({u[0],u[1],u[2]}))));
      const int top = std::max(0, static_cast<int>(std::floor(std::min({v[0],v[1],v[2]}))));
      const int bottom = std::min(static_cast<int>(height)-1,
        static_cast<int>(std::ceil(std::max({v[0],v[1],v[2]}))));
      if (left > right || top > bottom) continue;
      const double invdet = 1.0/det;
      for (int py = top; py <= bottom; ++py) {
        for (int px = left; px <= right; ++px) {
          const double sx = px + 0.5, sy = py + 0.5;
          const double a = ((v[1]-v[2])*(sx-u[2]) + (u[2]-u[1])*(sy-v[2]))*invdet;
          const double b = ((v[2]-v[0])*(sx-u[2]) + (u[0]-u[2])*(sy-v[2]))*invdet;
          const double c = 1-a-b;
          if (a < -1e-7 || b < -1e-7 || c < -1e-7) continue;
          const float z = static_cast<float>(1.0/(a*invz[0] + b*invz[1] + c*invz[2]));
          const std::size_t index = static_cast<std::size_t>(py)*width + px;
          if (z < depth[index]) {depth[index] = z; labels[index] = static_cast<std::uint8_t>(triangle.link);}
        }
      }
    }
    if (std::fwrite(depth.data(), sizeof(float), pixels, stdout) != pixels ||
        std::fwrite(labels.data(), 1, pixels, stdout) != pixels || std::fflush(stdout)) return 4;
  }
  return 0;
}
