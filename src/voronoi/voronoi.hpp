#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>

namespace voronoi {

class Voronoi {
  std::vector<Eigen::Vector2f> pointcloud_;

 public:
  void UpdatePointcloud(const std::vector<Eigen::Vector2f>& pointcloud);
};

}  // namespace voronoi
