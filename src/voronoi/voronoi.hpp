#pragma once

#include <boost/polygon/voronoi.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>

namespace voronoi {

class Voronoi {
  std::vector<Eigen::Vector2f> pointcloud_;
  std::vector<Eigen::Vector2f> voronoi_vertices_;
  Eigen::MatrixXf pruned_voronoi_edges_;
  std::vector<Eigen::Vector2f> midline_;

  void UpdateMidline();

 public:
  void UpdatePointcloud(const std::vector<Eigen::Vector2f>& pointcloud);
  std::vector<Eigen::Vector2f> GetMidline() const { return midline_; }
};

void PruneEdges(
    const std::vector<Eigen::Vector2f>& clearance_points,
    const std::vector<Eigen::Vector2f>& vor_points,
    const std::vector<boost::polygon::voronoi_edge<double>>& vor_edges,
    Eigen::MatrixXf& edges);

}  // namespace voronoi
