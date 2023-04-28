#pragma once

#include <gflags/gflags.h>

#include <boost/polygon/voronoi.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>

DECLARE_double(scale);

namespace voronoi {

class Voronoi {
  std::vector<Eigen::Vector2f> pointcloud_;
  std::vector<Eigen::Vector2f> voronoi_vertices_;
  Eigen::MatrixXf pruned_voronoi_edges_;
  std::vector<bool> pruned_voronoi_vertices_;
  std::vector<Eigen::Vector2f> midline_;
  Eigen::Vector2f goal_;

  void PruneEdges(
      const std::vector<Eigen::Vector2f>& vor_points,
      const std::vector<boost::polygon::voronoi_edge<double>>& vor_edges);
  int FindStartVertex() const;
  Eigen::Vector2f FindGoalPoint() const;
  void UpdateMidline();

 public:
  void UpdatePointcloud(const std::vector<Eigen::Vector2f>& pointcloud);
  std::vector<Eigen::Vector2f> GetVoronoiVertices() const {
    return voronoi_vertices_;
  }
  Eigen::MatrixXf GetPrunedVoronoiEdges() const {
    return pruned_voronoi_edges_;
  }
  std::vector<Eigen::Vector2f> GetMidline() const { return midline_; }
  Eigen::Vector2f GetGoal() const { return goal_; }
};

}  // namespace voronoi
