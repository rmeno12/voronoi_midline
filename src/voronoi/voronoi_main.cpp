#include <gflags/gflags.h>
#include <glog/logging.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>

#include "visualization/visualization.h"
#include "voronoi/voronoi.hpp"

using Eigen::Vector2f;

namespace {

static const Vector2f kLaserOffset(0.2, 0.0);

std::unique_ptr<voronoi::Voronoi> vor_;

amrl_msgs::VisualizationMsg local_viz_msg_;
ros::Publisher midline_pub_;
ros::Publisher viz_pub_;

}  // namespace

void DrawMidline(const std::vector<Vector2f>& midline) {
  for (size_t i = 0; i < midline.size() - 1; i++) {
    const auto& p1 = midline[i];
    const auto& p2 = midline[i + 1];
    visualization::DrawLine(p1, p2, 0xff0000, local_viz_msg_);
  }
}

void DrawVoronoi(const std::vector<Vector2f>& voronoi_vertices,
                 const Eigen::MatrixXf& voronoi_edges) {
  for (long int i = 0; i < voronoi_edges.rows(); i++) {
    for (long int j = i; j < voronoi_edges.cols(); j++) {
      if (voronoi_edges(i, j) > 0) {
        const auto& p1 = voronoi_vertices[i] / FLAGS_scale;
        const auto& p2 = voronoi_vertices[j] / FLAGS_scale;
        visualization::DrawLine(p1, p2, 0x0000ff, local_viz_msg_);
      }
    }
  }
}

void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  static std::vector<Vector2f> cloud;

  // TODO: only do this update if the robot has moved enough
  cloud.clear();
  for (size_t i = 0; i < msg->ranges.size(); i++) {
    float angle = msg->angle_min + i * msg->angle_increment;
    float range = msg->ranges[i];
    cloud.emplace_back(range * cos(angle) + kLaserOffset.x(),
                       range * sin(angle) + kLaserOffset.y());
  }
  vor_->UpdatePointcloud(cloud);

  auto midline = vor_->GetMidline();

  nav_msgs::Path path;
  path.header.frame_id = "base_link";
  path.header.stamp = ros::Time::now();
  for (const auto& point : midline) {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    path.poses.push_back(pose);
  }
  midline_pub_.publish(path);

  visualization::ClearVisualizationMsg(local_viz_msg_);
  DrawVoronoi(vor_->GetVoronoiVertices(), vor_->GetPrunedVoronoiEdges());
  DrawMidline(midline);
  viz_pub_.publish(local_viz_msg_);
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);

  vor_ = std::make_unique<voronoi::Voronoi>();

  local_viz_msg_ =
      visualization::NewVisualizationMessage("base_link", "voronoi_local");

  ros::init(argc, argv, "voronoi");
  ros::NodeHandle nh;

  midline_pub_ = nh.advertise<nav_msgs::Path>("/midline", 1);
  viz_pub_ = nh.advertise<amrl_msgs::VisualizationMsg>("/visualization", 1);

  ros::Subscriber laser_sub = nh.subscribe("/scan", 1, LaserCallback);

  ros::spin();
}
