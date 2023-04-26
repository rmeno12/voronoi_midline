#include <gflags/gflags.h>
#include <glog/logging.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <eigen3/Eigen/Dense>
#include <memory>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <vector>

#include "voronoi/voronoi.hpp"

using Eigen::Vector2f;

namespace {

static const Vector2f kLaserOffset(0.2, 0.0);

std::unique_ptr<voronoi::Voronoi> vor_;

ros::Publisher midline_pub_;

cv::Mat image_ = cv::Mat::zeros(720, 1280, CV_8UC3);
const float x_min = -2.0;
const float x_max = 10.0;
const float y_min = -5.0;
const float y_max = 5.0;
const float x_scale = 1280.0 / (x_max - x_min);
const float y_scale = 720.0 / (y_max - y_min);

}  // namespace

void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  static std::vector<Vector2f> cloud;

  // TODO: only do this update if the robot has moved enough
  cloud.clear();
  for (size_t i = 0; i < msg->ranges.size(); i++) {
    float angle = msg->angle_min + i * msg->angle_increment;
    float range = msg->ranges[i];
    if (range < msg->range_min || range >= msg->range_max) continue;
    cloud.emplace_back(range * cos(angle) + kLaserOffset.x(),
                       range * sin(angle) + kLaserOffset.y());
  }
  vor_->UpdatePointcloud(cloud);

  image_.setTo(cv::Scalar(0, 0, 0));  // clear the image

  // show current pointcloud in opencv window
  for (const auto& p : cloud) {
    cv::circle(image_,
               cv::Point((p.x() - x_min) * x_scale, (p.y() - y_min) * y_scale),
               1, cv::Scalar(0, 0, 255), -1);
  }

  // show current voronoi edges in opencv window
  auto verts = vor_->GetVoronoiVertices();
  auto edges = vor_->GetPrunedVoronoiEdges();
  for (long int i = 0; i < edges.rows(); i++) {
    for (long int j = i; j < edges.cols(); j++) {
      if (edges(i, j) > 0) {
        cv::line(image_,
                 cv::Point((verts[i].x() / 100 - x_min) * x_scale,
                           (verts[i].y() / 100 - y_min) * y_scale),
                 cv::Point((verts[j].x() / 100 - x_min) * x_scale,
                           (verts[j].y() / 100 - y_min) * y_scale),
                 cv::Scalar(0, 255, 0), 1);
      }
    }
  }

  cv::imshow("voronoi", image_);
  cv::waitKey(1);

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
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);

  vor_ = std::make_unique<voronoi::Voronoi>();

  cv::namedWindow("voronoi", cv::WINDOW_AUTOSIZE);

  ros::init(argc, argv, "voronoi");
  ros::NodeHandle nh;

  midline_pub_ = nh.advertise<nav_msgs::Path>("/midline", 1);

  ros::Subscriber laser_sub = nh.subscribe("/scan", 1, LaserCallback);

  ros::spin();
}
