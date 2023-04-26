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

using Eigen::Vector2f;

namespace {

const Vector2f kLaserOffset(0.2, 0.0);

std::vector<Vector2f> pointcloud_;

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

  pointcloud_ = cloud;
}

void MidlineCallback(const nav_msgs::Path::ConstPtr& msg) {
  image_.setTo(cv::Scalar(0, 0, 0));  // clear the image

  // show current pointcloud in opencv window
  for (const auto& p : pointcloud_) {
    cv::circle(image_,
               cv::Point((p.x() - x_min) * x_scale, (p.y() - y_min) * y_scale),
               1, cv::Scalar(0, 0, 255), -1);
  }

  // show current midline in opencv window
  for (size_t i = 0; i < msg->poses.size() - 1; i++) {
    const auto& p1 = msg->poses[i].pose.position;
    const auto& p2 = msg->poses[i + 1].pose.position;
    cv::line(image_,
             cv::Point((p1.x - x_min) * x_scale, (p1.y - y_min) * y_scale),
             cv::Point((p2.x - x_min) * x_scale, (p2.y - y_min) * y_scale),
             cv::Scalar(0, 255, 0), 1);
  }

  cv::imshow("voronoi_vis", image_);
  cv::waitKey(1);
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);

  ros::init(argc, argv, "voronoi_vis");
  ros::NodeHandle nh;

  ros::Subscriber laser_sub = nh.subscribe("/scan", 1, LaserCallback);
  ros::Subscriber midline_sub = nh.subscribe("/midline", 1, MidlineCallback);

  cv::namedWindow("voronoi_vis", cv::WINDOW_AUTOSIZE);

  ros::spin();
}
