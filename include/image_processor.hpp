#ifndef IMAGE_PROCESSOR_HPP
#define IMAGE_PROCESSOR_HPP

#include <cv_bridge/cv_bridge.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/aruco.hpp>
#include <opencv2/core/eigen.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImageProcessor : public rclcpp::Node {
 public:
  ImageProcessor(const std::string &topic_name, uint32_t N,
                 Eigen::Vector3d &mean_tvec, Eigen::Quaterniond &mean_quat,
                 bool &mean_computed, bool show_img, uint16_t aruco_marker_id,
                 double arucobase_size, double cx, double cy, double fx,
                 double fy);
  std::string topic_name_;
  uint32_t N_;
  Eigen::Vector3d &mean_tvec_;
  Eigen::Quaterniond &mean_quat_;
  bool &mean_computed_;
  int idx_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

 private:
  std::vector<cv::Vec3d> rvec_cam_list_;
  std::vector<cv::Vec3d> tvec_cam_list_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  cv::Mat image_;
  bool show_img_;
  uint16_t aruco_marker_id_;
  double arucobase_size_;
  double cx_, cy_, fx_, fy_;
  cv::Mat intrinsic_camera_ = cv::Mat_<double>(3, 3);  // << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  cv::Mat distortion_camera_ = cv::Mat_<double>(1, 5); // << 0, 0, 0, 0, 0);

  bool camera_pose_estimation(cv::Mat &frame, cv::Vec3d &tvec_camera,
                              cv::Vec3d &rvec_camera);

  Eigen::Quaterniond quaternion_avg(
      const std::vector<Eigen::Quaterniond> &quat_list);
};

#endif  // IMAGE_PROCESSOR_HPP
