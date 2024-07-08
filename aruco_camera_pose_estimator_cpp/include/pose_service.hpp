#ifndef POSE_SERVICE_HPP
#define POSE_SERVICE_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/aruco.hpp>
#include <rclcpp/rclcpp.hpp>

#include "calibration_srv/srv/calibration.hpp"
#include "image_processor.hpp"

class PoseService : public rclcpp::Node {
 public:
  PoseService();

 private:
  rclcpp::Service<calibration_srv::srv::Calibration>::SharedPtr srv_;
  Eigen::Vector3d mean_tvec_;
  Eigen::Quaterniond mean_quat_;
  bool mean_computed_;
  std::shared_ptr<ImageProcessor> image_processor_node_;
  rclcpp::executors::SingleThreadedExecutor node_executor_;
  u_int32_t timeout_;
  std::vector<double> aruco_XYZ_offset_from_baseframe_;
  std::vector<double> aruco_rot_offset_from_baseframe_;

  void get_camera_pose(const Eigen::Vector3d &tvec,
                       const Eigen::Quaterniond &quat,
                       geometry_msgs::msg::PoseStamped &pose_msg);

  void get_camera_pose_service_callback(
      const std::shared_ptr<calibration_srv::srv::Calibration::Request> request,
      std::shared_ptr<calibration_srv::srv::Calibration::Response> response);
};

#endif  // POSE_SERVICE_HPP
