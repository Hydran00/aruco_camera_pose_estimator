#include "pose_service.hpp"

using std::chrono::milliseconds;

PoseService::PoseService()
    : Node("pose_service"),
      computing_avg_(false),
      mean_tvec_(Eigen::Vector3d::Zero()),
      mean_quat_(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)) {
  this->declare_parameter("input_topic_name", "/camera/camera/color/image_raw");
  input_topic_name_ = this->get_parameter("input_topic_name").as_string();

  this->declare_parameter("output_service_name",
                          "/calibration/get_camera_pose");

  this->declare_parameter("n_observation", 10);
  N_ = this->get_parameter("n_observation").as_int();

  this->declare_parameter("timeout_ms", 2000);
  timeout_ = (u_int32_t)this->get_parameter("timeout_ms").as_int();

  srv_ = this->create_service<calibration_srv::srv::Calibration>(
      this->get_parameter("output_service_name").as_string(),
      std::bind(&PoseService::get_camera_pose_service_callback, this,
                std::placeholders::_1, std::placeholders::_2));

  // Aruco parameters
  this->declare_parameter("show_img", true);
  this->declare_parameter("aruco_marker_id", 0);
  this->declare_parameter("aruco_size", 0.10);  // in meters

  // Aruco offsets from base frame
  this->declare_parameter("aruco_XYZ_offset_from_baseframe",
                          std::vector<double>{0.0, 0.0, 0.0});
  aruco_XYZ_offset_from_baseframe_ =
      this->get_parameter("aruco_XYZ_offset_from_baseframe").as_double_array();

  this->declare_parameter("aruco_rot_offset_from_baseframe",
                          std::vector<double>{1.0, 0.0, 0.0, 0.0});
  aruco_rot_offset_from_baseframe_ =
      this->get_parameter("aruco_rot_offset_from_baseframe").as_double_array();

  // camera intrinsics
  this->declare_parameter("cx", 958.620910644531);
  this->declare_parameter("cy", 564.277893066406);
  this->declare_parameter("fx", 1376.80395507812);
  this->declare_parameter("fy", 1376.80322265625);

  mean_computed_ = false;

  image_processor_node_ = std::make_shared<ImageProcessor>(
      input_topic_name_, N_, mean_tvec_, mean_quat_, mean_computed_,
      this->get_parameter("show_img").as_bool(),
      this->get_parameter("aruco_marker_id").as_int(),
      this->get_parameter("aruco_size").as_double(),
      this->get_parameter("cx").as_double(),
      this->get_parameter("cy").as_double(),
      this->get_parameter("fx").as_double(),
      this->get_parameter("fy").as_double());

  node_executor_.add_node(image_processor_node_);

  // print each parameter
  RCLCPP_INFO(this->get_logger(),
              "Parameters: input_topic_name: %s, output_service_name: %s, "
              "n_observation: %d, "
              "timeout_ms: %d, show_img: %d, aruco_marker_id: %ld, "
              "aruco_size: %f, cx: %f, cy: %f, fx: %f, fy: %f",
              input_topic_name_.c_str(),
              this->get_parameter("output_service_name").as_string().c_str(),
              N_, timeout_, this->get_parameter("show_img").as_bool(),
              this->get_parameter("aruco_marker_id").as_int(),
              this->get_parameter("aruco_size").as_double(),
              this->get_parameter("cx").as_double(),
              this->get_parameter("cy").as_double(),
              this->get_parameter("fx").as_double(),
              this->get_parameter("fy").as_double());
  RCLCPP_INFO(this->get_logger(), "aruco_XYZ_offset_from_baseframe: %f %f %f",
              aruco_XYZ_offset_from_baseframe_[0],
              aruco_XYZ_offset_from_baseframe_[1],
              aruco_XYZ_offset_from_baseframe_[2]);
  RCLCPP_INFO(
      this->get_logger(), "aruco_rot_offset_from_baseframe: %f %f %f %f",
      aruco_rot_offset_from_baseframe_[0], aruco_rot_offset_from_baseframe_[1],
      aruco_rot_offset_from_baseframe_[2], aruco_rot_offset_from_baseframe_[3]);
}

void PoseService::get_camera_pose(const Eigen::Vector3d &tvec,
                                  const Eigen::Quaterniond &quat,
                                  geometry_msgs::msg::PoseStamped &pose_msg) {
  // rTc = rTa * cTa -> rTc = rTa * cTa where r is robot frame, a is aruco
  // frame, c is camera frame
  Eigen::Matrix3d R = quat.toRotationMatrix();

  auto tvec_cTa = -R.transpose() * tvec;
  Eigen::Quaterniond q(R.transpose());

  // create aTc 4x4 matrix
  Eigen::Matrix4d T_aruco_to_camera = Eigen::Matrix4d::Identity();
  T_aruco_to_camera.block<3, 3>(0, 0) = R.transpose();
  T_aruco_to_camera.block<3, 1>(0, 3) = tvec_cTa;

  // create rTa 4x4 matrix
  Eigen::Matrix4d T_robot_to_aruco = Eigen::Matrix4d::Identity();
  T_robot_to_aruco.block<3, 3>(0, 0) =
      Eigen::Quaterniond(aruco_rot_offset_from_baseframe_[0],
                         aruco_rot_offset_from_baseframe_[1],
                         aruco_rot_offset_from_baseframe_[2],
                         aruco_rot_offset_from_baseframe_[3])
          .normalized()
          .toRotationMatrix();

  T_robot_to_aruco.block<3, 1>(0, 3) = Eigen::Vector3d(
      aruco_XYZ_offset_from_baseframe_[0], aruco_XYZ_offset_from_baseframe_[1],
      aruco_XYZ_offset_from_baseframe_[2]);

  // create rTc 4x4 matrix
  Eigen::Matrix4d T_robot_to_camera = T_robot_to_aruco * T_aruco_to_camera;

  pose_msg.pose.position.x = T_robot_to_camera(0, 3);
  pose_msg.pose.position.y = T_robot_to_camera(1, 3);
  pose_msg.pose.position.z = T_robot_to_camera(2, 3);

  Eigen::Quaterniond q_rTc(T_robot_to_camera.block<3, 3>(0, 0));
  pose_msg.pose.orientation.w = q_rTc.w();
  pose_msg.pose.orientation.x = q_rTc.x();
  pose_msg.pose.orientation.y = q_rTc.y();
  pose_msg.pose.orientation.z = q_rTc.z();
}

void PoseService::get_camera_pose_service_callback(
    const std::shared_ptr<calibration_srv::srv::Calibration::Request> request,
    std::shared_ptr<calibration_srv::srv::Calibration::Response> response) {
  (void)request;
  RCLCPP_INFO(this->get_logger(), "Request received");

  mean_computed_ = false;

  // Unsubscribe if already subscribed
  if (image_processor_node_->sub_) {
    image_processor_node_->sub_.reset();
  }

  // subscribe to the camera topic
  image_processor_node_->sub_ =
      image_processor_node_->create_subscription<sensor_msgs::msg::Image>(
          input_topic_name_, 1,
          std::bind(&ImageProcessor::image_callback, image_processor_node_,
                    std::placeholders::_1));

  auto start_time = this->get_clock()->now();
  while (rclcpp::ok() && !mean_computed_ &&
         (this->get_clock()->now() - start_time) < milliseconds(timeout_)) {
    node_executor_.spin_some();
  }

  if (!mean_computed_) {
    if (image_processor_node_->idx_ == 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "No Aruco found, possible reasons: topic name wrong or "
                   "Aruco not in the field of view!");
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Timeout reached before collecting %d observations: just %d "
                   "measurements collected, try improving the timeout time!",
                   N_, image_processor_node_->idx_);
    }
    response->camera_pose.header.frame_id = "error";
    return;
  }
  // unsubscribe from the camera topic
  image_processor_node_->sub_.reset();
  // node_executor_.remove_node(image_processor_node_);
  RCLCPP_INFO(this->get_logger(), "Mean computed correctly");
  get_camera_pose(mean_tvec_, mean_quat_, response->camera_pose);
  response->camera_pose.header.frame_id = "base_link";
  response->camera_pose.header.stamp = this->get_clock()->now();

  mean_tvec_ = Eigen::Vector3d::Zero();
  mean_quat_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseService>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
