#include "pose_service.hpp"

using std::chrono::milliseconds;

PoseService::PoseService()
    : Node("pose_service"),
      mean_tvec_(Eigen::Vector3d::Zero()),
      mean_quat_(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0))
{
  this->declare_parameter("input_topic_name", "/camera/camera/color/image_raw");
  this->declare_parameter("output_service_name",
                          "/calibration/get_camera_pose");
  this->declare_parameter("n_observation", 10);
  this->declare_parameter("timeout_ms", 2000);
  timeout_ = (u_int32_t)this->get_parameter("timeout_ms").as_int();

  srv_ = this->create_service<calibration_srv::srv::Calibration>(
      this->get_parameter("output_service_name").as_string(),
      std::bind(&PoseService::get_camera_pose_service_callback, this,
                std::placeholders::_1, std::placeholders::_2));

  // Aruco parameters
  this->declare_parameter("show_img", true);
  this->declare_parameter("aruco_size", 0.10); // in meters

  // Aruco offsets from base frame
  this->declare_parameter("aruco_XYZ_offset_from_baseframe",
                          std::vector<double>{0.0, 0.0, 0.0});
  aruco_XYZ_offset_from_baseframe_ =
      this->get_parameter("aruco_XYZ_offset_from_baseframe").as_double_array();

  this->declare_parameter("aruco_rot_offset_from_baseframe",
                          std::vector<double>{1.0, 0.0, 0.0, 0.0});
  aruco_rot_offset_from_baseframe_ =
      this->get_parameter("aruco_rot_offset_from_baseframe").as_double_array();

  this->declare_parameter("frame_id", "world");

  // camera intrinsics
  this->declare_parameter("cx", 958.620910644531);
  this->declare_parameter("cy", 564.277893066406);
  this->declare_parameter("fx", 1376.80395507812);
  this->declare_parameter("fy", 1376.80322265625);
  // camera distortion
  this->declare_parameter("k1", 0.0);
  this->declare_parameter("k2", 0.0);
  this->declare_parameter("k3", 0.0);
  this->declare_parameter("p1", 0.0);
  this->declare_parameter("p2", 0.0);

  mean_computed_ = false;

  image_processor_node_ = std::make_shared<ImageProcessor>(
      this->get_parameter("input_topic_name").as_string(),
      this->get_parameter("n_observation").as_int(),
      mean_tvec_, mean_quat_, mean_computed_,
      this->get_parameter("show_img").as_bool(),
      this->get_parameter("aruco_size").as_double(),
      this->get_parameter("cx").as_double(),
      this->get_parameter("cy").as_double(),
      this->get_parameter("fx").as_double(),
      this->get_parameter("fy").as_double(),
      this->get_parameter("k1").as_double(),
      this->get_parameter("k2").as_double(),
      this->get_parameter("k3").as_double(),
      this->get_parameter("p1").as_double(),
      this->get_parameter("p2").as_double());

  node_executor_.add_node(image_processor_node_);

  // print each parameter
  RCLCPP_INFO(this->get_logger(),
              "Parameters: input_topic_name: %s, output_service_name: %s, "
              "n_observation: %ld, "
              "timeout_ms: %ld, show_img: %d, "
              "aruco_size: %f, cx: %f, cy: %f, fx: %f, fy: %f, "
              "k1: %f, k2: %f, k3: %f, p1: %f, p2: %f",
              this->get_parameter("input_topic_name").as_string().c_str(),
              this->get_parameter("output_service_name").as_string().c_str(),
              this->get_parameter("n_observation").as_int(),
              this->get_parameter("timeout_ms").as_int(),
              this->get_parameter("show_img").as_bool(),
              this->get_parameter("aruco_size").as_double(),
              this->get_parameter("cx").as_double(),
              this->get_parameter("cy").as_double(),
              this->get_parameter("fx").as_double(),
              this->get_parameter("fy").as_double(),
              this->get_parameter("k1").as_double(),
              this->get_parameter("k2").as_double(),
              this->get_parameter("k3").as_double(),
              this->get_parameter("p1").as_double(),
              this->get_parameter("p2").as_double());
  RCLCPP_INFO(this->get_logger(), "aruco_XYZ_offset_from_baseframe: %f %f %f",
              aruco_XYZ_offset_from_baseframe_[0],
              aruco_XYZ_offset_from_baseframe_[1],
              aruco_XYZ_offset_from_baseframe_[2]);
  RCLCPP_INFO(
      this->get_logger(), "aruco_rot_offset_from_baseframe: %f %f %f %f",
      aruco_rot_offset_from_baseframe_[0], aruco_rot_offset_from_baseframe_[1],
      aruco_rot_offset_from_baseframe_[2], aruco_rot_offset_from_baseframe_[3]);
}

void PoseService::get_camera_pose_service_callback(
    const std::shared_ptr<calibration_srv::srv::Calibration::Request> request,
    std::shared_ptr<calibration_srv::srv::Calibration::Response> response)
{
  (void)request;
  RCLCPP_INFO(this->get_logger(), "Request received");

  mean_computed_ = false;

  // Unsubscribe if already subscribed
  if (image_processor_node_->sub_)
  {
    image_processor_node_->sub_.reset();
  }

  RCLCPP_INFO(this->get_logger(), "Received request to find Aruco with id %d",
              request->marker_id);
  // subscribe to the camera topic
  image_processor_node_->sub_ =
      image_processor_node_->create_subscription<sensor_msgs::msg::Image>(
          this->get_parameter("input_topic_name").as_string(), 1,
          std::bind(&ImageProcessor::image_callback, image_processor_node_,
                    std::placeholders::_1));
  image_processor_node_->aruco_marker_id_ = request->marker_id;

  auto start_time = this->get_clock()->now();
  while (rclcpp::ok() && !mean_computed_ &&
         (this->get_clock()->now() - start_time) < milliseconds(timeout_))
  {
    node_executor_.spin_some();
  }

  if (!mean_computed_)
  {
    if (image_processor_node_->idx_ == 0)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "No Aruco found, possible reasons: topic name wrong, "
                   "Aruco id wrong or Aruco not in the field of view!");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Timeout reached before collecting %ld observations: just %d "
                   "measurements collected, try improving the timeout time!",
                   this->get_parameter("n_observation").as_int(),
                   image_processor_node_->idx_);
    }
    response->camera_pose.header.frame_id = "error";
    return;
  }
  // unsubscribe from the camera topic
  image_processor_node_->sub_.reset();

  // node_executor_.remove_node(image_processor_node_);
  RCLCPP_INFO(this->get_logger(), "Mean pose computed correctly");
  get_camera_pose(mean_tvec_, mean_quat_, response->camera_pose);
  response->camera_pose.header.frame_id = this->get_parameter("frame_id").as_string();
  response->camera_pose.header.stamp = this->get_clock()->now();

  mean_tvec_ = Eigen::Vector3d::Zero();
  mean_quat_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
}

void PoseService::get_camera_pose(const Eigen::Vector3d &tvec,
                                  const Eigen::Quaterniond &quat,
                                  geometry_msgs::msg::PoseStamped &pose_msg)
{
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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseService>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
