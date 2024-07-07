#include "image_processor.hpp"

#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

ImageProcessor::ImageProcessor(
    const std::string &topic_name, const uint32_t N, Eigen::Vector3d &mean_tvec,
    Eigen::Quaterniond &mean_quat, bool &mean_computed, bool show_img,
    uint16_t aruco_marker_id, double arucobase_size, double cx, double cy,
    double fx, double fy, double k1, double k2, double k3, double p1, double p2)
    : Node("image_processor"),
      topic_name_(topic_name),
      N_(N),
      mean_tvec_(mean_tvec),
      mean_quat_(mean_quat),
      mean_computed_(mean_computed),
      show_img_(show_img),
      aruco_marker_id_(aruco_marker_id),
      arucobase_size_(arucobase_size),
      cx_(cx),
      cy_(cy),
      fx_(fx),
      fy_(fy),
      k1_(k1),
      k2_(k2),
      k3_(k3),
      p1_(p1),
      p2_(p2) {
  idx_ = 0;
  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  detector_params_ = cv::aruco::DetectorParameters::create();
  tvec_cam_list_.resize(N_, cv::Vec3d(0, 0, 0));
  rvec_cam_list_.resize(N_, cv::Vec3d(0, 0, 0));

  distortion_camera_ = (cv::Mat_<double>(1, 5) << k1_, k2_, p1_, p2_, k3_);
  intrinsic_camera_ =
      (cv::Mat_<double>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
}

bool ImageProcessor::camera_pose_estimation(cv::Mat &frame,
                                            cv::Vec3d &tvec_camera,
                                            cv::Vec3d &rvec_camera) {
  cv::Mat gray;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detector_params_);

  if (!ids.empty()) {
    for (size_t i = 0; i < ids.size(); ++i) {
      if (ids[i] == aruco_marker_id_) {
        // RCLCPP_INFO(this->get_logger(), "Detected Aruco base marker");
        std::vector<cv::Vec3d> rvec, tvec;
        cv::aruco::estimatePoseSingleMarkers(corners, arucobase_size_,
                                             intrinsic_camera_,
                                             distortion_camera_, rvec, tvec);
        tvec_camera = tvec[0];
        rvec_camera = rvec[0];
        // Draw axis for the aruco marker
        if (show_img_) {
          cv::aruco::drawAxis(frame, intrinsic_camera_, distortion_camera_,
                              rvec[0], tvec[0], 0.1);
          cv::imshow("Aruco Marker", frame);
          cv::waitKey(1);
        }
        return true;
      }
    }
  }
  return false;
}

Eigen::Quaterniond ImageProcessor::quaternion_avg(
    const std::vector<Eigen::Quaterniond> &quat_list) {
  Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
  for (const auto &q : quat_list) {
    Eigen::Vector4d q_vec(q.w(), q.x(), q.y(), q.z());
    M += q_vec * q_vec.transpose();
  }
  M /= quat_list.size();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(M);
  Eigen::Vector4d avg_q = solver.eigenvectors().col(3);
  return Eigen::Quaterniond(avg_q[0], avg_q[1], avg_q[2], avg_q[3]);
  return quat_list[0];
}

void ImageProcessor::image_callback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  cv::Vec3d tvec_camera, rvec_camera;
  bool success =
      camera_pose_estimation(cv_ptr->image, tvec_camera, rvec_camera);
  if (success) {
    tvec_cam_list_[idx_] = tvec_camera;
    rvec_cam_list_[idx_] = rvec_camera;
    idx_++;
  }
  // Explicitly reset cv_ptr to free memory
  cv_ptr.reset();

  // Check if we've reached N samples
  if (idx_ == static_cast<int>(N_)) {
    idx_ = 0;
    Eigen::Vector3d mean_tvec = Eigen::Vector3d::Zero();
    for (const auto &tvec : tvec_cam_list_) {
      mean_tvec += Eigen::Vector3d(tvec[0], tvec[1], tvec[2]);
    }
    mean_tvec /= tvec_cam_list_.size();
    mean_tvec_ = mean_tvec;
    std::vector<Eigen::Quaterniond> quat_list(tvec_cam_list_.size());
    cv::Mat R_cv;
    Eigen::Matrix3d R;
    for (size_t i = 0; i < tvec_cam_list_.size(); ++i) {
      cv::Rodrigues(rvec_cam_list_[i], R_cv);
      cv::cv2eigen(R_cv, R);
      quat_list[i] = Eigen::Quaterniond(R).normalized();
    }
    mean_quat_ = quaternion_avg(quat_list);
    // Lock mean_computed with mutex (if multithreading is involved)
    mean_computed_ = true;
  } else {
    if (idx_ >= static_cast<int>(N_)) {
      idx_ = 0;
    }
  }
}
