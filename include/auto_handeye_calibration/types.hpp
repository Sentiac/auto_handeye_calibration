#pragma once

#include <Eigen/Geometry>
#include <industrial_calibration/core/types.h>
#include <opencv2/core.hpp>
#include <rclcpp/time.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace auto_handeye_calibration
{
enum class CalibrationMode { EYE_IN_HAND, EYE_TO_HAND };

struct CameraModel
{
  double fx{0.0}, fy{0.0}, cx{0.0}, cy{0.0};
  int width{0}, height{0};
  std::vector<double> distortion;
};

struct DetectionMetrics
{
  double corner_fraction{0.0};
  double coverage_ratio{0.0};
  double border_margin_px{0.0};
  double blur_score{0.0};
  double reprojection_rms_px{0.0};
};

struct Sample
{
  std::size_t id{0};
  rclcpp::Time image_stamp;
  Eigen::Isometry3d base_to_flange{Eigen::Isometry3d::Identity()};
  std::vector<double> joints;
  industrial_calibration::Correspondence2D3D::Set correspondences;
  DetectionMetrics metrics;
  std::string image_path;
};

struct Candidate
{
  std::string id;
  Eigen::Isometry3d base_to_camera{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d base_to_flange{Eigen::Isometry3d::Identity()};
  double geometric_score{0.0};
  double information_gain{0.0};
  std::string rejection_reason;
};

struct CalibrationResult
{
  bool valid{false};
  Eigen::Isometry3d camera_mount_to_camera{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d target_mount_to_target{Eigen::Isometry3d::Identity()};
  double rms_px{0.0};
  std::string initializer;
};
}  // namespace auto_handeye_calibration
