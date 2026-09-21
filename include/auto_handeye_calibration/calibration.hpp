#pragma once

#include "auto_handeye_calibration/types.hpp"

#include <industrial_calibration/target_finders/opencv/charuco_grid_target_finder.h>

#include <map>
#include <string>
#include <vector>

namespace auto_handeye_calibration
{
class CalibrationEngine
{
public:
  CalibrationEngine(int squares_x, int squares_y, double square_length,
                    double marker_length, int dictionary_id);

  industrial_calibration::Correspondence2D3D::Set detect(
      const cv::Mat& image, cv::Mat* annotated = nullptr) const;
  std::vector<Eigen::Vector3d> allTargetPoints() const;
  bool estimateCameraToTarget(const industrial_calibration::Correspondence2D3D::Set& correspondences,
                              const CameraModel& camera,
                              Eigen::Isometry3d& camera_to_target,
                              double& rms_px) const;
  std::map<std::string, CalibrationResult> initializeAll(
      const std::vector<Sample>& samples, const CameraModel& camera,
      CalibrationMode mode) const;
  CalibrationResult refine(const std::vector<Sample>& samples,
                           const CameraModel& camera,
                           CalibrationMode mode,
                           const CalibrationResult& seed) const;
  double reprojectionRms(const std::vector<Sample>& samples,
                         const CameraModel& camera,
                         CalibrationMode mode,
                         const Eigen::Isometry3d& camera_mount_to_camera,
                         const Eigen::Isometry3d& target_mount_to_target) const;

private:
  industrial_calibration::CharucoGridTarget target_;
  industrial_calibration::CharucoGridBoardTargetFinder finder_;
};
}  // namespace auto_handeye_calibration
