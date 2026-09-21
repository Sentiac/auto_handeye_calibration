#pragma once

#include "auto_handeye_calibration/types.hpp"

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <vector>

namespace auto_handeye_calibration
{
Eigen::Isometry3d lookAtOptical(const Eigen::Vector3d& camera_position,
                               const Eigen::Vector3d& target,
                               const Eigen::Vector3d& preferred_up,
                               double roll_rad = 0.0);
Eigen::Isometry3d perturb(const Eigen::Isometry3d& transform, int dimension, double delta);
double rotationDistance(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b);
double logDetPositive(const Eigen::MatrixXd& matrix, double damping = 1e-9);
std::vector<cv::Point2d> project(const std::vector<Eigen::Vector3d>& points,
                                const Eigen::Isometry3d& camera_to_target,
                                const CameraModel& camera);
Eigen::MatrixXd numericalProjectionJacobian(
    const std::vector<Eigen::Vector3d>& points,
    const Eigen::Isometry3d& base_to_flange,
    const Eigen::Isometry3d& flange_to_camera,
    const Eigen::Isometry3d& base_to_target,
    const CameraModel& camera,
    double translation_step = 1e-5,
    double rotation_step = 1e-5);
}  // namespace auto_handeye_calibration
