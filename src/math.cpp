#include "auto_handeye_calibration/math.hpp"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace auto_handeye_calibration
{
Eigen::Isometry3d lookAtOptical(const Eigen::Vector3d& camera_position,
                               const Eigen::Vector3d& target,
                               const Eigen::Vector3d& preferred_up,
                               double roll_rad)
{
  Eigen::Vector3d z = (target - camera_position).normalized();
  Eigen::Vector3d x = z.cross(preferred_up);
  if (x.norm() < 1e-8)
    x = z.cross(Eigen::Vector3d::UnitX());
  x.normalize();
  Eigen::Vector3d y = z.cross(x).normalized();  // optical +Y points down
  Eigen::AngleAxisd roll(roll_rad, z);
  x = roll * x;
  y = roll * y;
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear().col(0) = x;
  result.linear().col(1) = y;
  result.linear().col(2) = z;
  result.translation() = camera_position;
  return result;
}

Eigen::Isometry3d perturb(const Eigen::Isometry3d& transform, int dimension, double delta)
{
  Eigen::Isometry3d increment = Eigen::Isometry3d::Identity();
  if (dimension < 3)
    increment.translation()[dimension] = delta;
  else
    increment.linear() = Eigen::AngleAxisd(delta, Eigen::Vector3d::Unit(dimension - 3)).toRotationMatrix();
  return transform * increment;
}

double rotationDistance(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b)
{
  return Eigen::AngleAxisd(a.transpose() * b).angle();
}

double logDetPositive(const Eigen::MatrixXd& matrix, double damping)
{
  Eigen::MatrixXd regularized = matrix;
  regularized.diagonal().array() += damping;
  Eigen::LDLT<Eigen::MatrixXd> ldlt(regularized);
  if (ldlt.info() != Eigen::Success)
    throw std::runtime_error("Information matrix factorization failed");
  const auto diagonal = ldlt.vectorD();
  if ((diagonal.array() <= 0.0).any())
    throw std::runtime_error("Information matrix is not positive definite");
  return diagonal.array().log().sum();
}

std::vector<cv::Point2d> project(const std::vector<Eigen::Vector3d>& points,
                                const Eigen::Isometry3d& camera_to_target,
                                const CameraModel& camera)
{
  std::vector<cv::Point3d> object_points;
  object_points.reserve(points.size());
  for (const auto& point : points)
    object_points.emplace_back(point.x(), point.y(), point.z());
  Eigen::AngleAxisd angle_axis(camera_to_target.linear());
  cv::Mat rvec = (cv::Mat_<double>(3, 1) << angle_axis.axis().x() * angle_axis.angle(),
                  angle_axis.axis().y() * angle_axis.angle(), angle_axis.axis().z() * angle_axis.angle());
  cv::Mat tvec = (cv::Mat_<double>(3, 1) << camera_to_target.translation().x(),
                  camera_to_target.translation().y(), camera_to_target.translation().z());
  cv::Mat k = (cv::Mat_<double>(3, 3) << camera.fx, 0.0, camera.cx, 0.0,
               camera.fy, camera.cy, 0.0, 0.0, 1.0);
  cv::Mat distortion(camera.distortion, true);
  std::vector<cv::Point2d> pixels;
  cv::projectPoints(object_points, rvec, tvec, k, distortion, pixels);
  return pixels;
}

Eigen::MatrixXd numericalProjectionJacobian(
    const std::vector<Eigen::Vector3d>& points,
    const Eigen::Isometry3d& base_to_flange,
    const Eigen::Isometry3d& flange_to_camera,
    const Eigen::Isometry3d& base_to_target,
    const CameraModel& camera,
    double translation_step,
    double rotation_step)
{
  Eigen::MatrixXd jacobian(points.size() * 2, 12);
  for (int parameter = 0; parameter < 12; ++parameter)
  {
    const int local = parameter % 6;
    const double step = local < 3 ? translation_step : rotation_step;
    Eigen::Isometry3d plus_camera = flange_to_camera;
    Eigen::Isometry3d minus_camera = flange_to_camera;
    Eigen::Isometry3d plus_target = base_to_target;
    Eigen::Isometry3d minus_target = base_to_target;
    if (parameter < 6)
    {
      plus_camera = perturb(flange_to_camera, local, step);
      minus_camera = perturb(flange_to_camera, local, -step);
    }
    else
    {
      plus_target = perturb(base_to_target, local, step);
      minus_target = perturb(base_to_target, local, -step);
    }
    const auto plus = project(points, (base_to_flange * plus_camera).inverse() * plus_target, camera);
    const auto minus = project(points, (base_to_flange * minus_camera).inverse() * minus_target, camera);
    for (std::size_t i = 0; i < points.size(); ++i)
    {
      jacobian(2 * i, parameter) = (plus[i].x - minus[i].x) / (2.0 * step);
      jacobian(2 * i + 1, parameter) = (plus[i].y - minus[i].y) / (2.0 * step);
    }
  }
  return jacobian;
}
}  // namespace auto_handeye_calibration
