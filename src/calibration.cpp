#include "auto_handeye_calibration/calibration.hpp"
#include "auto_handeye_calibration/math.hpp"

#include <industrial_calibration/optimizations/extrinsic_hand_eye.h>
#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace auto_handeye_calibration
{
namespace
{
cv::Mat cameraMatrix(const CameraModel& camera)
{
  return (cv::Mat_<double>(3, 3) << camera.fx, 0.0, camera.cx, 0.0, camera.fy,
          camera.cy, 0.0, 0.0, 1.0);
}

Eigen::Isometry3d fromCv(const cv::Mat& rotation, const cv::Mat& translation)
{
  cv::Mat r;
  if (rotation.rows == 3 && rotation.cols == 3)
    r = rotation;
  else
    cv::Rodrigues(rotation, r);
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  for (int row = 0; row < 3; ++row)
  {
    result.translation()[row] = translation.at<double>(row);
    for (int col = 0; col < 3; ++col)
      result.linear()(row, col) = r.at<double>(row, col);
  }
  return result;
}

void toCv(const Eigen::Isometry3d& transform, cv::Mat& rotation, cv::Mat& translation)
{
  rotation = cv::Mat(3, 3, CV_64F);
  translation = cv::Mat(3, 1, CV_64F);
  for (int row = 0; row < 3; ++row)
  {
    translation.at<double>(row) = transform.translation()[row];
    for (int col = 0; col < 3; ++col)
      rotation.at<double>(row, col) = transform.linear()(row, col);
  }
}

Eigen::Isometry3d meanTransform(const std::vector<Eigen::Isometry3d>& transforms)
{
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Vector4d quaternion = Eigen::Vector4d::Zero();
  for (const auto& transform : transforms)
  {
    translation += transform.translation();
    Eigen::Quaterniond q(transform.linear());
    if (quaternion.dot(q.coeffs()) < 0.0)
      q.coeffs() *= -1.0;
    quaternion += q.coeffs();
  }
  Eigen::Isometry3d mean = Eigen::Isometry3d::Identity();
  mean.translation() = translation / static_cast<double>(transforms.size());
  Eigen::Quaterniond q;
  q.coeffs() = quaternion.normalized();
  mean.linear() = q.toRotationMatrix();
  return mean;
}
}  // namespace

CalibrationEngine::CalibrationEngine(int squares_x, int squares_y, double square_length,
                                     double marker_length, int dictionary_id)
  : target_(squares_y, squares_x, static_cast<float>(square_length),
            static_cast<float>(marker_length), dictionary_id), finder_(target_)
{
}

industrial_calibration::Correspondence2D3D::Set CalibrationEngine::detect(
    const cv::Mat& image, cv::Mat* annotated) const
{
  const auto features = finder_.findTargetFeatures(image);
  if (annotated != nullptr)
    *annotated = finder_.drawTargetFeatures(image, features);
  return target_.createCorrespondences(features);
}

std::vector<Eigen::Vector3d> CalibrationEngine::allTargetPoints() const
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(target_.points.size());
  for (const auto& item : target_.points)
    points.push_back(item.second);
  return points;
}

bool CalibrationEngine::estimateCameraToTarget(
    const industrial_calibration::Correspondence2D3D::Set& correspondences,
    const CameraModel& camera, Eigen::Isometry3d& camera_to_target, double& rms_px) const
{
  if (correspondences.size() < 4)
    return false;
  std::vector<cv::Point3d> object;
  std::vector<cv::Point2d> image;
  for (const auto& correspondence : correspondences)
  {
    object.emplace_back(correspondence.in_target.x(), correspondence.in_target.y(), correspondence.in_target.z());
    image.emplace_back(correspondence.in_image.x(), correspondence.in_image.y());
  }
  cv::Mat rvec, tvec;
  if (!cv::solvePnP(object, image, cameraMatrix(camera), cv::Mat(camera.distortion, true), rvec, tvec))
    return false;
  cv::Mat rotation;
  cv::Rodrigues(rvec, rotation);
  camera_to_target = fromCv(rotation, tvec);
  std::vector<cv::Point2d> predicted;
  cv::projectPoints(object, rvec, tvec, cameraMatrix(camera), cv::Mat(camera.distortion, true), predicted);
  double squared_error = 0.0;
  for (std::size_t i = 0; i < image.size(); ++i)
    squared_error += cv::norm(image[i] - predicted[i]) * cv::norm(image[i] - predicted[i]);
  rms_px = std::sqrt(squared_error / static_cast<double>(image.size()));
  return true;
}

std::map<std::string, CalibrationResult> CalibrationEngine::initializeAll(
    const std::vector<Sample>& samples, const CameraModel& camera, CalibrationMode mode) const
{
  std::map<std::string, CalibrationResult> results;
  if (samples.size() < 3 || mode != CalibrationMode::EYE_IN_HAND)
    return results;
  std::vector<cv::Mat> gripper_rotations, gripper_translations, target_rotations, target_translations;
  std::vector<Eigen::Isometry3d> camera_to_targets;
  for (const auto& sample : samples)
  {
    Eigen::Isometry3d camera_to_target;
    double rms = 0.0;
    if (!estimateCameraToTarget(sample.correspondences, camera, camera_to_target, rms))
      continue;
    cv::Mat rg, tg, rt, tt;
    toCv(sample.base_to_flange, rg, tg);
    toCv(camera_to_target, rt, tt);
    gripper_rotations.push_back(rg);
    gripper_translations.push_back(tg);
    target_rotations.push_back(rt);
    target_translations.push_back(tt);
    camera_to_targets.push_back(camera_to_target);
  }
  const std::vector<std::pair<std::string, cv::HandEyeCalibrationMethod>> methods = {
    {"TSAI", cv::CALIB_HAND_EYE_TSAI}, {"PARK", cv::CALIB_HAND_EYE_PARK},
    {"HORAUD", cv::CALIB_HAND_EYE_HORAUD}, {"ANDREFF", cv::CALIB_HAND_EYE_ANDREFF},
    {"DANIILIDIS", cv::CALIB_HAND_EYE_DANIILIDIS}};
  for (const auto& method : methods)
  {
    CalibrationResult result;
    result.initializer = method.first;
    try
    {
      cv::Mat rcg, tcg;
      cv::calibrateHandEye(gripper_rotations, gripper_translations, target_rotations,
                           target_translations, rcg, tcg, method.second);
      result.camera_mount_to_camera = fromCv(rcg, tcg);
      std::vector<Eigen::Isometry3d> targets;
      for (std::size_t i = 0; i < camera_to_targets.size(); ++i)
        targets.push_back(samples[i].base_to_flange * result.camera_mount_to_camera * camera_to_targets[i]);
      result.target_mount_to_target = meanTransform(targets);
      result.rms_px = reprojectionRms(samples, camera, mode, result.camera_mount_to_camera,
                                      result.target_mount_to_target);
      result.valid = std::isfinite(result.rms_px) &&
                     std::abs(result.camera_mount_to_camera.linear().determinant() - 1.0) < 1e-3;
    }
    catch (const cv::Exception&)
    {
      result.valid = false;
    }
    results.emplace(method.first, result);
  }
  return results;
}

CalibrationResult CalibrationEngine::refine(const std::vector<Sample>& samples,
                                            const CameraModel& camera,
                                            CalibrationMode mode,
                                            const CalibrationResult& seed) const
{
  industrial_calibration::ExtrinsicHandEyeProblem2D3D problem;
  problem.intr.fx() = camera.fx;
  problem.intr.fy() = camera.fy;
  problem.intr.cx() = camera.cx;
  problem.intr.cy() = camera.cy;
  problem.camera_mount_to_camera_guess = seed.camera_mount_to_camera;
  problem.target_mount_to_target_guess = seed.target_mount_to_target;
  for (const auto& sample : samples)
  {
    industrial_calibration::Observation2D3D observation;
    observation.correspondence_set = sample.correspondences;
    if (mode == CalibrationMode::EYE_IN_HAND)
      observation.to_camera_mount = sample.base_to_flange;
    else
      observation.to_target_mount = sample.base_to_flange;
    problem.observations.push_back(observation);
  }
  const auto optimized = industrial_calibration::optimize(problem);
  CalibrationResult result;
  result.valid = optimized.converged;
  result.initializer = seed.initializer + "+INDUSTRIAL";
  result.camera_mount_to_camera = optimized.camera_mount_to_camera;
  result.target_mount_to_target = optimized.target_mount_to_target;
  result.rms_px = std::sqrt(std::max(0.0, optimized.final_cost_per_obs));
  return result;
}

double CalibrationEngine::reprojectionRms(
    const std::vector<Sample>& samples, const CameraModel& camera, CalibrationMode mode,
    const Eigen::Isometry3d& camera_mount_to_camera,
    const Eigen::Isometry3d& target_mount_to_target) const
{
  double squared = 0.0;
  std::size_t count = 0;
  for (const auto& sample : samples)
  {
    Eigen::Isometry3d camera_to_target;
    if (mode == CalibrationMode::EYE_IN_HAND)
      camera_to_target = (sample.base_to_flange * camera_mount_to_camera).inverse() * target_mount_to_target;
    else
      camera_to_target = camera_mount_to_camera.inverse() * sample.base_to_flange * target_mount_to_target;
    std::vector<Eigen::Vector3d> points;
    for (const auto& correspondence : sample.correspondences)
      points.push_back(correspondence.in_target);
    const auto pixels = project(points, camera_to_target, camera);
    for (std::size_t i = 0; i < pixels.size(); ++i)
    {
      const double du = pixels[i].x - sample.correspondences[i].in_image.x();
      const double dv = pixels[i].y - sample.correspondences[i].in_image.y();
      squared += du * du + dv * dv;
      ++count;
    }
  }
  return count == 0 ? std::numeric_limits<double>::infinity() : std::sqrt(squared / count);
}
}  // namespace auto_handeye_calibration
