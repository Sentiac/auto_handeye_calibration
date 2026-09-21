#include "auto_handeye_calibration/calibration.hpp"
#include "auto_handeye_calibration/math.hpp"

#include <gtest/gtest.h>
#include <opencv2/aruco.hpp>

using namespace auto_handeye_calibration;

TEST(SyntheticCalibration, IndustrialRefinementRecoversEyeInHand)
{
  CameraModel camera{620.0, 615.0, 320.0, 240.0, 640, 480, {0, 0, 0, 0, 0}};
  CalibrationEngine engine(5, 7, 0.038268, 0.023960, cv::aruco::DICT_5X5_250);
  const Eigen::Isometry3d flange_to_camera = Eigen::Translation3d(0.03, -0.02, 0.12) *
      Eigen::AngleAxisd(0.08, Eigen::Vector3d::UnitX());
  const Eigen::Isometry3d base_to_target = Eigen::Translation3d(0.45, 0.05, 0.25) *
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ());
  std::vector<Sample> samples;
  const auto points = engine.allTargetPoints();
  for (int i = 0; i < 10; ++i)
  {
    Sample sample;
    const Eigen::Vector3d camera_position(0.25 + 0.025 * (i % 5), -0.15 + 0.04 * (i % 3), 0.65);
    const auto base_to_camera = lookAtOptical(camera_position, base_to_target.translation(), Eigen::Vector3d::UnitY(),
                                               -0.15 + 0.03 * i);
    sample.base_to_flange = base_to_camera * flange_to_camera.inverse();
    const auto pixels = project(points, base_to_camera.inverse() * base_to_target, camera);
    for (std::size_t p = 0; p < points.size(); ++p)
      sample.correspondences.emplace_back(Eigen::Vector2d(pixels[p].x, pixels[p].y), points[p]);
    samples.push_back(sample);
  }
  CalibrationResult seed;
  seed.valid = true;
  seed.initializer = "synthetic";
  seed.camera_mount_to_camera = flange_to_camera *
      (Eigen::Translation3d(0.005, -0.004, 0.003) * Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitY()));
  seed.target_mount_to_target = base_to_target *
      (Eigen::Translation3d(-0.003, 0.002, 0.004) * Eigen::AngleAxisd(-0.02, Eigen::Vector3d::UnitX()));
  const auto result = engine.refine(samples, camera, CalibrationMode::EYE_IN_HAND, seed);
  ASSERT_TRUE(result.valid);
  EXPECT_LT((result.camera_mount_to_camera.translation() - flange_to_camera.translation()).norm(), 1e-4);
  EXPECT_LT(rotationDistance(result.camera_mount_to_camera.linear(), flange_to_camera.linear()), 1e-4);
  EXPECT_LT(result.rms_px, 1e-4);
}
