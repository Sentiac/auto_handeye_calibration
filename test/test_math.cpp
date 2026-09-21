#include "auto_handeye_calibration/math.hpp"

#include <gtest/gtest.h>

using namespace auto_handeye_calibration;

TEST(TransformConvention, CompositionAndInverse)
{
  Eigen::Isometry3d base_to_end = Eigen::Translation3d(1.0, 2.0, 3.0) *
                                  Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ());
  Eigen::Isometry3d end_to_camera = Eigen::Translation3d(0.1, -0.2, 0.3) *
                                    Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitX());
  const Eigen::Vector3d point_camera(0.2, 0.3, 1.0);
  const Eigen::Vector3d point_base = base_to_end * end_to_camera * point_camera;
  EXPECT_TRUE(point_camera.isApprox((base_to_end * end_to_camera).inverse() * point_base, 1e-12));
}

TEST(OpticalLookAt, PositiveZFacesTarget)
{
  const Eigen::Vector3d camera(0.3, -0.2, 0.8);
  const Eigen::Vector3d target(0.1, 0.2, 0.4);
  const auto transform = lookAtOptical(camera, target, Eigen::Vector3d::UnitZ());
  EXPECT_GT(transform.linear().col(2).dot((target - camera).normalized()), 1.0 - 1e-12);
  EXPECT_NEAR(transform.linear().determinant(), 1.0, 1e-12);
}

TEST(Information, DuplicateViewHasPositiveButDiminishingGain)
{
  Eigen::MatrixXd j = Eigen::MatrixXd::Random(50, 12);
  Eigen::MatrixXd h = j.transpose() * j + 1e-6 * Eigen::MatrixXd::Identity(12, 12);
  const double first = logDetPositive(h + j.transpose() * j) - logDetPositive(h);
  const double second = logDetPositive(h + 2.0 * j.transpose() * j) -
                        logDetPositive(h + j.transpose() * j);
  EXPECT_GT(first, 0.0);
  EXPECT_GT(second, 0.0);
  EXPECT_LT(second, first);
}
