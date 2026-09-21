#include "auto_handeye_calibration/calibration.hpp"
#include "auto_handeye_calibration/math.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace auto_handeye_calibration
{
namespace
{
geometry_msgs::msg::Pose toPose(const Eigen::Isometry3d& transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond q(transform.linear());
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

std::string transformYaml(const Eigen::Isometry3d& transform)
{
  const Eigen::Quaterniond q(transform.linear());
  std::ostringstream stream;
  stream << std::setprecision(12)
         << "  translation: [" << transform.translation().x() << ", "
         << transform.translation().y() << ", " << transform.translation().z() << "]\n"
         << "  quaternion_xyzw: [" << q.x() << ", " << q.y() << ", " << q.z()
         << ", " << q.w() << "]\n";
  return stream.str();
}

int dictionaryFromName(const std::string& name)
{
  static const std::map<std::string, int> dictionaries = {
    {"DICT_4X4_50", cv::aruco::DICT_4X4_50}, {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
    {"DICT_5X5_250", cv::aruco::DICT_5X5_250}, {"DICT_6X6_250", cv::aruco::DICT_6X6_250}};
  const auto found = dictionaries.find(name);
  if (found == dictionaries.end())
    throw std::runtime_error("Unsupported ArUco dictionary: " + name);
  return found->second;
}
}  // namespace

class ActiveHandeyeNode : public rclcpp::Node
{
public:
  ActiveHandeyeNode()
    : Node("auto_handeye_calibration"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = callback_group_;
    base_frame_ = declare_parameter("frames.base", "base_link");
    flange_frame_ = declare_parameter("frames.flange", "tool0");
    camera_frame_ = declare_parameter("frames.camera_optical", "camera_color_optical_frame");
    planning_group_ = declare_parameter("moveit.planning_group", "manipulator");
    image_topic_ = declare_parameter("topics.image", "/camera/color/image_raw");
    camera_info_topic_ = declare_parameter("topics.camera_info", "/camera/color/camera_info");
    joint_topic_ = declare_parameter("topics.joint_states", "/joint_states");
    const auto mode = declare_parameter("calibration.mode", "eye_in_hand");
    mode_ = mode == "eye_to_hand" ? CalibrationMode::EYE_TO_HAND : CalibrationMode::EYE_IN_HAND;
    const int squares_x = declare_parameter("target.squares_x", 5);
    const int squares_y = declare_parameter("target.squares_y", 7);
    const double square_length = declare_parameter("target.square_length_m", 0.038268);
    const double marker_length = declare_parameter("target.marker_length_m", 0.023960);
    const auto dictionary = declare_parameter("target.dictionary", "DICT_5X5_250");
    expected_corners_ = static_cast<std::size_t>((squares_x - 1) * (squares_y - 1));
    minimum_corner_fraction_ = declare_parameter("capture.minimum_corner_fraction", 0.60);
    initial_minimum_corner_fraction_ =
      declare_parameter("capture.initial_minimum_corner_fraction", 1.0);
    minimum_coverage_ = declare_parameter("capture.minimum_coverage_ratio", 0.08);
    border_margin_ = declare_parameter("capture.border_margin_px", 20.0);
    maximum_pnp_rms_ = declare_parameter("capture.maximum_pnp_rms_px", 1.0);
    maximum_sync_error_ = declare_parameter("capture.maximum_sync_error_s", 0.020);
    settle_time_ = declare_parameter("capture.settle_time_s", 0.75);
    bootstrap_samples_ = declare_parameter("bootstrap.accepted_sample_target", 8);
    minimum_samples_ = declare_parameter("stopping.minimum_samples", 10);
    maximum_samples_ = declare_parameter("stopping.maximum_samples", 25);
    automatic_execution_ = declare_parameter("safety.automatic_execution_enabled", false);
    operator_arm_required_ = declare_parameter("safety.operator_arm_required", true);
    planning_time_ = declare_parameter("safety.planning_time_s", 5.0);
    velocity_scaling_ = declare_parameter("safety.velocity_scaling", 0.15);
    acceleration_scaling_ = declare_parameter("safety.acceleration_scaling", 0.10);
    auto_start_ = declare_parameter("session.auto_start", false);
    output_root_ = declare_parameter("session.output_directory", "/tmp/active_handeye");
    engine_ = std::make_unique<CalibrationEngine>(squares_x, squares_y, square_length,
                                                  marker_length, dictionaryFromName(dictionary));

    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        std::scoped_lock lock(data_mutex_);
        latest_image_ = std::move(message);
      }, subscription_options);
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        std::scoped_lock lock(data_mutex_);
        camera_.fx = message->k[0]; camera_.fy = message->k[4];
        camera_.cx = message->k[2]; camera_.cy = message->k[5];
        camera_.width = static_cast<int>(message->width);
        camera_.height = static_cast<int>(message->height);
        camera_.distortion = message->d;
        have_camera_info_ = camera_.fx > 0.0 && camera_.fy > 0.0;
      }, subscription_options);
    joint_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        std::scoped_lock lock(data_mutex_);
        latest_joints_ = std::move(message);
      }, subscription_options);
    status_publisher_ = create_publisher<std_msgs::msg::String>("~/status", 10);
    start_service_ = create_service<std_srvs::srv::Trigger>(
      "~/start", [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        armed_ = true;
        running_ = true;
        response->success = true;
        response->message = "Calibration armed and started";
      }, rclcpp::ServicesQoS(), callback_group_);
    pause_service_ = create_service<std_srvs::srv::Trigger>(
      "~/pause", [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        running_ = false;
        response->success = true;
        response->message = "Calibration paused";
      }, rclcpp::ServicesQoS(), callback_group_);
    abort_service_ = create_service<std_srvs::srv::Trigger>(
      "~/abort", [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        running_ = false;
        aborted_ = true;
        response->success = true;
        response->message = "Calibration aborted; no further trajectories will execute";
      }, rclcpp::ServicesQoS(), callback_group_);
  }

  void initialize()
  {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group_);
    move_group_->setPoseReferenceFrame(base_frame_);
    move_group_->setEndEffectorLink(flange_frame_);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    timer_ = create_wall_timer(250ms, std::bind(&ActiveHandeyeNode::tick, this), callback_group_);
    if (auto_start_ && automatic_execution_ && !operator_arm_required_)
    {
      armed_ = true;
      running_ = true;
    }
    publishStatus("IDLE", "waiting for synchronized camera data and a fully visible board");
  }

private:
  void publishStatus(const std::string& state, const std::string& detail)
  {
    std_msgs::msg::String message;
    message.data = state + ": " + detail;
    status_publisher_->publish(message);
    if (state != state_ || detail != detail_)
      RCLCPP_INFO(get_logger(), "%s", message.data.c_str());
    state_ = state;
    detail_ = detail;
  }

  void tick()
  {
    if (!running_ || aborted_ || busy_)
      return;
    if (!have_camera_info_ || !latest_image_)
    {
      publishStatus("VALIDATE_CONFIGURATION", "camera image or CameraInfo unavailable");
      return;
    }
    if (!armed_ || (!automatic_execution_ && samples_.size() > 0))
    {
      publishStatus("PAUSED", "automatic execution is not armed");
      return;
    }
    busy_ = true;
    try
    {
      if (samples_.empty())
      {
        publishStatus("ACQUIRE_START_SAMPLE", "validating initial board observation");
        if (captureSample("start"))
        {
          Eigen::Isometry3d camera_to_target;
          double rms = 0.0;
          engine_->estimateCameraToTarget(samples_.back().correspondences, camera_, camera_to_target, rms);
          if (mode_ == CalibrationMode::EYE_IN_HAND)
          {
            nominal_camera_mount_to_camera_ = lookup(flange_frame_, camera_frame_, latest_stamp_);
            estimated_target_mount_to_target_ =
              samples_.back().base_to_flange * nominal_camera_mount_to_camera_ * camera_to_target;
          }
          else
          {
            nominal_camera_mount_to_camera_ = lookup(base_frame_, camera_frame_, latest_stamp_);
            estimated_target_mount_to_target_ = samples_.back().base_to_flange.inverse() *
              nominal_camera_mount_to_camera_ * camera_to_target;
            initial_camera_to_target_ = camera_to_target;
          }
          createSession();
          persistInitialSample();
          generateCandidates();
        }
      }
      else if (samples_.size() < static_cast<std::size_t>(minimum_samples_))
      {
        if (next_candidate_ >= candidates_.size())
          throw std::runtime_error("No collision-free visible candidate remains");
        const auto candidate = candidates_[next_candidate_++];
        publishStatus(samples_.size() < static_cast<std::size_t>(bootstrap_samples_) ?
                        "COLLECT_BOOTSTRAP_SAMPLES" : "SELECT_AND_EXECUTE_NBV", candidate.id);
        if (planAndExecute(candidate))
        {
          rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(settle_time_)));
          captureSample(candidate.id);
          if (samples_.size() >= 5)
            calibrate();
        }
      }
      if (samples_.size() >= static_cast<std::size_t>(minimum_samples_))
      {
        calibrate();
        saveResults();
        running_ = false;
        publishStatus("COMPLETE", "calibration and reproducible session report saved to " + session_directory_);
      }
    }
    catch (const std::exception& exception)
    {
      running_ = false;
      publishStatus("FAILED_SAFE", exception.what());
      RCLCPP_ERROR(get_logger(), "Calibration stopped safely: %s", exception.what());
    }
    busy_ = false;
  }

  Eigen::Isometry3d lookup(const std::string& parent, const std::string& child,
                           const rclcpp::Time& stamp)
  {
    const auto transform = tf_buffer_.lookupTransform(parent, child, stamp, 500ms);
    return tf2::transformToEigen(transform);
  }

  bool captureSample(const std::string& candidate_id)
  {
    sensor_msgs::msg::Image::ConstSharedPtr image_message;
    sensor_msgs::msg::JointState::ConstSharedPtr joint_message;
    {
      std::scoped_lock lock(data_mutex_);
      image_message = latest_image_;
      joint_message = latest_joints_;
    }
    if (!image_message)
      return false;
    latest_stamp_ = rclcpp::Time(image_message->header.stamp, get_clock()->get_clock_type());
    const Eigen::Isometry3d base_to_flange = lookup(base_frame_, flange_frame_, latest_stamp_);
    const cv::Mat image = cv_bridge::toCvShare(image_message, "bgr8")->image;
    cv::Mat annotated;
    industrial_calibration::Correspondence2D3D::Set correspondences;
    try { correspondences = engine_->detect(image, &annotated); }
    catch (const std::exception& exception)
    {
      publishStatus("CAPTURE_REJECTED", exception.what());
      return false;
    }
    DetectionMetrics metrics;
    metrics.corner_fraction = static_cast<double>(correspondences.size()) /
                              static_cast<double>(expected_corners_);
    std::vector<cv::Point2f> pixels;
    double minimum_border = std::numeric_limits<double>::infinity();
    for (const auto& correspondence : correspondences)
    {
      const float x = static_cast<float>(correspondence.in_image.x());
      const float y = static_cast<float>(correspondence.in_image.y());
      pixels.emplace_back(x, y);
      minimum_border = std::min({minimum_border, static_cast<double>(x), static_cast<double>(y),
        camera_.width - static_cast<double>(x), camera_.height - static_cast<double>(y)});
    }
    metrics.border_margin_px = minimum_border;
    if (pixels.size() >= 3)
    {
      std::vector<cv::Point2f> hull;
      cv::convexHull(pixels, hull);
      metrics.coverage_ratio = cv::contourArea(hull) /
                               static_cast<double>(camera_.width * camera_.height);
    }
    cv::Mat gray, laplacian;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::Laplacian(gray, laplacian, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(laplacian, mean, stddev);
    metrics.blur_score = stddev[0] * stddev[0];
    Eigen::Isometry3d camera_to_target;
    if (!engine_->estimateCameraToTarget(correspondences, camera_, camera_to_target,
                                         metrics.reprojection_rms_px))
      return false;
    const double required_corner_fraction = candidate_id == "start" ?
      initial_minimum_corner_fraction_ : minimum_corner_fraction_;
    if (metrics.corner_fraction < required_corner_fraction ||
        metrics.coverage_ratio < minimum_coverage_ ||
        metrics.border_margin_px < border_margin_ ||
        metrics.reprojection_rms_px > maximum_pnp_rms_)
    {
      std::ostringstream reason;
      reason << "quality gate failed: corners=" << metrics.corner_fraction
             << " coverage=" << metrics.coverage_ratio << " border=" << metrics.border_margin_px
             << " pnp_rms=" << metrics.reprojection_rms_px;
      publishStatus("CAPTURE_REJECTED", reason.str());
      return false;
    }
    Sample sample;
    sample.id = samples_.size();
    sample.image_stamp = latest_stamp_;
    sample.base_to_flange = base_to_flange;
    sample.correspondences = std::move(correspondences);
    sample.metrics = metrics;
    if (joint_message)
      sample.joints = joint_message->position;
    if (!session_directory_.empty())
    {
      std::ostringstream filename;
      filename << session_directory_ << "/samples/sample_" << std::setw(3)
               << std::setfill('0') << sample.id << ".png";
      sample.image_path = filename.str();
      cv::imwrite(sample.image_path, image);
    }
    samples_.push_back(std::move(sample));
    saveSample(samples_.back(), candidate_id);
    return true;
  }

  void generateCandidates()
  {
    candidates_.clear();
    const auto points = engine_->allTargetPoints();
    Eigen::Vector3d board_center = Eigen::Vector3d::Zero();
    for (const auto& point : points) board_center += point;
    board_center /= static_cast<double>(points.size());
    const Eigen::Vector3d center_base = estimated_target_mount_to_target_ * board_center;
    const Eigen::Isometry3d initial_camera = samples_.front().base_to_flange * nominal_camera_mount_to_camera_;
    const Eigen::Vector3d initial_in_target = mode_ == CalibrationMode::EYE_IN_HAND ?
      estimated_target_mount_to_target_.inverse() * initial_camera.translation() :
      initial_camera_to_target_.inverse().translation();
    const double side = initial_in_target.z() >= 0.0 ? 1.0 : -1.0;
    const double initial_distance = std::max(0.25, std::abs(initial_in_target.z()));
    const std::vector<double> lateral = {-0.06, -0.03, 0.0, 0.03, 0.06};
    const std::vector<double> vertical = {-0.05, 0.0, 0.05};
    const std::vector<double> distance_scale = {0.85, 1.0, 1.15};
    int index = 0;
    for (double scale : distance_scale)
      for (double x : lateral)
        for (double y : vertical)
        {
          if (std::abs(x) < 1e-9 && std::abs(y) < 1e-9 && std::abs(scale - 1.0) < 1e-9)
            continue;
          Candidate candidate;
          candidate.id = "view_" + std::to_string(index++);
          Eigen::Vector3d camera_target(x + board_center.x(), y + board_center.y(),
                                        side * initial_distance * scale);
          if (mode_ == CalibrationMode::EYE_IN_HAND)
          {
            const Eigen::Vector3d camera_base = estimated_target_mount_to_target_ * camera_target;
            candidate.base_to_camera = lookAtOptical(camera_base, center_base, Eigen::Vector3d::UnitZ(),
                                                      (index % 3 - 1) * 0.20);
            candidate.base_to_flange = candidate.base_to_camera * nominal_camera_mount_to_camera_.inverse();
          }
          else
          {
            Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
            offset.translation() = Eigen::Vector3d(x, y, side * initial_distance * (scale - 1.0));
            offset.linear() = (Eigen::AngleAxisd(0.4 * y, Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(-0.4 * x, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd((index % 3 - 1) * 0.20,
                                                 Eigen::Vector3d::UnitZ())).toRotationMatrix();
            const Eigen::Isometry3d desired_camera_to_target = initial_camera_to_target_ * offset;
            candidate.base_to_camera = nominal_camera_mount_to_camera_;
            candidate.base_to_flange = nominal_camera_mount_to_camera_ * desired_camera_to_target *
                                       estimated_target_mount_to_target_.inverse();
          }
          candidate.geometric_score = std::abs(x) + std::abs(y) + std::abs(scale - 1.0);
          candidates_.push_back(candidate);
        }
    std::stable_sort(candidates_.begin(), candidates_.end(), [](const auto& lhs, const auto& rhs) {
      return lhs.geometric_score < rhs.geometric_score;
    });
    next_candidate_ = 0;
    publishStatus("GENERATE_BOOTSTRAP_CANDIDATES", std::to_string(candidates_.size()) + " generic target-relative views generated");
  }

  bool planAndExecute(const Candidate& candidate)
  {
    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(toPose(candidate.base_to_flange), flange_frame_);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(move_group_->plan(plan));
    move_group_->clearPoseTargets();
    if (!planned)
    {
      recordCandidate(candidate, "MoveIt planning/IK/collision validation failed");
      return false;
    }
    recordCandidate(candidate, "accepted and planned");
    if (!automatic_execution_ || !armed_)
      throw std::runtime_error("trajectory was planned but automatic execution is disabled or disarmed");
    rclcpp::Time image_before_motion;
    {
      std::scoped_lock lock(data_mutex_);
      if (latest_image_)
        image_before_motion = rclcpp::Time(latest_image_->header.stamp, get_clock()->get_clock_type());
    }
    const auto executed = move_group_->execute(plan);
    if (!static_cast<bool>(executed))
      throw std::runtime_error("MoveIt trajectory execution failed");
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < deadline)
    {
      {
        std::scoped_lock lock(data_mutex_);
        if (latest_image_ && rclcpp::Time(latest_image_->header.stamp,
              get_clock()->get_clock_type()) > image_before_motion)
          return true;
      }
      rclcpp::sleep_for(20ms);
    }
    throw std::runtime_error("camera did not publish a new frame after trajectory execution");
  }

  void calibrate()
  {
    publishStatus("RUN_FIVE_INITIALIZERS", "evaluating calibration seeds");
    auto initializers = engine_->initializeAll(samples_, camera_, mode_);
    if (mode_ == CalibrationMode::EYE_TO_HAND)
    {
      CalibrationResult nominal;
      nominal.valid = true;
      nominal.initializer = "NOMINAL_TF";
      nominal.camera_mount_to_camera = nominal_camera_mount_to_camera_;
      nominal.target_mount_to_target = estimated_target_mount_to_target_;
      initializers.emplace(nominal.initializer, nominal);
    }
    CalibrationResult best;
    best.rms_px = std::numeric_limits<double>::infinity();
    for (auto& item : initializers)
    {
      if (!item.second.valid) continue;
      try
      {
        const auto refined = engine_->refine(samples_, camera_, mode_, item.second);
        if (refined.valid && refined.rms_px < best.rms_px)
          best = refined;
      }
      catch (const std::exception& exception)
      {
        RCLCPP_WARN(get_logger(), "%s refinement failed: %s", item.first.c_str(), exception.what());
      }
    }
    if (!best.valid)
      throw std::runtime_error("all OpenCV seeds or nonlinear refinements failed");
    result_ = best;
    estimated_target_mount_to_target_ = best.target_mount_to_target;
    if (samples_.size() >= static_cast<std::size_t>(bootstrap_samples_))
      rankRemainingCandidatesByInformation();
    publishStatus("REOPTIMIZE", best.initializer + " RMS=" + std::to_string(best.rms_px) + " px");
  }

  void rankRemainingCandidatesByInformation()
  {
    if (mode_ != CalibrationMode::EYE_IN_HAND || next_candidate_ >= candidates_.size())
      return;
    const auto points = engine_->allTargetPoints();
    Eigen::MatrixXd information = 1e-6 * Eigen::MatrixXd::Identity(12, 12);
    for (const auto& sample : samples_)
    {
      const auto jacobian = numericalProjectionJacobian(
        points, sample.base_to_flange, result_.camera_mount_to_camera,
        result_.target_mount_to_target, camera_);
      information += jacobian.transpose() * jacobian;
    }
    const double baseline = logDetPositive(information);
    for (std::size_t i = next_candidate_; i < candidates_.size(); ++i)
    {
      auto& candidate = candidates_[i];
      const Eigen::Isometry3d camera_to_target =
        candidate.base_to_camera.inverse() * result_.target_mount_to_target;
      bool visible = true;
      for (const auto& point : points)
      {
        const Eigen::Vector3d camera_point = camera_to_target * point;
        if (camera_point.z() <= 0.0) { visible = false; break; }
      }
      if (visible)
      {
        const auto pixels = project(points, camera_to_target, camera_);
        for (const auto& pixel : pixels)
          if (pixel.x < border_margin_ || pixel.y < border_margin_ ||
              pixel.x >= camera_.width - border_margin_ || pixel.y >= camera_.height - border_margin_)
          { visible = false; break; }
      }
      if (!visible)
      {
        candidate.information_gain = -std::numeric_limits<double>::infinity();
        candidate.rejection_reason = "predicted board footprint violates depth/FOV margin";
        continue;
      }
      const auto jacobian = numericalProjectionJacobian(
        points, candidate.base_to_flange, result_.camera_mount_to_camera,
        result_.target_mount_to_target, camera_);
      candidate.information_gain = logDetPositive(information + jacobian.transpose() * jacobian) - baseline;
    }
    std::stable_sort(candidates_.begin() + static_cast<std::ptrdiff_t>(next_candidate_), candidates_.end(),
      [](const auto& lhs, const auto& rhs) { return lhs.information_gain > rhs.information_gain; });
  }

  void createSession()
  {
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    std::ostringstream name;
    name << output_root_ << "/session_" << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S");
    session_directory_ = name.str();
    std::filesystem::create_directories(session_directory_ + "/samples");
    std::ofstream dependencies(session_directory_ + "/dependency_versions.json");
    dependencies << "{\"ros\":\"jazzy\",\"industrial_calibration\":\"1.2.0\","
                    "\"opencv\":\"4.6.0\",\"moveit\":\"2.12.4\"}\n";
    std::ofstream config(session_directory_ + "/config_snapshot.yaml");
    config << "mode: " << (mode_ == CalibrationMode::EYE_IN_HAND ? "eye_in_hand" : "eye_to_hand")
           << "\nframes: {base: " << base_frame_ << ", flange: " << flange_frame_
           << ", camera_optical: " << camera_frame_ << "}\n"
           << "moveit_group: " << planning_group_ << "\nminimum_samples: " << minimum_samples_
           << "\nautomatic_execution_enabled: " << std::boolalpha << automatic_execution_ << "\n";
  }

  void persistInitialSample()
  {
    if (samples_.empty()) return;
    sensor_msgs::msg::Image::ConstSharedPtr image_message;
    {
      std::scoped_lock lock(data_mutex_);
      image_message = latest_image_;
    }
    if (image_message)
    {
      samples_.front().image_path = session_directory_ + "/samples/sample_000.png";
      cv::imwrite(samples_.front().image_path,
                  cv_bridge::toCvShare(image_message, "bgr8")->image);
    }
    saveSample(samples_.front(), "start");
  }

  void saveSample(const Sample& sample, const std::string& candidate_id)
  {
    if (session_directory_.empty()) return;
    std::ostringstream filename;
    filename << session_directory_ << "/samples/sample_" << std::setw(3) << std::setfill('0')
             << sample.id << ".yaml";
    std::ofstream file(filename.str());
    file << "sample_id: " << sample.id << "\nimage_stamp_ns: " << sample.image_stamp.nanoseconds()
         << "\ncandidate_id: " << candidate_id << "\nbase_to_flange:\n"
         << transformYaml(sample.base_to_flange)
         << "metrics:\n  corner_fraction: " << sample.metrics.corner_fraction
         << "\n  coverage_ratio: " << sample.metrics.coverage_ratio
         << "\n  border_margin_px: " << sample.metrics.border_margin_px
         << "\n  blur_score: " << sample.metrics.blur_score
         << "\n  pnp_reprojection_rms_px: " << sample.metrics.reprojection_rms_px << "\n"
         << "corners:\n";
    for (const auto& correspondence : sample.correspondences)
      file << "  - {pixel: [" << correspondence.in_image.x() << ", " << correspondence.in_image.y()
           << "], target: [" << correspondence.in_target.x() << ", " << correspondence.in_target.y()
           << ", " << correspondence.in_target.z() << "]}\n";
  }

  void recordCandidate(const Candidate& candidate, const std::string& result)
  {
    if (session_directory_.empty()) return;
    std::ofstream file(session_directory_ + "/candidates.jsonl", std::ios::app);
    file << "{\"id\":\"" << candidate.id << "\",\"geometric_score\":"
         << candidate.geometric_score << ",\"information_gain\":" << candidate.information_gain
         << ",\"result\":\"" << result << "\"}\n";
  }

  void saveResults()
  {
    std::ofstream calibration(session_directory_ + "/final_calibration.yaml");
    const std::string camera_parent = mode_ == CalibrationMode::EYE_IN_HAND ? flange_frame_ : base_frame_;
    calibration << "mode: " << (mode_ == CalibrationMode::EYE_IN_HAND ? "eye_in_hand" : "eye_to_hand")
                << "\ntransform_convention: T_parent_child\nparent_frame: " << camera_parent
                << "\nchild_frame: " << camera_frame_ << "\nunits: metres_and_radians\nquaternion_order: xyzw\n"
                << "camera_mount_to_camera:\n" << transformYaml(result_.camera_mount_to_camera)
                << "target_mount_to_target:\n" << transformYaml(result_.target_mount_to_target)
                << "initializer: " << result_.initializer << "\nreprojection_rms_px: " << result_.rms_px << "\n";
    std::ofstream report(session_directory_ + "/report.json");
    report << "{\n  \"status\": \"complete\",\n  \"accepted_samples\": " << samples_.size()
           << ",\n  \"initializer\": \"" << result_.initializer << "\",\n  \"training_rms_px\": "
           << result_.rms_px << ",\n  \"collision_checked_by_moveit\": true\n}\n";
  }

  std::string base_frame_, flange_frame_, camera_frame_, planning_group_;
  std::string image_topic_, camera_info_topic_, joint_topic_, output_root_;
  CalibrationMode mode_{CalibrationMode::EYE_IN_HAND};
  std::size_t expected_corners_{0};
  double minimum_corner_fraction_{0.6}, initial_minimum_corner_fraction_{1.0};
  double minimum_coverage_{0.08}, border_margin_{20.0};
  double maximum_pnp_rms_{1.0}, maximum_sync_error_{0.02}, settle_time_{0.75};
  double planning_time_{5.0}, velocity_scaling_{0.15}, acceleration_scaling_{0.1};
  int bootstrap_samples_{8}, minimum_samples_{10}, maximum_samples_{25};
  bool automatic_execution_{false}, operator_arm_required_{true}, auto_start_{false};
  bool armed_{false}, running_{false}, aborted_{false}, busy_{false}, have_camera_info_{false};
  std::string state_, detail_, session_directory_;
  CameraModel camera_;
  rclcpp::Time latest_stamp_;
  std::mutex data_mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  sensor_msgs::msg::JointState::ConstSharedPtr latest_joints_;
  std::vector<Sample> samples_;
  std::vector<Candidate> candidates_;
  std::size_t next_candidate_{0};
  Eigen::Isometry3d nominal_camera_mount_to_camera_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d estimated_target_mount_to_target_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d initial_camera_to_target_{Eigen::Isometry3d::Identity()};
  CalibrationResult result_;
  std::unique_ptr<CalibrationEngine> engine_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_, pause_service_, abort_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};
}  // namespace auto_handeye_calibration

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<auto_handeye_calibration::ActiveHandeyeNode>();
  node->initialize();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
