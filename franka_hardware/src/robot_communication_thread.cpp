#include <cassert>
#include <mutex>

#include <franka/control_tools.h>
#include <franka/rate_limiting.h>
#include <research_interface/robot/rbk_types.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <rclcpp/logging.hpp>
#include "franka_hardware/utils.hpp"

#include "realtime_tools/realtime_helpers.hpp"

#include "franka_hardware/robot_communication_thread.hpp"

namespace franka_hardware {

RobotCommunicationThread::RobotCommunicationThread(std::shared_ptr<Robot> robot, double lambda)
    : std::thread(&RobotCommunicationThread::run, this),
      robot_(robot),
      logger_(rclcpp::get_logger("RobotCommunicationThread")),
      lambda_(lambda) {
  iir_filters_.reserve(N_JOINTS);
  for (int i = 0; i < N_JOINTS; ++i) {
    iir_filters_.emplace_back(
        std::array<double, 8>{1. / 8, 1. / 8, 1. / 8, 1. / 8, 1. / 8, 1. / 8, 1. / 8, 1. / 8},
        std::array<double, 0>{});
  }
}

void RobotCommunicationThread::request_command_mode_switch(RobotCommandMode robot_command_mode) {
  std::lock_guard<std::mutex> lock(robot_state_mutex_);
  next_robot_command_mode_ = robot_command_mode;
}

void RobotCommunicationThread::perform_command_mode_switch() {
  std::lock_guard<std::mutex> lock(robot_state_mutex_);
  if (current_robot_command_mode_ == next_robot_command_mode_) {
    return;
  }
  current_robot_command_mode_ = next_robot_command_mode_;

  // Stop the robot before switching command modes
  robot_->stopRobot();

  // Handle the command mode switch
  if (current_robot_command_mode_ == RobotCommandMode::IDLE) {
    return;
  } else if (current_robot_command_mode_ == RobotCommandMode::JOINT_POSITION) {
    robot_->initializeJointPositionInterface();
  } else if (current_robot_command_mode_ == RobotCommandMode::JOINT_VELOCITY) {
    async_hw_velocity_commands_.fill(0.0);
    filtered_velocity_commands_.fill(0.0);
    robot_->initializeJointVelocityInterface();
  } else if (current_robot_command_mode_ == RobotCommandMode::EFFORT) {
    async_hw_effort_commands_.fill(0.0);
    robot_->initializeTorqueInterface();
  } else if (current_robot_command_mode_ == RobotCommandMode::CARTESIAN_POSE ||
             current_robot_command_mode_ == RobotCommandMode::CARTESIAN_POSE_WITH_ELBOW) {
    robot_->initializeCartesianPoseInterface();
  } else if (current_robot_command_mode_ == RobotCommandMode::CARTESIAN_VELOCITY ||
             current_robot_command_mode_ == RobotCommandMode::CARTESIAN_VELOCITY_WITH_ELBOW) {
    async_hw_cartesian_velocities_.fill(0.0);
    robot_->initializeCartesianVelocityInterface();
  }

  // Require a state read before commanding the robot with position-based command interfaces
  if (current_robot_command_mode_ == RobotCommandMode::JOINT_POSITION ||
      current_robot_command_mode_ == RobotCommandMode::CARTESIAN_POSE ||
      current_robot_command_mode_ == RobotCommandMode::CARTESIAN_POSE_WITH_ELBOW ||
      current_robot_command_mode_ == RobotCommandMode::CARTESIAN_VELOCITY_WITH_ELBOW) {
    is_internal_read_required_ = true;
    is_external_read_required_ = true;
  }

  // TODO: check if the elbow command is activated without cartesian command interface
}

void RobotCommunicationThread::read() {
  // Read the robot state without blocking the ROS 2 control loop
  temp_franka_robot_state_ = robot_->readOnce();

  {
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    current_robot_state_ = temp_franka_robot_state_;
    is_internal_read_required_ = false;
  }
}

void RobotCommunicationThread::filter_commands(std::chrono::steady_clock::time_point now) {
  if (current_robot_command_mode_ != RobotCommandMode::JOINT_VELOCITY) {
    return;  // Filtering is only applied for joint velocity commands
  }

  // Ensure the command mutex is locked to prevent concurrent access
  std::lock_guard<std::mutex> lock(command_mutex_);

  // Get the current time in seconds
  acg_signal_processing::Timestamp current_time =
      std::chrono::duration<double>(now.time_since_epoch()).count();

  // Update IIR filters for each joint velocity command
  for (size_t i = 0; i < N_JOINTS; ++i) {
    acg_signal_processing::StampedDataPoint<1> sample;
    sample.timestamp = current_time;
    sample.data[0] = async_hw_velocity_commands_[i];
    iir_filters_[i].add_sample(sample);
  }

  // Apply the IIR filter to each joint velocity command
  for (size_t i = 0; i < N_JOINTS; ++i) {
    if (iir_filters_[i].is_ready()) {
      acg_signal_processing::DataPoint<1> filtered_sample = iir_filters_[i].sample(current_time);
      filtered_velocity_commands_[i] = filtered_sample[0];
    } else {
      iir_filters_[i].set_filter_state(async_hw_velocity_commands_[i]);
    }
  }
}

void RobotCommunicationThread::write() {
  // state mutex is needed for reading the current command mode
  std::lock_guard<std::mutex> command_lock(command_mutex_), state_lock(robot_state_mutex_);

  if (current_robot_command_mode_ == RobotCommandMode::EFFORT &&
      !hasInfinite(async_hw_effort_commands_)) {
    robot_->writeOnce(async_hw_effort_commands_);
  } else if (current_robot_command_mode_ == RobotCommandMode::JOINT_VELOCITY &&
             !hasInfinite(async_hw_velocity_commands_)) {
    if (should_filter_ && !hasInfinite(filtered_velocity_commands_)) {
      robot_->writeOnce(filtered_velocity_commands_);
    } else if (!hasInfinite(async_hw_velocity_commands_)) {
      robot_->writeOnce(async_hw_velocity_commands_);
    }
  } else if (current_robot_command_mode_ == RobotCommandMode::CARTESIAN_VELOCITY &&
             !hasInfinite(async_hw_cartesian_velocities_)) {
    robot_->writeOnce(async_hw_cartesian_velocities_);
  }

  // For position-based command interfaces, we need to wait until the first read pass after the
  // robot controller is activated to write the position command to the robot.
  // The following condition checks that the external control loop has read an updated robot state
  if (is_internal_read_required_ || is_external_read_required_) {
    return;
  }

  const bool should_write_joint_position_commands =
      current_robot_command_mode_ == RobotCommandMode::JOINT_POSITION &&
      !hasInfinite(async_hw_position_commands_) && !hasInfinite(async_hw_velocity_commands_);

  const bool should_write_cartesian_pose_commands =
      !hasInfinite(async_hw_cartesian_pose_) &&
      (current_robot_command_mode_ == RobotCommandMode::CARTESIAN_POSE ||
       (current_robot_command_mode_ == RobotCommandMode::CARTESIAN_POSE_WITH_ELBOW &&
        !hasInfinite(async_hw_elbow_command_)));

  const bool should_write_cartesian_velocity_elbow_commands =
      current_robot_command_mode_ == RobotCommandMode::CARTESIAN_VELOCITY_WITH_ELBOW &&
      !hasInfinite(async_hw_cartesian_velocities_) && !hasInfinite(async_hw_elbow_command_);

  if (should_write_joint_position_commands) {
    // TODO: Implement the control strategy for joint and cartesian position commands
    std::array<double, N_JOINTS> joint_position_command;
    for (size_t i = 0; i < N_JOINTS; ++i) {
      joint_position_command[i] =
          current_robot_state_.q_d[i] + async_hw_velocity_commands_[i] * DT +
          (async_hw_position_commands_[i] - current_robot_state_.q_d[i]) * lambda_;
    }
    robot_->writeOnce(joint_position_command);
  } else if (should_write_cartesian_pose_commands) {
    const Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> current_cartesian_pose(
        current_robot_state_.O_T_EE_d.data());
    const Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> desired_cartesian_pose(
        async_hw_cartesian_pose_.data());
    const Eigen::Map<const Eigen::Matrix<double, 6, 1>> desired_cartesian_twist(
        async_hw_cartesian_velocities_.data());

    Eigen::Matrix<double, 4, 4, Eigen::ColMajor> cartesian_pose_reference = current_cartesian_pose;

    // Apply the desired twist to the reference pose
    Eigen::Matrix<double, 4, 4, Eigen::ColMajor> delta_pose;
    calculate_delta_pose(desired_cartesian_twist, DT, delta_pose);
    cartesian_pose_reference *= delta_pose;

    // Calculate the twist between the desired and current poses
    Eigen::Matrix<double, 6, 1> cartesian_twist;
    compute_twist(current_cartesian_pose, cartesian_pose_reference, cartesian_twist, 1.0);

    // Apply the twist to the reference pose
    calculate_delta_pose(cartesian_twist, 0.003, delta_pose);
    cartesian_pose_reference *= delta_pose;

    // Convert cartesian pose reference to std::array format
    std::array<double, DIM_CARTESIAN_POSE> cartesian_pose_command;
    for (size_t i = 0; i < 16; ++i) {
      cartesian_pose_command[i] = cartesian_pose_reference.data()[i];
    }

    if (current_robot_command_mode_ == RobotCommandMode::CARTESIAN_POSE_WITH_ELBOW) {
      robot_->writeOnce(cartesian_pose_command, async_hw_elbow_command_);
    } else {
      robot_->writeOnce(cartesian_pose_command);
    }
  } else if (should_write_cartesian_velocity_elbow_commands) {
    robot_->writeOnce(async_hw_cartesian_velocities_, async_hw_elbow_command_);
  }
}

void RobotCommunicationThread::run() {
  realtime_tools::configure_sched_fifo(99);

  using namespace std::chrono_literals;
  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point old_now = now;
  double measured_period = std::chrono::duration<double>(now - old_now).count();
  bool should_ignore_packet = false;
  int i = 0;
  while (true) {
    if (is_enabled_) {
      if (should_filter_) {
        // Apply filters to the joint commands
        filter_commands(now);
      }

      read();

      should_ignore_packet = false;
      now = std::chrono::steady_clock::now();
      measured_period = std::chrono::duration<double>(now - old_now).count();
      old_now = now;
      if (fabs(measured_period - 1e-3) > communication_period_error_tolerance_) {
        RCLCPP_DEBUG(logger_, "Anomalous communication period detected: %.6f s. Ignoring packet.",
                     measured_period);
        should_ignore_packet = true;
      }

      perform_command_mode_switch();

      if (!should_ignore_packet) {
        write();
      }
      i++;
    }
  }
}

// ------ PUBLIC MEMBER FUNCTIONS ------

void RobotCommunicationThread::set_filter_commands(bool should_filter) {
  should_filter_ = should_filter;
}

void RobotCommunicationThread::enable() {
  is_enabled_ = true;
}

void RobotCommunicationThread::disable() {
  is_enabled_ = false;
  request_command_mode_switch(RobotCommandMode::IDLE);
  robot_->stopRobot();
}

void RobotCommunicationThread::get_current_robot_state(franka::RobotState& robot_state,
                                                       RobotCommandMode& robot_command_mode) {
  std::lock_guard<std::mutex> lock(robot_state_mutex_);
  robot_state = current_robot_state_;
  robot_command_mode = current_robot_command_mode_;
  is_external_read_required_ = false;
}

void RobotCommunicationThread::write_commands(
    const std::array<double, N_JOINTS>& joint_effort_commands,
    const std::array<double, N_JOINTS>& joint_position_commands,
    const std::array<double, N_JOINTS>& joint_velocity_commands,
    const std::array<double, DIM_CARTESIAN_POSE>& cartesian_pose_commands,
    const std::array<double, DIM_CARTESIAN_VELOCITIES>& cartesian_velocity_commands,
    const std::array<double, DIM_ELBOW_COMMANDS>& elbow_command) {
  std::lock_guard<std::mutex> lock(command_mutex_);
  async_hw_effort_commands_ = joint_effort_commands;
  async_hw_position_commands_ = joint_position_commands;
  async_hw_velocity_commands_ = joint_velocity_commands;
  async_hw_cartesian_pose_ = cartesian_pose_commands;
  async_hw_cartesian_velocities_ = cartesian_velocity_commands;
  async_hw_elbow_command_ = elbow_command;
}

franka_hardware::Model* RobotCommunicationThread::get_model() {
  std::lock_guard<std::mutex> lock(robot_state_mutex_);
  return robot_->getModel();
}
}  // namespace franka_hardware
