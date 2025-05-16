#include <cassert>
#include <mutex>

#include <franka/control_tools.h>
#include <franka/rate_limiting.h>
#include <research_interface/robot/rbk_types.h>
#include <rclcpp/logging.hpp>

#include "realtime_tools/realtime_helpers.hpp"

#include "franka_hardware/robot_communication_thread.hpp"

namespace franka_hardware {

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

void RobotCommunicationThread::write() {
  // state mutex is needed for reading the current command mode
  std::lock_guard<std::mutex> command_lock(command_mutex_), state_lock(robot_state_mutex_);

  if (hasInfinite(async_hw_position_commands_) || hasInfinite(async_hw_effort_commands_) ||
      hasInfinite(async_hw_velocity_commands_) || hasInfinite(async_hw_cartesian_velocities_) ||
      hasInfinite(async_hw_elbow_command_) || hasInfinite(async_hw_cartesian_pose_)) {
    return;
  }

  if (current_robot_command_mode_ == RobotCommandMode::EFFORT) {
    robot_->writeOnce(async_hw_effort_commands_);
  } else if (current_robot_command_mode_ == RobotCommandMode::JOINT_VELOCITY) {
    robot_->writeOnce(async_hw_velocity_commands_);
  } else if (current_robot_command_mode_ == RobotCommandMode::CARTESIAN_VELOCITY) {
    robot_->writeOnce(async_hw_cartesian_velocities_);
  }

  // For position-based command interfaces, we need to wait until the first read pass after the
  // robot controller is activated to write the position command to the robot.
  // The following condition checks that the external control loop has read an updated robot state
  if (is_internal_read_required_ || is_external_read_required_) {
    return;
  }

  if (current_robot_command_mode_ == RobotCommandMode::JOINT_POSITION) {
    // TODO: Implement the control strategy for joint and cartesian position commands
    // std::array<double, N_JOINTS> joint_position_command_;
    // for (size_t i = 0; i < N_JOINTS; ++i) {
    //   joint_position_command_[i] = current_robot_state_.q_d[i] + async_hw_velocity_commands_[i] * 0.001 + 
    // }
    robot_->writeOnce(async_hw_position_commands_);
  } else if (current_robot_command_mode_ == RobotCommandMode::CARTESIAN_POSE) {
    robot_->writeOnce(async_hw_cartesian_pose_);
  } else if (current_robot_command_mode_ == RobotCommandMode::CARTESIAN_VELOCITY_WITH_ELBOW) {
    robot_->writeOnce(async_hw_cartesian_velocities_, async_hw_elbow_command_);
  } else if (current_robot_command_mode_ == RobotCommandMode::CARTESIAN_POSE_WITH_ELBOW) {
    robot_->writeOnce(async_hw_cartesian_pose_, async_hw_elbow_command_);
  }
}

void RobotCommunicationThread::run() {
  realtime_tools::configure_sched_fifo(70);

  using namespace std::chrono_literals;
  // auto const period = std::chrono::nanoseconds(1'000'000'000 / 1000);
  auto const period = std::chrono::nanoseconds(200'000);

  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point old_now = now;
  double measured_period = std::chrono::duration<double>(now - old_now).count();
  bool should_ignore_packet = false;
  // std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
  // next_iteration_time{now};
  int i = 0;
  while (true) {
    if (is_enabled_) {
      read();

      should_ignore_packet = false;
      now = std::chrono::steady_clock::now();
      measured_period = std::chrono::duration<double>(now - old_now).count();
      old_now = now;
      if (fabs(measured_period - 1e-3) > communication_period_error_tolerance_) {
        RCLCPP_WARN(logger_, "Anomalous communication period detected: %.6f s. Ignoring packet.",
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

void RobotCommunicationThread::enable() {
  is_enabled_ = true;
}

void RobotCommunicationThread::disable() {
  is_enabled_ = false;
  request_command_mode_switch(RobotCommandMode::IDLE);
  robot_->stopRobot();
}

void RobotCommunicationThread::get_current_robot_state(
    franka::RobotState& robot_state,
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
