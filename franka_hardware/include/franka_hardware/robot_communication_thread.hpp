#pragma once

#include <array>
#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <franka/active_control.h>
#include <franka/active_control_base.h>
#include <franka/active_motion_generator.h>
#include <franka/active_torque_control.h>

#include <franka/model.h>
#include <franka/robot.h>
#include <franka_hardware/model.hpp>

#include <franka_msgs/srv/set_cartesian_stiffness.hpp>
#include <franka_msgs/srv/set_force_torque_collision_behavior.hpp>
#include <franka_msgs/srv/set_full_collision_behavior.hpp>
#include <franka_msgs/srv/set_joint_stiffness.hpp>
#include <franka_msgs/srv/set_load.hpp>
#include <franka_msgs/srv/set_stiffness_frame.hpp>
#include <franka_msgs/srv/set_tcp_frame.hpp>

#include <rclcpp/logger.hpp>

#include "franka_hardware/robot.hpp"

namespace franka_hardware {

enum class RobotCommandMode {
  IDLE,
  EFFORT,
  JOINT_POSITION,
  JOINT_VELOCITY,
  CARTESIAN_POSE,
  CARTESIAN_VELOCITY,
  CARTESIAN_POSE_WITH_ELBOW,
  CARTESIAN_VELOCITY_WITH_ELBOW,
};

class RobotCommunicationThread : public std::thread {
 public:
  static constexpr int N_JOINTS = 7;
  static constexpr int DIM_CARTESIAN_POSE = 16;
  static constexpr int DIM_CARTESIAN_VELOCITIES = 6;
  static constexpr int DIM_ELBOW_COMMANDS = 2;

  RobotCommunicationThread(std::shared_ptr<Robot> robot)
      : std::thread(&RobotCommunicationThread::run, this), robot_(robot), logger_(rclcpp::get_logger("RobotCommunicationThread")) {}

  franka_hardware::Model* get_model();

  void get_current_robot_state(franka::RobotState& robot_state,
                               RobotCommandMode& robot_command_mode);
  void write_commands(const std::array<double, N_JOINTS>& joint_effort_commands,
                      const std::array<double, N_JOINTS>& joint_position_commands,
                      const std::array<double, N_JOINTS>& joint_velocity_commands,
                      const std::array<double, DIM_CARTESIAN_POSE>& cartesian_pose_commands,
                      const std::array<double, DIM_CARTESIAN_VELOCITIES>& cartesian_velocity_commands,
                      const std::array<double, DIM_ELBOW_COMMANDS>& elbow_command);
  void request_command_mode_switch(RobotCommandMode robot_command_mode);
  void enable();
  void disable();

 private:
  void run();
  void read();
  void write();
  void perform_command_mode_switch();

  template <typename CommandType>
  bool hasInfinite(const CommandType& commands) {
    return std::any_of(commands.begin(), commands.end(),
                       [](double command) { return !std::isfinite(command); });
  }

  // Torque joint commands for the effort command interface
  std::array<double, N_JOINTS> async_hw_effort_commands_{0, 0, 0, 0, 0, 0, 0};
  // Position joint commands for the position command interface
  std::array<double, N_JOINTS> async_hw_position_commands_{0, 0, 0, 0, 0, 0, 0};
  // Velocity joint commands for the position command interface
  std::array<double, N_JOINTS> async_hw_velocity_commands_{0, 0, 0, 0, 0, 0, 0};
  // Cartesian commands
  std::array<double, DIM_CARTESIAN_VELOCITIES> async_hw_cartesian_velocities_{0, 0, 0, 0, 0, 0};
  std::array<double, DIM_CARTESIAN_POSE> async_hw_cartesian_pose_{1, 0, 0, 0, 0, 1, 0, 0,
                                                                        0, 0, 1, 0, 0, 0, 0, 1};
  std::array<double, DIM_ELBOW_COMMANDS> async_hw_elbow_command_{0, 0};

  // Module enabled
  std::atomic<bool> is_enabled_{false};

  // Full robot state
  franka::RobotState current_robot_state_;
  franka::RobotState temp_franka_robot_state_;

  // Robot command mode
  RobotCommandMode current_robot_command_mode_ = RobotCommandMode::IDLE;
  RobotCommandMode next_robot_command_mode_ = RobotCommandMode::IDLE;

  // Flags for disabling commands until the first read pass
  bool is_internal_read_required_ = false;
  bool is_external_read_required_ = false;

  // Mutexes for thread safety
  std::mutex robot_state_mutex_;
  std::mutex command_mutex_;

  std::shared_ptr<Robot> robot_;
  const rclcpp::Logger logger_;

  static constexpr double communication_period_error_tolerance_ = 5e-4; //0.5 ms
};

}  // namespace franka_hardware
