#include <fmt/core.h>
#include <algorithm>
#include <cmath>
#include <exception>

#include <franka/exception.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include "franka_hardware/utils.hpp"
#include "realtime_tools/realtime_helpers.hpp"

#include "franka_hardware/franka_async_hardware_interface.hpp"

namespace franka_hardware {

using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

CallbackReturn FrankaAsyncHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  CallbackReturn return_value = this->FrankaHardwareInterface::on_init(info);
  if (return_value != CallbackReturn::SUCCESS) {
    return return_value;
  }
  if (robot_ == nullptr) {
    RCLCPP_FATAL(getLogger(), "Robot object is not initialized");
    return CallbackReturn::ERROR;
  }

  // Read the filter_commands parameter from the hardware parameters
  bool filter_commands = false;
  if (auto param_value = info.hardware_parameters.find("filter_commands"); param_value != info.hardware_parameters.end())
  {
    std::string value_lower = param_value->second;
    std::transform(value_lower.begin(), value_lower.end(), value_lower.begin(), ::tolower);
    filter_commands = (value_lower == "true");
  }

  // Initialize the robot communication thread
  robot_communication_thread_ = std::make_shared<RobotCommunicationThread>(robot_);

  // Set the filter commands flag
  robot_communication_thread_->set_filter_commands(filter_commands);
  // TODO: Wait for the robot state to be available
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaAsyncHardwareInterface::on_activate(const rclcpp_lifecycle::State&) {
  robot_communication_thread_->enable();
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaAsyncHardwareInterface::on_deactivate(const rclcpp_lifecycle::State&) {
  robot_communication_thread_->disable();
  return CallbackReturn::SUCCESS;
}

void FrankaAsyncHardwareInterface::initialize_command_interfaces(
    const franka::RobotState& robot_state) {
  // The command interfaces are initialized only once when the robot is activated
  // and the command mode is switched
  if (robot_command_mode_ == last_robot_command_mode_) {
    return;
  }
  if (robot_command_mode_ == RobotCommandMode::EFFORT) {
    hw_effort_commands_.fill(0.0);
  }
  if (robot_command_mode_ == RobotCommandMode::JOINT_POSITION) {
    hw_position_commands_ = robot_state.q_d;
    last_hw_position_commands_ = robot_state.q_d;
  }
  if (robot_command_mode_ == RobotCommandMode::JOINT_VELOCITY) {
    hw_velocity_commands_.fill(0.0);
  }
  if (robot_command_mode_ == RobotCommandMode::CARTESIAN_POSE ||
      robot_command_mode_ == RobotCommandMode::CARTESIAN_POSE_WITH_ELBOW) {
    hw_cartesian_pose_ = robot_state.O_T_EE_d;
    last_hw_cartesian_pose_ = robot_state.O_T_EE_d;
  }
  if (robot_command_mode_ == RobotCommandMode::CARTESIAN_VELOCITY ||
      robot_command_mode_ == RobotCommandMode::CARTESIAN_VELOCITY_WITH_ELBOW) {
    hw_cartesian_velocities_.fill(0.0);
  }
  if (robot_command_mode_ == RobotCommandMode::CARTESIAN_POSE_WITH_ELBOW ||
      robot_command_mode_ == RobotCommandMode::CARTESIAN_VELOCITY_WITH_ELBOW) {
    hw_elbow_command_ = robot_state.elbow_c;
  }
}

void FrankaAsyncHardwareInterface::set_initial_state_interfaces(
    const franka::RobotState& robot_state) {
  // The initial state interfaces are initialized only once when the robot is activated
  if (robot_command_mode_ == last_robot_command_mode_) {
    return;
  }

  if (robot_command_mode_ == RobotCommandMode::JOINT_POSITION) {
    initial_joint_positions_ = robot_state.q_d;
  }

  if (robot_command_mode_ == RobotCommandMode::CARTESIAN_POSE ||
      robot_command_mode_ == RobotCommandMode::CARTESIAN_POSE_WITH_ELBOW) {
    initial_robot_pose_ = robot_state.O_T_EE_d;
  }

  if (robot_command_mode_ == RobotCommandMode::CARTESIAN_POSE_WITH_ELBOW ||
      robot_command_mode_ == RobotCommandMode::CARTESIAN_VELOCITY_WITH_ELBOW) {
    initial_elbow_state_ = robot_state.elbow_c;
  }
}

hardware_interface::return_type FrankaAsyncHardwareInterface::read(const rclcpp::Time&,
                                                                   const rclcpp::Duration&) {
  if (hw_franka_model_ptr_ == nullptr) {
    hw_franka_model_ptr_ = robot_communication_thread_->get_model();
  }
  robot_communication_thread_->get_current_robot_state(hw_franka_robot_state_, robot_command_mode_);

  hw_positions_ = hw_franka_robot_state_.q;
  hw_velocities_ = hw_franka_robot_state_.dq;
  hw_efforts_ = hw_franka_robot_state_.tau_J;

  initialize_command_interfaces(hw_franka_robot_state_);
  set_initial_state_interfaces(hw_franka_robot_state_);
  last_robot_command_mode_ = robot_command_mode_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaAsyncHardwareInterface::write(
    const rclcpp::Time&,
    const rclcpp::Duration& duration) {
  if (robot_command_mode_ == RobotCommandMode::JOINT_POSITION) {
    for (size_t i = 0; i < N_JOINTS; ++i) {
      hw_velocity_commands_[i] =
          (hw_position_commands_[i] - last_hw_position_commands_[i]) / duration.seconds();
    }
  } else if (robot_command_mode_ == RobotCommandMode::CARTESIAN_POSE ||
             robot_command_mode_ == RobotCommandMode::CARTESIAN_POSE_WITH_ELBOW) {
    const Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> last_cartesian_pose(
        last_hw_cartesian_pose_.data());
    const Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> hw_cartesian_pose(
        hw_cartesian_pose_.data());
    Eigen::Map<Eigen::Matrix<double, 6, 1>> cartesian_twist(hw_cartesian_velocities_.data());

    compute_twist(last_cartesian_pose, hw_cartesian_pose, cartesian_twist, duration.seconds());
  }

  last_hw_position_commands_ = hw_position_commands_;
  last_hw_cartesian_pose_ = hw_cartesian_pose_;
  robot_communication_thread_->write_commands(hw_effort_commands_, hw_position_commands_,
                                              hw_velocity_commands_, hw_cartesian_pose_,
                                              hw_cartesian_velocities_, hw_elbow_command_);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaAsyncHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  auto contains_interface_type = [](const std::string& interface,
                                    const std::string& interface_type) {
    size_t slash_position = interface.find('/');
    if (slash_position != std::string::npos && slash_position + 1 < interface.size()) {
      std::string after_slash = interface.substr(slash_position + 1);
      return after_slash == interface_type;
    }
    return false;
  };

  auto generate_error_message = [this](const std::string& start_stop_command,
                                       const std::string& interface_name,
                                       size_t actual_interface_size,
                                       size_t expected_interface_size) {
    std::string error_message =
        fmt::format("Invalid number of {} interfaces to {}. Expected {}, given {}", interface_name,
                    start_stop_command, expected_interface_size, actual_interface_size);
    RCLCPP_FATAL(this->getLogger(), "%s", error_message.c_str());

    throw std::invalid_argument(error_message);
  };

  for (const auto& interface : command_interfaces_info_) {
    size_t num_stop_interface =
        std::count_if(stop_interfaces.begin(), stop_interfaces.end(),
                      [contains_interface_type, &interface](const std::string& interface_given) {
                        return contains_interface_type(interface_given, interface.interface_type);
                      });
    size_t num_start_interface =
        std::count_if(start_interfaces.begin(), start_interfaces.end(),
                      [contains_interface_type, &interface](const std::string& interface_given) {
                        return contains_interface_type(interface_given, interface.interface_type);
                      });

    if (num_stop_interface == interface.size) {
      interface.claim_flag = false;
    } else if (num_stop_interface != 0U) {
      generate_error_message("stop", interface.interface_type, num_stop_interface, interface.size);
    }
    if (num_start_interface == interface.size) {
      interface.claim_flag = true;
    } else if (num_start_interface != 0U) {
      generate_error_message("start", interface.interface_type, num_start_interface,
                             interface.size);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaAsyncHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>&,
    const std::vector<std::string>&) {
  // The robot command mode is switched in the asynchronous control loop
  if (effort_interface_claimed_) {
    robot_communication_thread_->request_command_mode_switch(RobotCommandMode::EFFORT);
  } else if (velocity_joint_interface_claimed_) {
    robot_communication_thread_->request_command_mode_switch(RobotCommandMode::JOINT_VELOCITY);
  } else if (position_joint_interface_claimed_) {
    robot_communication_thread_->request_command_mode_switch(RobotCommandMode::JOINT_POSITION);
  } else if (velocity_cartesian_interface_claimed_) {
    if (elbow_command_interface_claimed_) {
      robot_communication_thread_->request_command_mode_switch(
          RobotCommandMode::CARTESIAN_VELOCITY_WITH_ELBOW);
    } else {
      robot_communication_thread_->request_command_mode_switch(
          RobotCommandMode::CARTESIAN_VELOCITY);
    }
  } else if (pose_cartesian_interface_claimed_) {
    if (elbow_command_interface_claimed_) {
      robot_communication_thread_->request_command_mode_switch(
          RobotCommandMode::CARTESIAN_POSE_WITH_ELBOW);
    } else {
      robot_communication_thread_->request_command_mode_switch(RobotCommandMode::CARTESIAN_POSE);
    }
  } else {
    robot_communication_thread_->request_command_mode_switch(RobotCommandMode::IDLE);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace franka_hardware

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_hardware::FrankaAsyncHardwareInterface,
                       hardware_interface::SystemInterface)
