#pragma once

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/visibility_control.h>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "franka_action_server.hpp"
#include "franka_hardware/franka_executor.hpp"
#include "franka_hardware/franka_param_service_server.hpp"
#include "franka_hardware/robot.hpp"
#include "franka_hardware/robot_communication_thread.hpp"

#include "franka_hardware/franka_hardware_interface.hpp"

namespace franka_hardware {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class FrankaAsyncHardwareInterface : public FrankaHardwareInterface {
 public:
  FrankaAsyncHardwareInterface() : FrankaHardwareInterface() {};
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

  /**
   * @brief Reads a boolean parameter from the hardware parameters.
   * @param info The hardware info containing parameters.
   * @param param_name The name of the parameter to read.
   * @param value Output parameter to store the boolean value.
   * @return True if the parameter exists, false otherwise.
   */
  bool read_bool_param(const hardware_interface::HardwareInfo& info, const std::string& param_name, bool& value);

  static constexpr int N_JOINTS = 7;

 private:
  std::shared_ptr<RobotCommunicationThread> robot_communication_thread_;

  const rclcpp::Time empty_time_ = rclcpp::Time(0);
  const rclcpp::Duration empty_period_ = rclcpp::Duration(0, 0);
  const std::vector<std::string> empty_string_vector_ = {};

  RobotCommandMode robot_command_mode_{RobotCommandMode::IDLE},
      last_robot_command_mode_{RobotCommandMode::IDLE};

  std::array<double, kNumberOfJoints> last_hw_position_commands_{
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()};
  
  // Cartesian poses are represented as a column-major homogeneous transformation matrix.
  std::array<double, 16> last_hw_cartesian_pose_{ std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()};

  rclcpp::Duration last_duration_ = rclcpp::Duration::from_seconds(0.001);

  void initialize_command_interfaces(const franka::RobotState& robot_state);
  void set_initial_state_interfaces(const franka::RobotState& robot_state);
};

}  // namespace franka_hardware
