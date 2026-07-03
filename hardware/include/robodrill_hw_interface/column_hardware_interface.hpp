// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef COLUMN_HW_INTERFACE__COLUMN_HARDWARE_INTERFACE_HPP_
#define COLUMN_HW_INTERFACE__COLUMN_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <modbus/modbus.h>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "robodrill_hw_interface/visibility_control.h"

#include "wheel.hpp"


namespace column_hw_interface
{
  class ColumnHardwareInterface : public hardware_interface::SystemInterface
  {
    struct Config
    {
      std::string column_name = "";
      
    };

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ColumnHardwareInterface)

    ROBODRILL_HW_INTERFACE_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    ROBODRILL_HW_INTERFACE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    ROBODRILL_HW_INTERFACE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    ROBODRILL_HW_INTERFACE_PUBLIC
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    ROBODRILL_HW_INTERFACE_PUBLIC
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state) override;

    ROBODRILL_HW_INTERFACE_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    ROBODRILL_HW_INTERFACE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    ROBODRILL_HW_INTERFACE_PUBLIC
    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &previous_state) override;

    ROBODRILL_HW_INTERFACE_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    ROBODRILL_HW_INTERFACE_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    Config cfg_;
    robodrill_hw_interface::Wheel column_;
    // Modbus registers
    static constexpr uint16_t REG_STATUS = 0x2001;
    static constexpr uint16_t REG_COMMAND_POSITION = 0x2002;
    static constexpr uint16_t REG_COMMAND_CURRENT = 0x2003;
    static constexpr uint16_t REG_COMMAND_SPEED = 0x2004;
    static constexpr uint16_t REG_COMMAND_SOFT_START = 0x2005;
    static constexpr uint16_t REG_COMMAND_SOFT_STOP = 0x2006;
    static constexpr uint16_t REG_FB_POSITION = 0x2101;
    static constexpr uint16_t REG_FB_CURRENT = 0x2102;
    static constexpr uint16_t REG_STATUS_FLAG = 0x2103;
    static constexpr uint16_t REG_ERROR_CODE = 0x2104;
    static constexpr uint16_t REG_FB_SPEED = 0x2105;

    // Commands
    static constexpr uint16_t CMD_STOP = 0xFB03;
    static constexpr uint16_t CMD_RUN_OUT = 0xFB01;
    static constexpr uint16_t CMD_RUN_IN = 0xFB02;

    // Connection parameters
    static constexpr const char* LC3_IP = "192.168.1.10";
    static constexpr int LC3_PORT = 502;
    static constexpr int SLAVE_ID = 1;
    // Modbus client
    modbus_t* modbus_ctx_;

    // State variables
    uint16_t heartbeat_;
    int32_t last_position_;
    bool has_last_position_;

    bool general_run_prerequisites();
    hardware_interface::return_type send_target_position(int32_t target_position);
  };

} // namespace column_hw_interface

#endif // COLUMN_HW_INTERFACE__COLUMN_HARDWARE_INTERFACE_HPP_

#define GREEN  "\033[1;32m"
#define RESET  "\033[0m"