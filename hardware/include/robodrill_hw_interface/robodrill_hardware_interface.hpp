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

#ifndef ROBODRILL_HW_INTERFACE__ROBODRILL_HARDWARE_INTERFACE_HPP_
#define ROBODRILL_HW_INTERFACE__ROBODRILL_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <net/if.h>
#include <unistd.h>

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

// SOEM EtherCAT library
#include <soem/soem.h>
namespace robodrill_hw_interface
{
  class RobodrillHardwareInterface : public hardware_interface::SystemInterface
  {
    struct Config
    {
      std::string left_wheel_name = "";
      std::string right_wheel_name = "";
      std::string turret_name = "";
      float transmission_ratio_wheels = 0;
      float transmission_ratio_turret = 0;
      int encoder_ticks_per_rev = 0;

      // ethercat slave IDs
      int slave_turret_motor;
      int slave_right_motor;
      int slave_left_motor;
    };

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(RobodrillHardwareInterface)

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

    bool init_ethercat();
    void shutdown_ethercat();
    bool configure_slave(int slave);
    bool enable_slave(int slave);
    uint16_t read_statusword(int slave);
    int32_t convert_rps_to_tps(float rps); // rps = radians per second, tps = ticks per second
    void write_controlword(int slave, uint16_t cw);

    // Convert ticks per second to radians per second
    float tps_to_rps(int32_t tps);

    // Convert absolute ticks to radians
    double ticksToRadians(int32_t ticks);

    // SDO helper functions
    bool write_sdo8(ecx_contextt *context, uint16_t slave, uint16_t index, uint8_t subindex, uint8_t value);
    bool write_sdo16(ecx_contextt *context, uint16_t slave, uint16_t index, uint8_t subindex, uint16_t value);
    bool write_sdo32(ecx_contextt *context, uint16_t slave, uint16_t index, uint8_t subindex, uint32_t value);

  private:
    Config cfg_;
    Wheel wheel_l_;
    Wheel wheel_r_;
    Wheel turret_;
    int m_initial_encoder_ticks_l;
    int m_initial_encoder_ticks_r;
    int32_t m_initial_encoder_ticks_turret;

    ecx_contextt ec_context_; // SOEM EtherCAT context

    // EtherCAT variables
    rclcpp::Logger logger_{rclcpp::get_logger("RobodrillHardwareInterface")};
    std::string ethercat_interface_{"eno1"};
    int slave_count_{0};
    char io_map_[4096];
    int expected_wkc_{0};
    bool ec_initialized_{false};
    
    // Separate buffers for each slave
    std::vector<uint8_t> rx_buffer_left_;
    std::vector<uint8_t> tx_buffer_left_;
    std::vector<uint8_t> rx_buffer_right_;
    std::vector<uint8_t> tx_buffer_right_;
    std::vector<uint8_t> rx_buffer_turret_;
    std::vector<uint8_t> tx_buffer_turret_;

    // PDO sizes
    static constexpr int RX_PDO_SIZE = 7; // Output PDO size in bytes
    static constexpr int TX_PDO_SIZE = 7; // Input PDO size in bytes (actual from drive)

    // CiA 402 operation modes
    static constexpr int8_t MODE_CSV = 9; // Cyclic Synchronous Velocity mode

  };

} // namespace robodrill_hw_interface

#endif // ROBODRILL_HW_INTERFACE__ROBODRILL_HARDWARE_INTERFACE_HPP_

#define GREEN  "\033[1;32m"
#define RESET  "\033[0m"
