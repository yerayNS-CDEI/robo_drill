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

#include "robodrill_hw_interface/robodrill_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robodrill_hw_interface
{
  // =========================================================
  // Internal constants used by this driver
  // =========================================================

  // Mode of operation constants
  static constexpr int MODE_PV = 3;
  static constexpr int MODE_CSV = 9;

  // RPM safety limit (requested by user)
  static constexpr double MAX_RPM = 4910.0;


  // PDO byte offsets inside the process image
  static constexpr int OFFSET_CW = 0;       // Controlword     (0–1)
  static constexpr int OFFSET_TARGET_V = 2; // Target velocity (2–5)

  static constexpr int OFFSET_SW = 0;        // Statusword         (0–1)
  static constexpr int OFFSET_ACT_V = 2;     // Actual velocity    (2–5)
  static constexpr int OFFSET_MODE_DISP = 6; // Mode display       (6)

  int SIZE_OF_INT8 = sizeof(int8_t);
  int SIZE_OF_INT16 = sizeof(int16_t);
  int SIZE_OF_INT32 = sizeof(int32_t);

  uint16_t RobodrillHardwareInterface::read_statusword(int slave)
  {
    std::vector<uint8_t> *tx_buffer;
    if (slave == cfg_.slave_left_motor) tx_buffer = &tx_buffer_left_;
    else if (slave == cfg_.slave_right_motor) tx_buffer = &tx_buffer_right_;
    else tx_buffer = &tx_buffer_turret_;
    
    return (uint16_t)((uint16_t)(*tx_buffer)[OFFSET_SW] |
                      ((uint16_t)(*tx_buffer)[OFFSET_SW + 1] << 8));
  }

  void RobodrillHardwareInterface::write_controlword(int slave, uint16_t cw)
  {
    std::vector<uint8_t> *rx_buffer;
    if (slave == cfg_.slave_left_motor) rx_buffer = &rx_buffer_left_;
    else if (slave == cfg_.slave_right_motor) rx_buffer = &rx_buffer_right_;
    else rx_buffer = &rx_buffer_turret_;
    
    (*rx_buffer)[OFFSET_CW] = (uint8_t)(cw & 0xFF);
    (*rx_buffer)[OFFSET_CW + 1] = (uint8_t)((cw >> 8) & 0xFF);
  }

  void RobodrillHardwareInterface::shutdown_ethercat()
  {
    if (!ec_initialized_)
      return;

    RCLCPP_INFO(logger_, "Shutting down EtherCAT...");

    // Disable all drives with Quick stop
    for (int slave : {cfg_.slave_left_motor, cfg_.slave_right_motor, cfg_.slave_turret_motor})
    {
      write_controlword(slave, 0x0002); // Quick stop
    }
    
    // ecx_send_processdata(&ec_context_);
    // ecx_receive_processdata(&ec_context_, EC_TIMEOUTRET);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Transition all slaves to SAFE-OP
    ec_context_.slavelist[0].state = EC_STATE_SAFE_OP;
    ecx_writestate(&ec_context_, 0);
    
    for (int slave : {cfg_.slave_left_motor, cfg_.slave_right_motor, cfg_.slave_turret_motor})
    {
      ecx_statecheck(&ec_context_, slave, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    }

    // Transition to INIT
    ec_context_.slavelist[0].state = EC_STATE_INIT;
    ecx_writestate(&ec_context_, 0);

    // Close EtherCAT
    ecx_close(&ec_context_);
    ec_initialized_ = false;

    RCLCPP_INFO(logger_, " connection closed.");
  }


  hardware_interface::CallbackReturn RobodrillHardwareInterface::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Allocate internal process buffers for all three slaves
    rx_buffer_left_.resize(RX_PDO_SIZE, 0);
    tx_buffer_left_.resize(TX_PDO_SIZE, 0);
    rx_buffer_right_.resize(RX_PDO_SIZE, 0);
    tx_buffer_right_.resize(TX_PDO_SIZE, 0);
    rx_buffer_turret_.resize(RX_PDO_SIZE, 0);
    tx_buffer_turret_.resize(TX_PDO_SIZE, 0);

    // Parse hardware parameters
    cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    cfg_.turret_name = info_.hardware_parameters["turret_name"];
    cfg_.transmission_ratio_wheels = std::stof(info_.hardware_parameters["transmission_ratio_wheels"]);
    cfg_.transmission_ratio_turret = std::stof(info_.hardware_parameters["transmission_ratio_turret"]);
    cfg_.encoder_ticks_per_rev = std::stoi(info_.hardware_parameters["encoder_ticks_per_rev"]);
    cfg_.slave_turret_motor = std::stoi(info_.hardware_parameters["slave_turret_motor"]);
    cfg_.slave_right_motor = std::stoi(info_.hardware_parameters["slave_right_motor"]);
    cfg_.slave_left_motor = std::stoi(info_.hardware_parameters["slave_left_motor"]);

    wheel_l_.setup(cfg_.left_wheel_name);
    wheel_r_.setup(cfg_.right_wheel_name);
    turret_.setup(cfg_.turret_name);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RobodrillHardwareInterface"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RobodrillHardwareInterface"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RobodrillHardwareInterface"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RobodrillHardwareInterface"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RobodrillHardwareInterface"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // SDO helper functions
  bool RobodrillHardwareInterface::write_sdo8(ecx_contextt *context, uint16_t slave,
                                             uint16_t index, uint8_t subindex, uint8_t value)
  {
    int wkc = ecx_SDOwrite(context, slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
    return (wkc > 0);
  }

  bool RobodrillHardwareInterface::write_sdo16(ecx_contextt *context, uint16_t slave,
                                              uint16_t index, uint8_t subindex, uint16_t value)
  {
    int wkc = ecx_SDOwrite(context, slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
    return (wkc > 0);
  }

  bool RobodrillHardwareInterface::write_sdo32(ecx_contextt *context, uint16_t slave,
                                              uint16_t index, uint8_t subindex, uint32_t value)
  {
    int wkc = ecx_SDOwrite(context, slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
    return (wkc > 0);
  }

  bool RobodrillHardwareInterface::configure_slave(int slave)
  {
    RCLCPP_INFO(logger_, "Configuring slave %d...", slave);

    // Wait for PRE-OP
    ecx_statecheck(&ec_context_, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    if (ec_context_.slavelist[slave].state != EC_STATE_PRE_OP)
    {
      RCLCPP_ERROR(logger_, "Slave %d failed to reach PRE-OP. State=0x%02X",
                   slave, ec_context_.slavelist[slave].state);
      return false;
    }

    // Set operation mode to CSV (9)
    if (!write_sdo8(&ec_context_, slave, 0x6060, 0x00, MODE_CSV))
    {
      RCLCPP_ERROR(logger_, "Slave %d: Failed to set operation mode to CSV (9)", slave);
      return false;
    }

    // Configure PDO mapping
    uint8_t zero8 = 0;
    if (!write_sdo8(&ec_context_, slave, 0x1C12, 0, 1))
    {
      RCLCPP_ERROR(logger_, "Slave %d: Failed to disable RxPDO assignment", slave);
      return false;
    }
    if (!write_sdo8(&ec_context_, slave, 0x1C13, 0, 1))
    {
      RCLCPP_ERROR(logger_, "Slave %d: Failed to disable TxPDO assignment", slave);
      return false;
    }
    //! paste debug_code.txt here, if needed

    RCLCPP_INFO(logger_, "Slave %d: PDO mapping configured successfully.", slave);
    return true;
  }

  bool RobodrillHardwareInterface::init_ethercat()
  {
    RCLCPP_INFO(logger_, "Initializing SOEM on interface %s...",
                ethercat_interface_.c_str());

    // Initialize SOEM context
    if (ecx_init(&ec_context_, ethercat_interface_.c_str()) <= 0)
    {
      RCLCPP_ERROR(logger_, "Failed to initialize SOEM on %s", ethercat_interface_.c_str());
      return false;
    }

    RCLCPP_INFO(logger_, "SOEM initialized. Scanning EtherCAT network...");

    // Discover and configure slaves
    if (ecx_config_init(&ec_context_) <= 0)
    {
      RCLCPP_ERROR(logger_, "No EtherCAT slaves found.");
      ecx_close(&ec_context_);
      return false;
    }

    if (ec_context_.slavecount < 3)
    {
      RCLCPP_ERROR(logger_, "Expected 3 slaves but found %d on the bus!", ec_context_.slavecount);
      ecx_close(&ec_context_);
      return false;
    }

    slave_count_ = ec_context_.slavecount;
    RCLCPP_INFO(logger_, "Found %d slave(s).", slave_count_);

    // Configure all three slaves
    if (!configure_slave(cfg_.slave_left_motor))
    {
      RCLCPP_ERROR(logger_, "Failed to configure left motor slave");
      ecx_close(&ec_context_);
      return false;
    }
    
    if (!configure_slave(cfg_.slave_right_motor))
    {
      RCLCPP_ERROR(logger_, "Failed to configure right motor slave");
      ecx_close(&ec_context_);
      return false;
    }
    
    if (!configure_slave(cfg_.slave_turret_motor))
    {
      RCLCPP_ERROR(logger_, "Failed to configure turret motor slave");
      ecx_close(&ec_context_);
      return false;
    }

    RCLCPP_INFO(logger_, "All slaves configured. Setting up process data mapping...");

    // Configure process data mapping
    ecx_config_map_group(&ec_context_, io_map_, 0);
    ecx_configdc(&ec_context_);

    // Validate PDO sizes for all three slaves
    for (int slave : {cfg_.slave_left_motor, cfg_.slave_right_motor, cfg_.slave_turret_motor})
    {
      RCLCPP_INFO(logger_, "Slave %d I/O map: RxPDO=%d bytes, TxPDO=%d bytes",
                  slave,
                  ec_context_.slavelist[slave].Obytes,
                  ec_context_.slavelist[slave].Ibytes);

      if (ec_context_.slavelist[slave].Obytes != RX_PDO_SIZE)
      {
        RCLCPP_ERROR(logger_, "Slave %d RxPDO size mismatch! Expected=%d, Actual=%d",
                     slave, RX_PDO_SIZE, ec_context_.slavelist[slave].Obytes);
        return false;
      }
      if (ec_context_.slavelist[slave].Ibytes != TX_PDO_SIZE)
      {
        RCLCPP_ERROR(logger_, "Slave %d TxPDO size mismatch! Expected=%d, Actual=%d",
                     slave, TX_PDO_SIZE, ec_context_.slavelist[slave].Ibytes);
        return false;
      }

      RCLCPP_INFO(logger_, "Slave %d SM2 (outputs): addr=0x%04X, length=%d, control=0x%04X",
                  slave,
                  ec_context_.slavelist[slave].SM[2].StartAddr,
                  ec_context_.slavelist[slave].SM[2].SMlength,
                  ec_context_.slavelist[slave].SM[2].SMflags);
      RCLCPP_INFO(logger_, "Slave %d SM3 (inputs): addr=0x%04X, length=%d, control=0x%04X",
                  slave,
                  ec_context_.slavelist[slave].SM[3].StartAddr,
                  ec_context_.slavelist[slave].SM[3].SMlength,
                  ec_context_.slavelist[slave].SM[3].SMflags);
    }

    ec_context_.grouplist[0].blockLRW = 1; // Use separate LRD/LWR
    RCLCPP_INFO(logger_, "Using blockLRW=1 (separate LRD/LWR datagrams)");

    RCLCPP_INFO(logger_, GREEN "All PDO sizes validated successfully." RESET);

    // Request SAFE-OP for all slaves
    ec_context_.slavelist[0].state = EC_STATE_SAFE_OP;
    ecx_writestate(&ec_context_, 0);
    
    for (int slave : {cfg_.slave_left_motor, cfg_.slave_right_motor, cfg_.slave_turret_motor})
    {
      ecx_statecheck(&ec_context_, slave, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
      if (ec_context_.slavelist[slave].state != EC_STATE_SAFE_OP)
      {
        RCLCPP_ERROR(logger_, "Slave %d failed to reach SAFE-OP. State=0x%02X",
                     slave, ec_context_.slavelist[slave].state);
        ecx_close(&ec_context_);
        return false;
      }
    }

    RCLCPP_INFO(logger_, GREEN "All slaves reached SAFE-OP." RESET);

    // Calculate expected WKC using SOEM formula
    uint16_t outputsWKC = ec_context_.grouplist[0].outputsWKC;
    uint16_t inputsWKC = ec_context_.grouplist[0].inputsWKC;

    RCLCPP_INFO(logger_, "WKC calculation: outputsWKC=%d, inputsWKC=%d",
                outputsWKC, inputsWKC);

    // For separate LRD/LWR: send returns outputsWKC*2, receive returns inputsWKC
    if (ec_context_.grouplist[0].blockLRW == 0)
    {
      expected_wkc_ = (outputsWKC * 2) + inputsWKC; 
      RCLCPP_INFO(logger_, "Expected WKC (LRW combined) = %d", expected_wkc_);
    }
    else
    {
      expected_wkc_ = inputsWKC; // receive_processdata only returns input WKC
      RCLCPP_INFO(logger_, "Expected WKC (LRD only) = %d", expected_wkc_);
    }

    ec_initialized_ = true;
    return true;
  }

  std::vector<hardware_interface::StateInterface> RobodrillHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        turret_.name, hardware_interface::HW_IF_POSITION, &turret_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        turret_.name, hardware_interface::HW_IF_VELOCITY, &turret_.vel));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> RobodrillHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        turret_.name, hardware_interface::HW_IF_VELOCITY, &turret_.cmd));

    return command_interfaces;
  }

  bool RobodrillHardwareInterface::enable_slave(int slave)
  {
    RCLCPP_INFO(logger_, "Enabling slave %d via CiA402 state machine...", slave);

    // Initialize target velocity to 0 via SDO
    int32_t zero_velocity = 0;
    write_sdo32(&ec_context_, slave, 0x60FF, 0x00, zero_velocity);

    // Read initial statusword via SDO
    uint16_t sdo_sw = 0;
    int size = sizeof(sdo_sw);
    ecx_SDOread(&ec_context_, slave, 0x6041, 0x00, FALSE, &size, &sdo_sw, EC_TIMEOUTRXM);
    RCLCPP_INFO(logger_, "Slave %d initial statusword: 0x%04X", slave, sdo_sw);

    // Check for fault and attempt reset if needed
    if (sdo_sw & 0x0008)
    {
      RCLCPP_WARN(logger_, "Slave %d in FAULT state (SW=0x%04X). Attempting reset...", slave, sdo_sw);
      
      write_sdo16(&ec_context_, slave, 0x6040, 0x00, 0x0080); // Fault reset
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      write_sdo16(&ec_context_, slave, 0x6040, 0x00, 0x0000); // Clear fault reset bit
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      
      ecx_SDOread(&ec_context_, slave, 0x6041, 0x00, FALSE, &size, &sdo_sw, EC_TIMEOUTRXM);
      if (sdo_sw & 0x0008)
      {
        RCLCPP_ERROR(logger_, "Slave %d fault reset failed! SW=0x%04X", slave, sdo_sw);
        return false;
      }
      RCLCPP_INFO(logger_, "Slave %d fault reset successful.", slave);
    }

    // Seq 1: Disable Voltage → Switch On Disabled
    write_sdo16(&ec_context_, slave, 0x6040, 0x00, 0x0000);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ecx_SDOread(&ec_context_, slave, 0x6041, 0x00, FALSE, &size, &sdo_sw, EC_TIMEOUTRXM);
    RCLCPP_INFO(logger_, "Slave %d after Disable Voltage: SW=0x%04X", slave, sdo_sw);

    // Seq 2: Shutdown → Ready to Switch On
    write_sdo16(&ec_context_, slave, 0x6040, 0x00, 0x0006);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ecx_SDOread(&ec_context_, slave, 0x6041, 0x00, FALSE, &size, &sdo_sw, EC_TIMEOUTRXM);
    RCLCPP_INFO(logger_, "Slave %d after Shutdown: SW=0x%04X", slave, sdo_sw);

    if (sdo_sw & 0x0008)
    {
      RCLCPP_ERROR(logger_, "Slave %d FAULT during Shutdown! SW=0x%04X", slave, sdo_sw);
      return false;
    }

    // Seq 3: Switch On → Switched On
    write_sdo16(&ec_context_, slave, 0x6040, 0x00, 0x0007);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ecx_SDOread(&ec_context_, slave, 0x6041, 0x00, FALSE, &size, &sdo_sw, EC_TIMEOUTRXM);
    RCLCPP_INFO(logger_, "Slave %d after Switch On: SW=0x%04X", slave, sdo_sw);

    if (sdo_sw & 0x0008)
    {
      RCLCPP_ERROR(logger_, "Slave %d FAULT during Switch On! SW=0x%04X", slave, sdo_sw);
      return false;
    }

    // Seq 4: Enable Operation → Operation Enabled
    write_sdo16(&ec_context_, slave, 0x6040, 0x00, 0x000F);
    
    bool operation_enabled = false;
    for (int i = 0; i < 10; i++)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      ecx_SDOread(&ec_context_, slave, 0x6041, 0x00, FALSE, &size, &sdo_sw, EC_TIMEOUTRXM);

      if ((sdo_sw & 0x026F) == 0x0227)
      {
        RCLCPP_INFO(logger_, GREEN "Slave %d reached Operation Enabled (0x%04X)" RESET, slave, sdo_sw);
        operation_enabled = true;
        break;
      }
    }

    if (!operation_enabled)
    {
      RCLCPP_ERROR(logger_, "Slave %d failed to reach Operation Enabled. Final SW=0x%04X", slave, sdo_sw);
      return false;
    }

    return true;
  }

  hardware_interface::CallbackReturn RobodrillHardwareInterface::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("RobodrillHardwareInterface"), "Configuring ...please wait...");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RobodrillHardwareInterface::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("RobodrillHardwareInterface"), "Cleaning up ...please wait...");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RobodrillHardwareInterface::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(logger_, "Activating IRD EtherCAT hardware interface...");

    if (!init_ethercat())
    {
      RCLCPP_ERROR(logger_, "EtherCAT initialization failed.");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Send/receive processdata a few times in SAFE-OP before requesting OP
    RCLCPP_INFO(logger_, "Sending initial processdata cycles in SAFE-OP...");
    for (int i = 0; i < 50; i++)
    {
      ecx_send_processdata(&ec_context_);
      ecx_receive_processdata(&ec_context_, EC_TIMEOUTRET);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Request OPERATIONAL state for all slaves
    RCLCPP_INFO(logger_, "Requesting OPERATIONAL state...");
    ec_context_.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ec_context_, 0);

    // Continue cyclic exchange while waiting for OP
    int timeout_ms = EC_TIMEOUTSTATE * 4;
    auto start = std::chrono::steady_clock::now();
    bool all_operational = false;
    
    while (!all_operational)
    {
      ecx_send_processdata(&ec_context_);
      ecx_receive_processdata(&ec_context_, EC_TIMEOUTRET);
      
      all_operational = true;
      for (int slave : {cfg_.slave_left_motor, cfg_.slave_right_motor, cfg_.slave_turret_motor})
      {
        ecx_statecheck(&ec_context_, slave, EC_STATE_OPERATIONAL, 0);
        if (ec_context_.slavelist[slave].state != EC_STATE_OPERATIONAL)
        {
          all_operational = false;
          break;
        }
      }

      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now() - start)
                         .count();
      if (elapsed > timeout_ms)
      {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Verify all slaves reached OPERATIONAL
    for (int slave : {cfg_.slave_left_motor, cfg_.slave_right_motor, cfg_.slave_turret_motor})
    {
      if (ec_context_.slavelist[slave].state != EC_STATE_OPERATIONAL)
      {
        RCLCPP_ERROR(logger_, "Slave %d failed to reach OPERATIONAL. State=0x%02X, AL=0x%04X",
                     slave,
                     ec_context_.slavelist[slave].state,
                     ec_context_.slavelist[slave].ALstatuscode);
        ecx_close(&ec_context_);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    RCLCPP_INFO(logger_, GREEN "All slaves reached OPERATIONAL state." RESET);

    // Enable all slaves via CiA402 state machine
    if (!enable_slave(cfg_.slave_left_motor))
    {
      RCLCPP_ERROR(logger_, "Failed to enable left motor slave");
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    if (!enable_slave(cfg_.slave_right_motor))
    {
      RCLCPP_ERROR(logger_, "Failed to enable right motor slave");
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    if (!enable_slave(cfg_.slave_turret_motor))
    {
      RCLCPP_ERROR(logger_, "Failed to enable turret motor slave");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize PDO buffers for all slaves with Operation Enabled controlword
    write_controlword(cfg_.slave_left_motor, 0x000F);
    write_controlword(cfg_.slave_right_motor, 0x000F);
    write_controlword(cfg_.slave_turret_motor, 0x000F);

    // Read initial encoder positions for all motors
    ecx_SDOread(&ec_context_, cfg_.slave_left_motor, 0x3129, 0x00, FALSE, &SIZE_OF_INT32, &m_initial_encoder_ticks_l, EC_TIMEOUTRXM);
    ecx_SDOread(&ec_context_, cfg_.slave_right_motor, 0x3129, 0x00, FALSE, &SIZE_OF_INT32, &m_initial_encoder_ticks_r, EC_TIMEOUTRXM);
    ecx_SDOread(&ec_context_, cfg_.slave_turret_motor, 0x3129, 0x00, FALSE, &SIZE_OF_INT32, &m_initial_encoder_ticks_turret, EC_TIMEOUTRXM);

    RCLCPP_INFO(logger_, GREEN "All drives enabled successfully. Ready for PDO cyclic operation." RESET);
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RobodrillHardwareInterface::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(logger_, "Deactivating hardware interface...");

    if (ec_initialized_)
    {
      // Stop all motors by sending zero velocity
      RCLCPP_INFO(logger_, "Stopping all motors...");
      write_sdo32(&ec_context_, cfg_.slave_left_motor, 0x60FF, 0x00, 0);
      write_sdo32(&ec_context_, cfg_.slave_right_motor, 0x60FF, 0x00, 0);
      write_sdo32(&ec_context_, cfg_.slave_turret_motor, 0x60FF, 0x00, 0);
      
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      // Disable all drives via CiA402 state machine
      RCLCPP_INFO(logger_, "Disabling all drives...");
      for (int slave : {cfg_.slave_left_motor, cfg_.slave_right_motor, cfg_.slave_turret_motor})
      {
        write_sdo16(&ec_context_, slave, 0x6040, 0x00, 0x0006); // Disable operation -> Ready to switch on
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        write_sdo16(&ec_context_, slave, 0x6040, 0x00, 0x0000); // Disable voltage
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }

      // Shutdown EtherCAT
      shutdown_ethercat();
      
      RCLCPP_INFO(logger_, GREEN "Hardware interface deactivated successfully." RESET);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RobodrillHardwareInterface::on_shutdown(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(logger_, "Shutting down hardware interface...");

    // Ensure EtherCAT is properly closed if still active
    if (ec_initialized_)
    {
      RCLCPP_WARN(logger_, "EtherCAT still active during shutdown. Forcing cleanup...");
      
      // Stop all motors
      write_sdo32(&ec_context_, cfg_.slave_left_motor, 0x60FF, 0x00, 0);
      write_sdo32(&ec_context_, cfg_.slave_right_motor, 0x60FF, 0x00, 0);
      write_sdo32(&ec_context_, cfg_.slave_turret_motor, 0x60FF, 0x00, 0);
      
      // Disable all drives
      for (int slave : {cfg_.slave_left_motor, cfg_.slave_right_motor, cfg_.slave_turret_motor})
      {
        write_sdo16(&ec_context_, slave, 0x6040, 0x00, 0x0000);
      }
      
      shutdown_ethercat();
    }

    RCLCPP_INFO(logger_, GREEN "Hardware interface shutdown complete." RESET);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // radians per second to ticks per second
  int32_t RobodrillHardwareInterface::convert_rps_to_tps(float rps)
  {
    return static_cast<int32_t>(rps * cfg_.encoder_ticks_per_rev / (2 * M_PI));
  }

  // ticks per second to radians per second
  float RobodrillHardwareInterface::tps_to_rps(int32_t tps)
  {
    return static_cast<float>(tps) * 2 * M_PI / cfg_.encoder_ticks_per_rev;
  }

  // Convert encoder ticks to shaft angle in radians
  double RobodrillHardwareInterface::ticksToRadians(int32_t ticks)
  {
    return (static_cast<double>(ticks) / cfg_.encoder_ticks_per_rev) * 2.0 * M_PI;
  }

  hardware_interface::return_type RobodrillHardwareInterface::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    int32_t enc_vel = 0;
    int32_t enc_pos = 0;
    int size = sizeof(enc_vel);

    // Read left motor via SDO
    ecx_SDOread(&ec_context_, cfg_.slave_left_motor, 0x606C, 0x00, FALSE, &size, &enc_vel, EC_TIMEOUTRXM);
    ecx_SDOread(&ec_context_, cfg_.slave_left_motor, 0x3129, 0x00, FALSE, &size, &enc_pos, EC_TIMEOUTRXM);
    wheel_l_.vel = tps_to_rps(enc_vel/cfg_.transmission_ratio_wheels);
    wheel_l_.pos = ticksToRadians((enc_pos - m_initial_encoder_ticks_l)/cfg_.transmission_ratio_wheels);

    // Read right motor via SDO
    ecx_SDOread(&ec_context_, cfg_.slave_right_motor, 0x606C, 0x00, FALSE, &size, &enc_vel, EC_TIMEOUTRXM);
    ecx_SDOread(&ec_context_, cfg_.slave_right_motor, 0x3129, 0x00, FALSE, &size, &enc_pos, EC_TIMEOUTRXM);
    wheel_r_.vel = tps_to_rps(enc_vel/cfg_.transmission_ratio_wheels);
    wheel_r_.pos = ticksToRadians((enc_pos - m_initial_encoder_ticks_r)/cfg_.transmission_ratio_wheels);

    // Read turret motor via SDO
    ecx_SDOread(&ec_context_, cfg_.slave_turret_motor, 0x606C, 0x00, FALSE, &size, &enc_vel, EC_TIMEOUTRXM);
    ecx_SDOread(&ec_context_, cfg_.slave_turret_motor, 0x3129, 0x00, FALSE, &size, &enc_pos, EC_TIMEOUTRXM);
    turret_.vel = tps_to_rps(enc_vel/cfg_.transmission_ratio_turret);
    turret_.pos = ticksToRadians((enc_pos - m_initial_encoder_ticks_turret)/cfg_.transmission_ratio_turret);

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type robodrill_hw_interface::RobodrillHardwareInterface::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // Write left motor via SDO
    int32_t tps_left = (int32_t)(convert_rps_to_tps(wheel_l_.cmd*cfg_.transmission_ratio_wheels));
    if (!write_sdo32(&ec_context_, cfg_.slave_left_motor, 0x60FF, 0x00, tps_left))
    {
      static int sdo_error_count_left = 0;
      if (++sdo_error_count_left % 100 == 0)
      {
        RCLCPP_WARN(logger_, "Left motor: Failed to write velocity (error count: %d)", sdo_error_count_left);
      }
    }

    // Write right motor via SDO
    int32_t tps_right = (int32_t)(convert_rps_to_tps(wheel_r_.cmd*cfg_.transmission_ratio_wheels));
    if (!write_sdo32(&ec_context_, cfg_.slave_right_motor, 0x60FF, 0x00, tps_right))
    {
      static int sdo_error_count_right = 0;
      if (++sdo_error_count_right % 100 == 0)
      {
        RCLCPP_WARN(logger_, "Right motor: Failed to write velocity (error count: %d)", sdo_error_count_right);
      }
    }

    // Write turret motor via SDO
    int32_t tps_turret = (int32_t)(convert_rps_to_tps(turret_.cmd*cfg_.transmission_ratio_turret));
    if (!write_sdo32(&ec_context_, cfg_.slave_turret_motor, 0x60FF, 0x00, tps_turret))
    {
      static int sdo_error_count_turret = 0;
      if (++sdo_error_count_turret % 100 == 0)
      {
        RCLCPP_WARN(logger_, "Turret motor: Failed to write velocity (error count: %d)", sdo_error_count_turret);
      }
    }

    return hardware_interface::return_type::OK;
  }

} // namespace robodrill_hw_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    robodrill_hw_interface::RobodrillHardwareInterface, hardware_interface::SystemInterface)
