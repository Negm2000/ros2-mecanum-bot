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

#ifndef MECBOT__SYSTEM_HPP_
#define MECBOT__SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include "arduino_comms.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
// #include "visibility_control.h"
#include "mecanumbot_hardware_compiler.h"
#include "Wheel.hpp"

namespace debict
{
    namespace mecanumbot
    {
        namespace hardware
        {
            class MecanumbotHardware : public hardware_interface::SystemInterface
            {
            public:

              struct Config // POD struct to hold config data
              {
                std::vector <std::string> joint_names = {};
                std::string device = "";
                float loop_rate = 0;
                int baud_rate = 0;
                int timeout_ms = 0;
                int encoder_ticks_per_rev = 0;
                int pid_p = 0; // PID proportional
                int pid_d = 0; // PID derivative
                int pid_i = 0; // PID integral
                int pid_o = 0; // PID output scaling 

                Config() = default;
              };
              
              RCLCPP_SHARED_PTR_DEFINITIONS(MecanumbotHardware)

              DEBICT_MECANUMBOT_HARDWARE_PUBLIC
              virtual hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareInfo & info) override;

              DEBICT_MECANUMBOT_HARDWARE_PUBLIC
              std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

              DEBICT_MECANUMBOT_HARDWARE_PUBLIC
              std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

              DEBICT_MECANUMBOT_HARDWARE_PUBLIC
              virtual hardware_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State & previous_state) override;

              DEBICT_MECANUMBOT_HARDWARE_PUBLIC
              virtual hardware_interface::CallbackReturn on_cleanup(
                const rclcpp_lifecycle::State & previous_state) override;


              DEBICT_MECANUMBOT_HARDWARE_PUBLIC
              virtual hardware_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State & previous_state) override;

              DEBICT_MECANUMBOT_HARDWARE_PUBLIC
              virtual hardware_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State & previous_state) override;

              DEBICT_MECANUMBOT_HARDWARE_PUBLIC
              virtual hardware_interface::return_type read(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;

              DEBICT_MECANUMBOT_HARDWARE_PUBLIC
              virtual hardware_interface::return_type write(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;


            private:
              ArduinoComms arduino_comms_;
              Config cfg_;
              std::vector<Wheel> wheels_;
              
            };  // class MecbotHardware

        }  // namespace mecbot_arduino
    }  // namespace hardware
}  // namespace debict

#endif  
