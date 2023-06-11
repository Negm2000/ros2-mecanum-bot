
// #include <hardware_interface/types/hardware_interface_type_values.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_lifecycle/state.hpp>
// #include <vector>
// #include <string>

// #include "mecanumbot_hardware/mecanumbot_hardware.hpp"


// hardware_interface::CallbackReturn MecanumbotHardware::on_init(const hardware_interface::HardwareInfo & hardware_info)
// {
//     hardware_interface::CallbackReturn baseResult = hardware_interface::SystemInterface::on_init(hardware_info);
//     if (baseResult != hardware_interface::CallbackReturn::SUCCESS) {
//         return baseResult;
//     }

//     serial_port_name_ = info_.hardware_parameters["serial_port"];
//     motor_ids_.resize(info_.joints.size());
//     position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
//     velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
//     velocity_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
//     velocity_commands_saved_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    
//     for (hardware_interface::ComponentInfo & joint : info_.joints)
//     {
//         if (joint.parameters["motor_id"].empty()) {
//             RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Motor id not defined for join %s", joint.name.c_str());
//             return hardware_interface::CallbackReturn::ERROR;
//         }
//         if (joint.command_interfaces.size() != 1) {
//             RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid number of command interfaces (expected: 1)");
//             return hardware_interface::CallbackReturn::ERROR;
//         }
//         if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
//             RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid joint command interface 0 type (expected: velocity)");
//             return hardware_interface::CallbackReturn::ERROR;
//         }

//         if (joint.state_interfaces.size() != 2) {
//             RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid number of state interfaces (expected: 2)");
//             return hardware_interface::CallbackReturn::ERROR;
//         }
//         if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
//             RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid joint state interface 0 type (expected: position)");
//             return hardware_interface::CallbackReturn::ERROR;
//         }
//         if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
//             RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid joint state interface 1 type (expected: velocity)");
//             return hardware_interface::CallbackReturn::ERROR;
//         }
//     }

//     for (size_t i = 0; i < info_.joints.size(); i++) {
//         motor_ids_[i] = (uint8_t)std::stoi(info_.joints[i].parameters["motor_id"]);
//         RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "%s mapped to motor %d", info_.joints[i].name.c_str(), motor_ids_[i]);
//     }

//     return hardware_interface::CallbackReturn::SUCCESS;
// }

// std::vector<hardware_interface::StateInterface> MecanumbotHardware::export_state_interfaces()
// {
//     RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "export_state_interfaces");

//     std::vector<hardware_interface::StateInterface> state_interfaces;
//     for (size_t i = 0; i < info_.joints.size(); i++) {
//         RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Adding position state interface: %s", info_.joints[i].name.c_str());
//         state_interfaces.emplace_back(
//             hardware_interface::StateInterface(
//                 info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]
//             )
//         );
//     }
//     for (size_t i = 0; i < info_.joints.size(); i++) {
//         RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Adding velocity state interface: %s", info_.joints[i].name.c_str());
//         state_interfaces.emplace_back(
//             hardware_interface::StateInterface(
//                 info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]
//             )
//         );
//     }
//     return state_interfaces;
// }

// std::vector<hardware_interface::CommandInterface> MecanumbotHardware::export_command_interfaces()
// {
//     RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "export_command_interfaces");

//     std::vector<hardware_interface::CommandInterface> command_interfaces;
//     for (size_t i = 0; i < info_.joints.size(); i++) {
//         RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Adding velocity command interface: %s", info_.joints[i].name.c_str());
//         command_interfaces.emplace_back(
//             hardware_interface::CommandInterface(
//                 info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]
//             )
//         );
//     }
//     return command_interfaces;
// }

// hardware_interface::CallbackReturn MecanumbotHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
// {
//     RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware starting ...");

//     for (size_t i = 0; i < info_.joints.size(); i++) {
//         if (std::isnan(position_states_[i])) {
//             position_states_[i] = 0.0f;
//         }
//         if (std::isnan(velocity_states_[i])) {
//             velocity_states_[i] = 0.0f;
//         }
//         if (std::isnan(velocity_commands_[i])) {
//             velocity_commands_[i] = 0.0f;
//         }
//         velocity_commands_saved_[i] = velocity_commands_[i];
//     }

//     serial_port_ = std::make_shared<MecanumbotSerialPort>();
//     if (serial_port_->open(serial_port_name_) != return_type::SUCCESS) {
//         RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware failed to open serial port");
//         return hardware_interface::CallbackReturn::ERROR;
//     }

//     RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware started");
//     return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn MecanumbotHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
// {
//     RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware stopping ...");

//     if (serial_port_->is_open()) {
//         serial_port_->close();
//         serial_port_.reset();
//     }

//     RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware stopped");
//     return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::return_type MecanumbotHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
// {

//     // We currently have an ack response, so read the frames
//     std::vector<SerialHdlcFrame> frames;
//     serial_port_->read_frames(frames);

//     /*
//     for (size_t i = 0; i < frames.size(); i++) {
//         char buff[100];
//         int offset = 0;
//         for (size_t l = 0; l < frames[i].length; l++) {
//             sprintf(&buff[offset], "%02X ", frames[i].data[l]);
//             offset += 3;
//         }
//         RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Frame received: %s", buff);
//     }
//     */

//     for (size_t i = 0; i < info_.joints.size(); i++) {
//         //RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Got position %.5f, velocity %.5f for joint %d!", position_states_[i], velocity_states_[i], i);
//     }
//     position_states_[0] = 1.1f;
//     return hardware_interface::return_type::OK;
// }

// hardware_interface::return_type MecanumbotHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
// {
//     for (size_t i = 0; i < info_.joints.size(); i++) {
//         // Only send motor commands if the velocity changed
//         if (velocity_commands_[i] != velocity_commands_saved_[i]) {

//             RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Motor velocity changed: %.5f", velocity_commands_[i]);

//             // Generate the motor command message
//             uint16_t duty = 0;
//             uint8_t message[6];
//             message[0] = (uint8_t)DeviceCommand::MotorSetDuty;
//             message[1] = 4; // Payload len
//             message[2] = motor_ids_[i];
//             if (velocity_commands_[i] >= 0.0) {
//                 duty = (uint16_t)(velocity_commands_[i]);
//                 message[3] = (uint8_t)DeviceMotorDirection::Forward;
//             } else {
//                 duty = (uint16_t)(-velocity_commands_[i]);
//                 message[3] = (uint8_t)DeviceMotorDirection::Reverse;
//             }
//             message[4] = (uint8_t)(duty & 0xFF);
//             message[5] = (uint8_t)((duty >> 8) & 0xFF);

//             // Send the motor command
//             serial_port_->write_frame(message, 6);

//             // Store the current velocity
//             velocity_commands_saved_[i] = velocity_commands_[i];
//         }
//     }
//     return hardware_interface::return_type::OK;
// }



#include "mecanumbot_hardware/mecbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>
#include <pluginlib/class_list_macros.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"




using namespace debict::mecanumbot::hardware;


  // Here we pass in hardware info from the control.xacro and use it to initialize the hardware
hardware_interface::CallbackReturn MecanumbotHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get the joint names from the hardware info
    cfg_.joint_names.emplace_back(info_.hardware_parameters["fl_wheel_joint_name"]);
    cfg_.joint_names.emplace_back(info_.hardware_parameters["fr_wheel_joint_name"]);
    cfg_.joint_names.emplace_back(info_.hardware_parameters["rl_wheel_joint_name"]);
    cfg_.joint_names.emplace_back(info_.hardware_parameters["rr_wheel_joint_name"]);
    cfg_.loop_rate =std::stof(info_.hardware_parameters["loop_rate"]);
     RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Loop rate: %f", cfg_.loop_rate);
    cfg_.device = "/dev/ttyACM0";
     RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Device = %s", cfg_.device.c_str());
    cfg_.baud_rate =std::stoi(info_.hardware_parameters["baud_rate"]);
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Baud rate: %d", cfg_.baud_rate);
    cfg_.timeout_ms =std::stoi(info_.hardware_parameters["timeout_ms"]);
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Timeout: %d", cfg_.timeout_ms);
    cfg_.encoder_ticks_per_rev = 26;
    if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "PID values not supplied, using defaults.");
  }

    // Configure the wheels
    for (int i=0; i<4; i++){
      wheels_.emplace_back();
      wheels_[i].setup(cfg_.joint_names[i], cfg_.encoder_ticks_per_rev);
    }

    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "MecanumbotHardware initialized successfully");



  // Looping over the joints to assert that they make sense for this robot (ie. Velocity command interface, and Position and Velocity state interfaces)
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MecanumbotHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MecanumbotHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MecanumbotHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MecanumbotHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MecanumbotHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// 
std::vector<hardware_interface::StateInterface> MecanumbotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &wheels_[i].pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &wheels_[i].vel));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MecanumbotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &wheels_[i].cmd));
  }

  return command_interfaces;
}

// On activate is for anything that needs to be done when the hardware is about to start
hardware_interface::CallbackReturn MecanumbotHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Activating ...please wait...");

  // Set PID values
  if(cfg_.pid_p > 0)
  {
    arduino_comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
  }


   
  RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// On deactivate is for anything that needs to be done when the hardware is about to stop
hardware_interface::CallbackReturn MecanumbotHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Deactivating ...please wait...");

  // Stop hardware communication
  arduino_comms_.disconnect();

  RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::CallbackReturn MecanumbotHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Configuring ...please wait...");

  // Start hardware communication

  do
    {  try
      {
          arduino_comms_.connect(cfg_.device,cfg_.baud_rate,cfg_.timeout_ms);
      }
      catch(const std::exception& e)
      {
        RCLCPP_ERROR(rclcpp::get_logger("MecanumbotHardware"), "Failed to connect to arduino: %s", e.what());
      }
      
    } while (!arduino_comms_.connected());


   
  RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumbotHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Cleaning up ...please wait...");

  // Stop hardware communication
  if (arduino_comms_.connected()) arduino_comms_.disconnect();

  RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::return_type MecanumbotHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{

 if (!arduino_comms_.connected()) return hardware_interface::return_type::ERROR;


  // Read encoder values to get the current position and velocity of the wheels
  arduino_comms_.read_encoder_values(wheels_);

  // To get the velocity, we store the previous position and measure the change in position in the measured duration of time between them
  double prev_positions[4];
  for (int i=0;i<4;i++)
    prev_positions[i] = wheels_[i].pos;


  // Get the current position of the wheels and store it in the pos variable ==> pos = encoder counts * radians per encoder count
  for (int i=0;i<4;i++){
    wheels_[i].pos = wheels_[i].getEncoderAngle(); // Encoder counts are converted to radians by multiplying by the number of radians per encoder count
  }

  // Calculate the velocity

  for (int i=0;i<4;i++){
    wheels_[i].vel = (wheels_[i].pos - prev_positions[i]) / period.seconds(); // Velocity is calculated by dividing the change in position by the time elapsed
  }



  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecanumbotHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

 if (!arduino_comms_.connected()) return hardware_interface::return_type::ERROR;  

    int motor_counts_per_loop[4];
    /*
    The motor_counts_per_loop array stores the number of encoder counts that each wheel should rotate in one loop cycle. 
    This is calculated by dividing the desired angular velocity of the wheel (wheels_[i].cmd) by the number of radians per encoder count (wheels_[i].rads_per_count) and then by the loop rate (cfg_.loop_rate).
    This gives the number of encoder counts per second, which is then multiplied by the loop duration to get the number of counts per loop.
    */

    for (int i=0; i<4; i++)
      motor_counts_per_loop[i] = wheels_[i].cmd/wheels_[i].rads_per_count / cfg_.loop_rate; 

    arduino_comms_.set_motor_values(motor_counts_per_loop);
    
      

  return hardware_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(
    debict::mecanumbot::hardware::MecanumbotHardware,
    hardware_interface::SystemInterface
)
