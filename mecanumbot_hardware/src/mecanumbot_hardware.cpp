

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
    gui_subscriber_.last_message_.clear();
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

    rclcpp::spin_some(gui_subscriber_.get_node_base_interface());
    if (!gui_subscriber_.last_message_.empty()){
      RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Going to %s", gui_subscriber_.last_message_.c_str());
      std::cout << gui_subscriber_.last_message_[0] << std::endl;
      arduino_comms_.send_gui_command(1);

      // switch (gui_subscriber_.last_message_[0])
      // {
      // case 'T':
      //   arduino_comms_.send_gui_command(1);
      //   break;
      
      // default:
      //   arduino_comms_.send_gui_command(1);
      //   gui_subscriber_.last_message_.clear();
      //   break;
      // }
      gui_subscriber_.last_message_.clear();
    } 

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
