#include "Mecanum_firmware/Mecanum_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sstream>
#include <iomanip>
#include <cmath>

namespace mecanum_firmware
{
namespace
{
// Wheel order used throughout this file AND expected on the Arduino side
// of the serial protocol. Your ros2_control URDF/xacro must declare the
// 4 wheel joints in this exact order: front_right, front_left, back_right, back_left
constexpr std::size_t NUM_WHEELS  = 4;
constexpr std::size_t FRONT_RIGHT = 0;
constexpr std::size_t FRONT_LEFT  = 1;
constexpr std::size_t BACK_RIGHT  = 2;
constexpr std::size_t BACK_LEFT   = 3;

std::string wheelPrefix(std::size_t index)
{
  switch (index)
  {
    case FRONT_RIGHT: return "fr";
    case FRONT_LEFT:  return "fl";
    case BACK_RIGHT:  return "br";
    case BACK_LEFT:   return "bl";
    default:          return "??";
  }
}

int wheelIndexFromPrefix(const std::string &prefix)
{
  if (prefix == "fr") return static_cast<int>(FRONT_RIGHT);
  if (prefix == "fl") return static_cast<int>(FRONT_LEFT);
  if (prefix == "br") return static_cast<int>(BACK_RIGHT);
  if (prefix == "bl") return static_cast<int>(BACK_LEFT);
  return -1;
}
}  // namespace

MecanumInterface::MecanumInterface()
{
}

MecanumInterface::~MecanumInterface()
{
  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("MecanumInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}

CallbackReturn MecanumInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  if (info_.joints.size() != NUM_WHEELS)
  {
    RCLCPP_FATAL(rclcpp::get_logger("MecanumInterface"),
                 "Expected 4 wheel joints (front_right, front_left, back_right, back_left), "
                 "got %zu. Check your ros2_control URDF/xacro.",
                 info_.joints.size());
    return CallbackReturn::FAILURE;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("MecanumInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  velocity_commands_.reserve(NUM_WHEELS);
  position_states_.reserve(NUM_WHEELS);
  velocity_states_.reserve(NUM_WHEELS);
  last_run_ = rclcpp::Clock().now();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MecanumInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MecanumInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn MecanumInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("MecanumInterface"), "Starting Mecanum hardware ...");

  velocity_commands_ = { 0.0, 0.0, 0.0, 0.0 };
  position_states_   = { 0.0, 0.0, 0.0, 0.0 };
  velocity_states_   = { 0.0, 0.0, 0.0, 0.0 };

  try
  {
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("MecanumInterface"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("MecanumInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MecanumInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("MecanumInterface"), "Stopping Mecanum hardware ...");

  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("MecanumInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("MecanumInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}

// Expected line format from Arduino, one token per wheel, comma separated:
//   fr<p|n><value>,fl<p|n><value>,br<p|n><value>,bl<p|n><value>,
// e.g.  frp12.34,fln05.67,brp00.00,blp00.00,
hardware_interface::return_type MecanumInterface::read(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  if (arduino_.IsDataAvailable())
  {
    auto dt = (rclcpp::Clock().now() - last_run_).seconds();
    std::string message;
    arduino_.ReadLine(message);
    std::stringstream ss(message);
    std::string token;

    while (std::getline(ss, token, ','))
    {
      if (token.size() < 4)
      {
        continue;  // malformed token, skip it
      }

      std::string prefix = token.substr(0, 2);
      char sign_char = token.at(2);
      int wheel = wheelIndexFromPrefix(prefix);

      if (wheel < 0)
      {
        continue;  // unrecognised wheel id, skip it
      }

      int multiplier = (sign_char == 'p') ? 1 : -1;
      double value = std::stod(token.substr(3));

      velocity_states_.at(wheel) = multiplier * value;
      position_states_.at(wheel) += velocity_states_.at(wheel) * dt;
    }

    last_run_ = rclcpp::Clock().now();
  }
  return hardware_interface::return_type::OK;
}

// Sends one token per wheel, comma separated, same format as read() above.
hardware_interface::return_type MecanumInterface::write(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  std::stringstream message_stream;
  message_stream << std::fixed << std::setprecision(2);

  for (std::size_t i = 0; i < NUM_WHEELS; i++)
  {
    double cmd = velocity_commands_.at(i);
    char sign = (cmd >= 0) ? 'p' : 'n';
    double magnitude = std::abs(cmd);
    std::string zero_pad = (magnitude < 10.0) ? "0" : "";

    message_stream << wheelPrefix(i) << sign << zero_pad << magnitude << ",";
  }

  try
  {
    arduino_.Write(message_stream.str());
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MecanumInterface"),
                        "Something went wrong while sending the message "
                            << message_stream.str() << " to the port " << port_);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}
}  // namespace mecanum_firmware

PLUGINLIB_EXPORT_CLASS(mecanum_firmware::MecanumInterface, hardware_interface::SystemInterface)