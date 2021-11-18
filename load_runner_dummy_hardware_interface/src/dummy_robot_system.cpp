#include "load_runner_dummy_hardware_interface/dummy_robot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace load_runner_dummy_hardware_interface
{
CallbackReturn DummyRobotSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
    // URDF에서 설정한 값을 읽어옴.
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    // URDF에서 설정한 파라미터 값을 받을 수 있음.
    auto user_parameter = info_.hardware_parameters["parameter1"];

    // 조인트의 수에 맞게 각 변수들 사이즈 변경 및 초기화
    hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    //사용자의 하드웨어에 따라 통신 등에 대한 초기화 과정을 이곳에 구현하면 됨.


    // 초기화 과정 끝.
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DummyRobotSystemHardware::export_state_interfaces()
{
    // 피드백 인터페이스 설정
    // 각 조인트마다 Position, Velocity 값을 피드백 하므로 다음과 같이 설정
    // 각 조인트의 피드백 값을 멤버변수 hw_position, hw_velocity에 맵핑한다.

    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DummyRobotSystemHardware::export_command_interfaces()
{
    // 코맨드 인터페이스 설정
    // JointA, JointB는 포지션 명령, JointC는 속도 명령을 받아야 함.
    // info_.joints[i].command_interfaces[0].name을 이용하면 속도/포지션 명령 인터페이스 구분 가능
    // 각 조인트의 코맨드 값을 멤버변수 hw_commands에 맵핑 가능

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
        if(info_.joints[i].command_interfaces[0].name == "position")
        {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]
                )
            );
        }
        else if(info_.joints[i].command_interfaces[0].name == "velocity")
        {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]
                )
            );
        }
    }

    return command_interfaces;
}

CallbackReturn DummyRobotSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("DummyRobotSystemHardware"), "Starting ...please wait...");

    // 초기화 과정을 완료하고, 노드가 시작되는 단계
    // 로봇의 시작 시퀀스를 이곳에 구현하면 됨.

    RCLCPP_INFO(rclcpp::get_logger("DummyRobotSystemHardware"), "System Successfully started!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn DummyRobotSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("DummyRobotSystemHardware"), "Stopping ...please wait...");

    // 동작 중 노드가 정지하는 단계
    // 로봇의 종료 시퀀스를 이곳에 구현하면 됨.

    RCLCPP_INFO(rclcpp::get_logger("DummyRobotSystemHardware"), "System successfully stopped!");
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DummyRobotSystemHardware::read()
{
    RCLCPP_INFO(rclcpp::get_logger("DummyRobotSystemHardware"), "Reading...");
    // 로봇의 상태를 읽어오는 단계
    // 각 조인트의 현재 값을 읽어오고 그 결과를 멤버변수 hw_positions_, hw_velocities_에 저장한다.

    hw_positions_[0] = 0.0;
    hw_positions_[1] = 0.1;
    hw_positions_[2] = 0.2;

    hw_velocities_[0] = 0.0;
    hw_velocities_[1] = -0.1;
    hw_velocities_[2] = -0.2;

    RCLCPP_INFO(rclcpp::get_logger("DummyRobotSystemHardware"), "Joints successfully read!");
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type load_runner_dummy_hardware_interface::DummyRobotSystemHardware::write()
{
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Writing...");

    // 컨트롤러부터 전달된 명령 지령 값을 로봇에 전달하는 단계
    // hw_commands_에 명령 지령값이 저장되어 있으므로 이를 이용하여 로봇 하드웨어에 전달한다.

    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "hw_commands[0] = %f...", hw_commands_[0]);
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "hw_commands[1] = %f...", hw_commands_[1]);
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "hw_commands[2] = %f...", hw_commands_[2]);

    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Joints successfully written!");
    return hardware_interface::return_type::OK;
}

}  // namespace load_runner_dummy_hardware_interface
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(load_runner_dummy_hardware_interface::DummyRobotSystemHardware, hardware_interface::SystemInterface)