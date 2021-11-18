#ifndef LOAD_RUNNER_DUMMY_HARDWARE_INTERFACE__DUMMY_ROBOT_SYSTEM_HPP_
#define LOAD_RUNNER_DUMMY_HARDWARE_INTERFACE__DUMMY_ROBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "load_runner_dummy_hardware_interface/visibility_control.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace load_runner_dummy_hardware_interface
{
class DummyRobotSystemHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DummyRobotSystemHardware)

    LOAD_RUNNER_DUMMY_HARDWARE_INTERFACE_PUBLIC
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    LOAD_RUNNER_DUMMY_HARDWARE_INTERFACE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    LOAD_RUNNER_DUMMY_HARDWARE_INTERFACE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    LOAD_RUNNER_DUMMY_HARDWARE_INTERFACE_PUBLIC
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    LOAD_RUNNER_DUMMY_HARDWARE_INTERFACE_PUBLIC
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    LOAD_RUNNER_DUMMY_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type read() override;

    LOAD_RUNNER_DUMMY_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type write() override;

private:
    std::vector<double> hw_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;

};

}  // namespace load_runner_dummy_hardware_interface
#endif  // LOAD_RUNNER_DUMMY_HARDWARE_INTERFACE__DUMMY_ROBOT_SYSTEM_HPP_