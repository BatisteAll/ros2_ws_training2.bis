///////////////////////////////////////////////////////////////////////
//          CUSTOM HARDWARE INTERFACE - HEADERS
// Hardware interface NAME: dof6bot_hardware_interface
// Hardware interface NAMESPACE: custom_hardware_armbot
// Hardware interface DESCRIPTION:
//      - interface to control a 6DOF robot under ISAAC SIM
///////////////////////////////////////////////////////////////////////


////////////////////////
//    HEADER GUARDS   //
////////////////////////
//(prevent multiple inclusion - https://en.wikipedia.org/wiki/Include_guard)
#ifndef CUSTOM_ARMBOT_HARDWARE__DOF6BOT_HARDWARE_INTERFACE_HPP_
#define CUSTOM_ARMBOT_HARDWARE__DOF6BOT_HARDWARE_INTERFACE_HPP_


////////////////////////
//      INCLUDES      //
////////////////////////
// include tools
#include "string"
#include "unordered_map"
#include "vector"
// include hardware interface basic knowledge
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
// include ROS2 and ROS2 lifecycle node
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

//Namespace for understanding purposes
namespace custom_armbot_hardware
{

    // Define the term "CallbackReturn" as the call to ROS2 Lifecycle NODE callback funtion
    // using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    ////////////////////////
    //    CLASS SYSTEM    //
    ////////////////////////
    // --> the class extends one of the 3 interface types: system (multi dof), actuator (1 dof), sensor
    // e.g. here the robot is a multi dof system
    class  DOF6BOTSystem : public hardware_interface::SystemInterface
    {
        public:

        // Macro needed to use the custom hardware interface with shared pointers
        RCLCPP_SHARED_PTR_DEFINITIONS(DOF6BOTSystem)


        // Functions defining the interaction with ROS to check that hardware params are properly set

        // Init function called after plugin is loaded
        // --> provides hardware URDF configuration using the `HardwareInfo` structure
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;

        // Configure function, called after initialization
        // --> configure all state interfaces and "non-movement" command interfaces for them to be available to controllers
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

        // Defines the state interfaces
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        // Defines the command interfaces
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        // activate function 
        // --> performs final preparations to start executing e.g. acquiring access to hardware
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

        // deactivate function
        // --> cleanup and reverse the onActivate changes
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

        // Defines how to read data from hardware
        hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

        // Defines how to write controller command to hardware
        hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;


        private:
        // The size of this vector is (standard_interfaces_.size() x nr_joints)
        // These vectors store hardware controller commands and states information
        std::vector<double> joint_position_command_;
        std::vector<double> joint_velocities_command_;
        std::vector<double> joint_position_;
        std::vector<double> joint_velocities_;
        std::vector<double> ft_states_;
        std::vector<double> ft_command_;
    };

}  // namespace custom_armbot_hardware

#endif  // CUSTOM_ARMBOT_HARDWARE__DOF6BOT_HARDWARE_HPP_ : closure of the header guard