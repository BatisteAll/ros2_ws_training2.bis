///////////////////////////////////////////////////////////////////////
//          CUSTOM HARDWARE INTERFACE - IMPLEMENTATION
// Hardware interface NAME: dof6bot_hardware
// Hardware interface NAMESPACE: custom_hardware_armbot
// Hardware interface DESCRIPTION:
//      - interface to control a 6DOF robot under ISAAC SIM
///////////////////////////////////////////////////////////////////////


////////////////////////
//      INCLUDES      //
////////////////////////
// include necessary dependencies
#include "custom_armbot_hardware/dof6bot_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <string>
#include <vector>

//Namespace for understanding purposes
namespace custom_hardware_armbot
{


    // METHOD on_init: initialization of all member variables and process parameters
    // from "info" argument to see if every parameter is valid else ERROR
    //---------------------------------------------------------------------------------------
    hardware_interface::CallbackReturn dof6botSystem::on_init(const hardware_interface::HardwareInfo &)
    {

        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        // Set of state and command vectors (robot has 6 joints and 2 interfaces)
        joint_position_.assign(6, 0);
        joint_velocities_.assign(6, 0);
        joint_position_command_.assign(6, 0);
        joint_velocities_command_.assign(6, 0);
        // force sensor has 6 readings
        ft_states_.assign(6, 0);
        ft_command_.assign(6, 0);

        for (const auto & joint : info_.joints)
        {
            for (const auto & interface : joint.state_interfaces)
            {
            joint_interfaces[interface.name].push_back(joint.name);
            }
        }

        return CallbackReturn::SUCCESS;
    }
    //---------------------------------------------------------------------------------------


    // METHOD on_configure: setup the communication to the hardware and set everything up
    // so that the hardware can be activated;
    //---------------------------------------------------------------------------------------
    hardware_interface::CallbackReturn dof6botSystem::on_configure(const rclcpp_lifecycle::State &)
    {
        // This is already done in the on_init
        // for (uint i = 0; i < joint_position_.size(); i++)
        // {
        //     joint_position_[i] = 0;
        //     joint_velocities_[i] = 0;;
        //     joint_position_command_[i] = 0;
        //     joint_velocities_command_[i] = 0;
        //     ft_states_[i] = 0;
        //     ft_command_[i] = 0;
        // }

        RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"),"Successfully configured!")



        return hardware_interface::CallbackReturn::SUCCESS;
    }
    //---------------------------------------------------------------------------------------



    // METHOD on_activate: executed when the hardware is transitionned to the active state reade to be executed
    //---------------------------------------------------------------------------------------
    hardware_interface::CallbackReturn dof6botSystem::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"),"Successfully activated!")

        return hardware_interface::CallbackReturn::SUCCESS;
    }
    //---------------------------------------------------------------------------------------


    // METHOD on_deactivate: reverse on activate changes and cleanUp
    //---------------------------------------------------------------------------------------
    hardware_interface::CallbackReturn dof6botSystem::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"),"Successfully deactivated!")

        return hardware_interface::CallbackReturn::SUCCESS;
    }
    //---------------------------------------------------------------------------------------


    // METHOD export_state_interfaces: export a struct type vector containing joint names, IF types, current positions and velocities
    //---------------------------------------------------------------------------------------
    std::vector<hardware_interface::StateInterface> dof6botSystem::export_state_interfaces()
    {
        // defines the "state_interfaces" struct type variable which contains joint names, types of IF, actual state
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // // fill "state_interfaces" with the joints positions
        // int ind = 0;
        // for (const auto & joint_name : joint_interfaces["position"])
        // {
        //     state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
        // }

        // // fill "state_interfaces" with the joints velocities
        // ind = 0;
        // for (const auto & joint_name : joint_interfaces["velocity"])
        // {
        //     state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
        // }

        for (uint i= 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
        }

        // fill "state_interfaces" with the force torque sensor data
        state_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_states_[0]);
        state_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_states_[1]);
        state_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_states_[2]);
        state_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_states_[3]);
        state_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_states_[4]);
        state_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_states_[5]);

        return state_interfaces;
    }
    //---------------------------------------------------------------------------------------

    
    // METHOD export_command_interfaces: export a struct type vector containing joint names, IF types, current positions and velocities
    //---------------------------------------------------------------------------------------
    std::vector<hardware_interface::CommandInterface> dof6botSystem::export_command_interfaces()
    {
        // defines the "command_interfaces" struct type variable which contains joint names, target states (positions, velocities)
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        // // fill "command_interfaces" with the joints target positions
        // int ind = 0;
        // for (const auto & joint_name : joint_interfaces["position"])
        // {
        //     command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
        // }

        // // fill "command_interfaces" with the joints target velocities
        // ind = 0;
        // for (const auto & joint_name : joint_interfaces["velocity"])
        // {
        //     command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
        // }

        for (uint i= 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_command_[i]));
            state_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_command_[i]));
        }

        // fill "command_interfaces" with the target forces &  torques
        command_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_command_[0]);
        command_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_command_[1]);
        command_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_command_[2]);
        command_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_command_[3]);
        command_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_command_[4]);
        command_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_command_[5]);

        return command_interfaces;
    }
    //---------------------------------------------------------------------------------------


    // METHOD read: 
    //---------------------------------------------------------------------------------------
    hardware_interface::return_type dof6botSystem::read(const rclcpp::Time & time, const rclcpp::Duration &)
    {
        // TODO(pac48) set sensor_states_ values from subscriber

        for (auto i = 0ul; i < joint_velocities_command_.size(); i++)
        {
            joint_velocities_[i] = joint_velocities_command_[i];
            joint_position_[i] += joint_velocities_command_[i] * period.seconds();
        }

        for (auto i = 0ul; i < joint_position_command_.size(); i++)
        {
            joint_position_[i] = joint_position_command_[i];
        }

        return hardware_interface::return_type::OK;
    }
    //---------------------------------------------------------------------------------------



    // METHOD write: 
    //---------------------------------------------------------------------------------------
    hardware_interface::return_type dof6botSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        return hardware_interface::return_type::OK;
    }
    //---------------------------------------------------------------------------------------



}  // namespace custom_armbot_hardware



////////////////////////
//     PLUGINLIB      //
////////////////////////
// include "PLUGINLIB_EXPORT_CLASS" macro to tell pluginlib that we are exporting this custom hardware class as ROS2 plugin
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(custom_hardware_armbot::dof6botSystem, hardware_interface::SystemInterface)
