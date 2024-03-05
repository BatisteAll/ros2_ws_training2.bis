///////////////////////////////////////////////////////////////////////
//          CUSTOM HARDWARE INTERFACE - IMPLEMENTATION
// Hardware interface NAME: dof6bot_hardware_interface
// Hardware interface NAMESPACE: custom_hardware_armbot
// Hardware interface DESCRIPTION:
//      - interface to control a 6DOF robot under ISAAC SIM
///////////////////////////////////////////////////////////////////////


////////////////////////
//      INCLUDES      //
////////////////////////
// include necessary dependencies
#include "isaacsim_rviz_training/dof6bot_hardware_interface.hpp"
#include <string>
#include <vector>


//Namespace for understanding purposes
namespace isaacsim_rviz_training
{


    // METHOD on_init: initialization of all member variables and process parameters
    // from "info" argument to see if every parameter is valid else ERROR
    //---------------------------------------------------------------------------------------
    CallbackReturn DOF6BOTSystem::on_init(const hardware_interface::HardwareInfo & info)
    {
        rclcpp::Logger logger = rclcpp::get_logger("test_logger");
        RCLCPP_INFO(logger, "Logging test");

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

        return CallbackReturn::SUCCESS;
    }
    //---------------------------------------------------------------------------------------


    // // METHOD on_configure: setup the communication to the hardware and set everything up
    // // so that the hardware can be activated;
    // //---------------------------------------------------------------------------------------
    // CallbackReturn DOF6BOTSystem::on_configure(const rclcpp_lifecycle::State &)
    // {
    //     // This is already done in the on_init
    //     // for (uint i = 0; i < joint_position_.size(); i++)
    //     // {
    //     //     joint_position_[i] = 0;
    //     //     joint_velocities_[i] = 0;;
    //     //     joint_position_command_[i] = 0;
    //     //     joint_velocities_command_[i] = 0;
    //     //     ft_states_[i] = 0;
    //     //     ft_command_[i] = 0;
    //     // }

    //     RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"),"Successfully configured!");



    //     return hardware_interface::CallbackReturn::SUCCESS;
    // }
    // //---------------------------------------------------------------------------------------



    // // METHOD on_activate: executed when the hardware is transitionned to the active state reade to be executed
    // //---------------------------------------------------------------------------------------
    // hardware_interface::CallbackReturn DOF6BOTSystem::on_activate(const rclcpp_lifecycle::State &)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"),"Successfully activated!");

    //     return hardware_interface::CallbackReturn::SUCCESS;
    // }
    // //---------------------------------------------------------------------------------------


    // // METHOD on_deactivate: reverse on activate changes and cleanUp
    // //---------------------------------------------------------------------------------------
    // hardware_interface::CallbackReturn DOF6BOTSystem::on_deactivate(const rclcpp_lifecycle::State &)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"),"Successfully deactivated!");

    //     return hardware_interface::CallbackReturn::SUCCESS;
    // }
    // //---------------------------------------------------------------------------------------


    // METHOD export_state_interfaces: export a struct type vector containing joint names, IF types, current positions and velocities
    //---------------------------------------------------------------------------------------
    std::vector<hardware_interface::StateInterface> DOF6BOTSystem::export_state_interfaces()
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

        for (uint i = 0; i < info_.joints.size(); i++)
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
    std::vector<hardware_interface::CommandInterface> DOF6BOTSystem::export_command_interfaces()
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
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_command_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_command_[i]));
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
    return_type DOF6BOTSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
    {
        for (auto i = 0ul; i < joint_velocities_command_.size(); i++)
        {
            joint_velocities_[i] = joint_velocities_command_[i];
            joint_position_[i] += joint_velocities_command_[i] * period.seconds();
        }
/*time*/
        for (auto i = 0ul; i < joint_position_command_.size(); i++)
        {
            joint_position_[i] = joint_position_command_[i];
        }

        return return_type::OK;
    }
    //---------------------------------------------------------------------------------------



    // METHOD write: 
    //---------------------------------------------------------------------------------------
    return_type DOF6BOTSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        return return_type::OK;
    }
    //---------------------------------------------------------------------------------------



}  // namespace isaacsim_rviz_training



////////////////////////
//     PLUGINLIB      //
////////////////////////
// include "PLUGINLIB_EXPORT_CLASS" macro to tell pluginlib that we are exporting this custom hardware class as ROS2 plugin
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    isaacsim_rviz_training::DOF6BOTSystem, hardware_interface::SystemInterface)
