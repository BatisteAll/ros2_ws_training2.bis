# === Tutorial ROS2 - 2.BIS ===

## ---= ISAAC SIM Rviz Training =---

### --- Prerequisites ---


### --- Build Phase Commands ---
    colcon build  
    source ./install/setup.bash   

### --- Operation Commands ---
#### --- 1 --- Launch Robot, Controllers, Simulation & Visualization ---
    ros2 launch isaacsim_rviz_training slrobot.launch.py
    ros2 launch isaacsim_rviz_training command_slrobot.launch.py goal_prim:=robot target_pose:=pick_far goal_exec_time:=6

    Launch step by step:
        [1]
        ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$( xacro $HOME/workspaces/ros2_ws_training2.bis/src/isaacsim_rviz_training/description/urdf/spacelab_robot.urdf.xacro )"
        [2]
        ros2 run controller_manager ros2_control_node robot_description:=$HOME/workspaces/ros2_ws_training2.bis/src/isaacsim_rviz_training/description/urdf/spacelab_robot.urdf --ros-args --params-file $HOME/workspaces/ros2_ws_training2.bis/src/isaacsim_rviz_training/bringup/config/slrobot_ros_params.yaml
        [3]
        ros2 run controller_manager spawner joint_state_broadcaster
        [4]
        ros2 run controller_manager spawner joint_trajectory_controller

    ros2 topic pub /joint_states sensor_msgs/msg/JointState "{name: ['joint_1_2', 'joint_2_3'], position: [0.5, 0.4], velocity: [1.0, 1.2], effort: [0.0, 0.0]}"
    ros2 topic pub /joint_commands sensor_msgs/msg/JointState "{name: ['joint_EE_Lgripper'], position: [0.02], velocity: [1.0], effort: [0.0]}"


#### --- XXX --- Debug ---
    ros2 control list_hardware_interfaces



## --------======== ISAAC Sim Rviz Training Code Description ========---------

### --- SUMMARY ---
* 1. start_world.launch.py is launched, it instentiates the gazebo world with the workbenches from a gazebo .world
* 2. spawn_robot_ros2.launch.xml is launched, it launches two other launch files:
    * 2bis. publish_urdf.launch.py launches the robot_state_publisher node (which is an independent downloaded library) to publish the urdf on the /robot_description topic and start xacro to parse the .xacro files:
        * ros2_training.urdf.xacro creates the urdf model of the robot by gathering, in order, the different xacro files:
            * robot_core.xacro defines the main joints and links of the robot
            * lidar, camera and gripper.xacro define the specific joint, links and sensors for the respective components
            * robot_gazebo.xacro defines the gazebo "references"=parameters for every joint/links and instentiates the joint_state_publisher topic thanks to the gazebo plug-in
            * ros2_control.xacro defines the gazebo_ros2_control plugin as controller for the simulation in gazebo (with parameters gotten from my_controllers.yaml) and the command/state interfaces for every joints
    * 2ter. spawn_robot_description.launch.py launches the robot thanks to spawn_entity.py from gazebo_ros package & activate the different controllers:
        * joint_trajectory_controller: with the ros2_control node 'load_controller'
        * joint_state_controller: with the ros2_control node 'load_controller'
        * grasping controller: by running grasping_controller.py
* 3. init_rviz.launch.py initializes rviz node using the associated .rviz config file & launches the import3D nodes scripts used to generate markers (3D shapes) in the rviz scene
* 4. Move the robot thanks to my_pose_name.launch.py which request the action node command_robot.py with a pose argument:
    * the node creates an action client that communicates the pose argument to FollowJointTrajectory server
    * FollowJointTrajectory server performs the interpolation
    * the computed traj is sent/implemented with the robot through the FollowJointTrajectory blackbox
* 5. Open/close the gripper thanks to gripper_open.launch.py/close_gripper.launch.py:
    * close_gripper.launch.py:
        * load a configuration (part to grasp + position + mode) to the lifecycle node grasping_controller.py
            * the get_state parameter can be set to two modes:
                * true : the part is forced to its current position in the gripper frame
                * false : the part is forced to the specified position in argument (e.g. at the center of the gripper)
        * calls the "configure" method of the /grasping_lifecycle_node, which define params (pose, mode, frame, part)
        * calls the "activate" method of the /grasping_lifecycle_node, which creates a subscriber to the contact sensor of the gripper left finger and calls a method that forces the part position whenever a contact is published
    * open_gripper.launch.py:
        * calls the "deactivate" method of the /grasping_lifecycle_node, which destroys the subscriber
        * calls the "cleanup" method of the /grasping_lifecycle_node, wich clear all states and return the node to a functionally equivalent state as when first created
        

### --- DEPENDENCIES ---

#### package.xml
Package.xml is responsible for:
* Ordering of, the configure step (cmake) sequence for catkin-packages in catkin workspaces
* Define packaging dependencies for bloom (what dependencies to export when creating debian pkgs)
* Define system (non-catkin-pkgs) build dependencies for rosdep
* Document build or install or runtime dependency for roswiki / graph tool (rqt_graph) 

#### CMakeLists.txt
In general, CMakeLists.txt is responsible for preparing and executing the build process.
The name of the system library and the name to be used to find the package are often not the same, that's a reason why we have to specify that dependency both in the Package.xml and the CMakeLists.txt. 

Note: to add a dependency to a package (and a script) you need to:
* add `<depend>my_dependent_pkg</depend>` to the package .xml
* add `find_package(my_dependent_pkg REQUIRED)` to the find package section of the CMakeLists.txt
* add `"my_dependent_pkg"` to the ament_target_dependencies() section of the CMakeLists.txt
* add `#include <my_dependent_pkg/my_dependent_pkg.h>` as header to your my_script.cpp

### --- MODELS ---

#### spacelab_robot.urdf.xacro
* defines the model of the whole robot
* main urdf file, parsing each independent urdf sub files

#### robot_core.xacro
* defines the core elements of the robot (joints, links, materials,)
* geometries can be defined thanks to meshes through absolute or relative paths:
    * e.g. absolute : `<mesh filename="file:///home/username/ros2_ws_training/src/gazebo_rviz_training/models/meshes/Base.dae"/>`
    * e.g. relative : `<mesh filename="package://gazebo_rviz_training/models/meshes/Base.dae"/>`
* Convention for joint and link name is: 
    * the link with ID x is named link_x
    * the joint holding link_x (as parent) and link_y (as child) is named joint_x.y in this order

#### ros2_control.xacro
* defines the hardwer components: the robot arm which is a "system" (multi DOF) and the gripper which is an "actuator" (one DOF, one joint)
* NOTE: a `hardware_component` is a library loaded by the controller manager using the pluginlib interface, to create a custom hardware component:
    *https://youtu.be/be-5DPuDtO8
    *https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html
    *https://control.ros.org/master/doc/ros2_control_demos/example_7/doc/userdoc.html
    *https://github.com/ros-controls/ros2_control_demos/tree/master/example_7
* attributes the additional "ros2 control specific" parameters to the existing links/joints defined in the core urdf
* defines the command/state interfaces for every joints (position, velocity, force, ...)
Note: to get a full training on ros2 control : https://github.com/ros-controls/roscon2022_workshop

#### gripper.xacro
* defines the joints/links of the end effector, the gripper, and some parameters to be tuned
* convention for joints and linked is kept but without number e.g. "link_EE" for link End Effector
* the inertial macro is included and defines inertia terms for simple shapes with their size in input
* a mimic command is applied to the right finger to copy the movements of the left finger like a mirror
* a contact sensor is defined on the left finger, and the appropriate gazebo plug-in for it is defined

#### camera.xacro
* defines the joints/links of the camera
* convention for joints and linked is kept but without number e.g. "link_cam3D"
* two non-geometrical links are defined to instentiate the sensor and its focale (the camera frame seen in gazebo)
* a camera sensor is defined, as for the appropriate gazebo plug-in

#### lidar.xacro
* defines the joints/links of the lidar
* convention for joints and linked is kept but without number e.g. "link_lidar"
* one non-geometrical link is defined to instentiate the focale=laser of the lidar (the lidar pyramidal fov seen in gazebo)
* a scan3D sensor is defined, as for the appropriate gazebo plug-in from the .so file

#### inertial_macros.xacro
* the inertial macro defines inertia terms for simple shapes based on their size in input

#### /meshes
* it is a folder gathering the meshes .dae files used to get the CAD shape in the URDF


### --- CONFIG ---

#### my_controllers.yaml
* defines the parameters of the controllers (ROS2 control)
* this file is called in ros2_control.xacro
* list of the controllers:
    * joint_state_broadcaster:
        * controller that broadcasts at each time step the states of the joints specified in the config file
        * the broadcaster reads all state interfaces and reports them on the topics /joint_states and /dynamic_joint_states
        * the states are defined by the state interfaces in the config file (here position & velocity)
        * Broadcasters are not real controllers, and therefore take no commands
        * https://control.ros.org/master/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html
    * joint_trajectory_controller:
        * controller that publishes commands to the joints
        * the commands are defined by the command interfaces in the command file (here position)
        * Controller for executing joint-space trajectories on a group of joints. The controller interpolates in time between the points so that their distance can be arbitrary. Even trajectories with only one point are accepted. Trajectories are specified as a set of waypoints to be reached at specific time instants, which the controller attempts to execute as well as the mechanism allows. Waypoints consist of positions, and optionally velocities and accelerations.
        * https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html
            * ~/joint_trajectory (input topic) [trajectory_msgs::msg::JointTrajectory] : Topic for commanding the controller.
            * ~/state (output topic) [control_msgs::msg::JointTrajectoryControllerState] : Topic publishing internal states.
            * ~/follow_joint_trajectory (action server) [control_msgs::action::FollowJointTrajectory] : Action server for commanding the controller.



### --- LAUNCH ---

#### sl_robot.launch.py
* TIPS: the parameters section of the controller manager initialization needs to be instentiated as follows, else it won't init properly
    *   WRONG `parameters=[{'robot_description': robot_description},controller_params_file]`
    * CORRECT `parameters=[robot_description,controller_params_file]`

#### command_slrobot.launch.py
* TIPS: variables of type DeclareLaunchArgument and LaunchConfiguration can't be used as a python variable, 
        they can simply be passed, as ROS2 first build the context and then executes the nodes. With the 
        OpaqueFunction() one can delay the execution of code, so that the context exists and the value can be retrieved. 
    * https://docs.openvins.com/gs-tutorial.html#gs-tutorial-ros2
    * https://robotics.stackexchange.com/questions/104340/getting-the-value-of-launchargument-inside-python-launch-file
    *   WRONG `goal_prim = LaunchConfiguration('prim')`
    * CORRECT `goal_prim= LaunchConfiguration('prim').perform(context)`


### --- SCRIPT ---

#### command_slrobot.py
* create an action client, from rclpy library, that will communicate with FollowJointTrajectory server from control_msgs.action library
* the output topic on wich the interpolated trajectory (set of waypoints) is published is:
    * TOPIC: /joint_trajectory_controller/controller_state 
    * MSG TYPE: control_msgs/msg/JointTrajectoryControllerState
    * ECHO: ros2 topic echo /joint_trajectory_controller/controller_state control_msgs/msg/JointTrajectoryControllerState --field reference
* this node is related to an action server, if the feedback needs to be printed, then the node needs to be spinning
    * to know more about the spin func: https://docs.ros2.org/foxy/api/rclpy/api/init_shutdown.html

#### controlMsg2jointMsg_pubSub.py
* Convert /joint_trajectory_controller/controller_state to sensor_msgs/msg/JointState
* https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
* It can happen that, when the callback function of the subscriber is called, self variable is populated with the subscribed_msg:
    * self.subscriber_callback_method is a bound method - it already contains a reference to the scope that 
        it is bound to (saved in the __self__ variable). So it is never needed to provide the self argument, 
        regardless of where the method is called.
    * https://robotics.stackexchange.com/questions/101565/dynamically-create-subcription-callback-functions-python
    * https://stackoverflow.com/questions/70511324/typeerror-missing-one-required-positional-argument-event
    * solution: hard set the self variable `setattr(self, 'listener2writer_callback', self.listener2writer_callback.__get__(self, self.__class__))`
* send the goal feedback to the actual joint trajectory feedback topic:
    * to find the hidden topic: `ros2 topic list --include-hidden-topics -t`
    * to list the publisher/subscriber of this topic `ros2 topic info --verbose /topic_name`

### --- rviz ---

#### example.rviz
* it is a rviz generated file, saving a certain configuration of the window layout and the rviz parameters


### --- world ---

#### example.world
* it is a gazebo generated file, saving a certain configuration of the gazebo environment
* the gazebo example.world defines, for each gazebo entity, the path to its mesh, some other properties as the position, and its name wich can be used as an identifier
* the meshes are stored in the model>gazebo folder



## --------======== CUSTOM ROS2 CONTROL HARDWARE INTERFACE - TRAINING ========---------

### --- FULL TUTORIALs ---
https://github.com/ros-controls/ros_control/wiki/hardware_interface
http://docs.ros.org/en/noetic/api/hardware_interface/html/c++/index.html


### --- EXPLAINATION ---
For details on each hardware interface type check `Hardware Components description https://control.ros.org/master/doc/getting_started/getting_started.html#hardware-components`.

#### Lifecycle of Hardware Components

##### Definitions and Nomenclature
Hardware interfaces use the lifecycle state machine `defined for ROS2 nodes <https://design.ros2.org/articles/node_lifecycle.html>`_.
There is only one addition to the state machine, that is the initialization method providing hardware configuration from URDF file as argument.

* Hardware Interface
    Hardware interfaces are used by ROS control in conjunction with one of the available ROS controllers to send (hardware_interface::RobotHW::write) commands to the hardware and receive (hardware_interface::RobotHW::read) states from the robot's resources (joints, sensors, actuators).
    Class Definition: https://control.ros.org/master/doc/api/namespacehardware__interface.html#af127ad1f7288042a450a4e783e44cf5c
    Class Explanation: http://docs.ros.org/en/noetic/api/hardware_interface/html/c++/index.html


* Hardware Components
    Wrapper and abstraction of hardware interface to manage life cycle and access to methods of hardware interface from Resource Manager.

* Resource Manager
     Class responsible for the management of hardware components in the ros2_control framework.

* "movement" command interfaces
    Interfaces responsible for robot to move, i.e., influence its dynamic behavior.
    The interfaces are defined in `hardware_interface_type_values.hpp <https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/types/hardware_interface_type_values.hpp>`.

* "non-movement" command interfaces
     All other interfaces that are not "movement" command interfaces


##### Initialization
Immediately after a plugin is loaded and the object created with the default constructor, the ``on_init`` method will be called providing hardware URDF configuration using the ``HardwareInfo`` structure.
In this stage, all of the needed memory should be initialized and storage prepared for interfaces.
The resource manager will export all interfaces after this and store them internally.


##### Configuration
Precondition is hardware interface state having id: ``lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED``.
After configuration, the ``read`` and ``write`` methods will be called in the update loop.
This means all internal state and commands variables have to be initialized.
After a successful call to ``on_configure``, all state interfaces and "non-movement" command interfaces should be available to controllers.

NOTE: If using "non-movement" command interfaces to parametrize the robot in the ``lifecycle_msgs::msg::State::PRIMARY_STATE_CONFIGURED`` state make sure to take care about current state in the ``write`` method of your Hardware Interface implementation.


### --- DEPENDENCIES ---

#### package.xml
Package.xml is responsible for:
* Ordering of, the configure step (cmake) sequence for catkin-packages in catkin workspaces
* Define packaging dependencies for bloom (what dependencies to export when creating debian pkgs)
* Define system (non-catkin-pkgs) build dependencies for rosdep
* Document build or install or runtime dependency for roswiki / graph tool (rqt_graph) 

#### CMakeLists.txt
In general, CMakeLists.txt is responsible for preparing and executing the build process.
The name of the system library and the name to be used to find the package are often not the same, that's a reason why we have to specify that dependency both in the Package.xml and the CMakeLists.txt. 
See the following links to have information on how to export xml files (pluginlib_export_plugin_description_file):
https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html
https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Pluginlib.html


### --- SCRIPT ---

#### dof6bot_hardware.cpp
* METHOD on_init():
    * The on_init method is called once during ros2_control initialization if the RobotSystem was specified in the URDF.
    * In this method, communication between the robot hardware needs to be setup and memory dynamic should be allocated. 
    * For simulated robots, explicit communication will not be established. Instead, vectors will be initialized that represent the state all the hardware, e.g. a vector of doubles describing joint angles, etc.


## --------======== TIPS Section ========---------

### Setup your package structure
https://rtw.stoglrobotics.de/master/guidelines/robot_package_structure.html