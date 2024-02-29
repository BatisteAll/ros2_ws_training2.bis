# === Tutorial ROS2 - 2.BIS ===

## ---= ISAAC SIM Rviz Training =---

### --- Prerequisites ---


### --- Build Phase Commands ---
    colcon build  
    source ./install/setup.bash   

### --- Operation Commands ---
#### --- 1 --- Launch Gazebo with a the "exemple.world" ---
    ros2 launch gazebo_rviz_training start_world.launch.py  





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

#### ros2_training.urdf.xacro
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

#### /gazebo
* it is a folder generated by gazebo with the information of the world, the meshes of every part, ...

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


### --- LAUNCH ---

#### start_world.launch.py
* first launch file to be executed to launch the gazebo world with the workbenches
* tool to get packages paths : get_package_share_directory(my_pkg_name)
* from the launch library:
    * PythonLaunchDescriptionSource(): identifies a path that links specifically to a launch file
    * IncludeLaunchDescription() : launches a launch file using a PythonLaunchDescriptionSource (e.g. from a launch file)

#### spawn_robot_ros2.launch.xml
* second launch file to be executed after the gazebo worl is initiated
* this is the example of a launch file written in xml format (but could have been written in .py the same way)
* this file launches two other launch files:
    * publish_urdf.launch.py: publishes the urdf file in the /robot_description topic
    * spawn_robot_description.launch.py: launches the robot in the gazebo world
* the syntax of xml launch file substitutions can be found here: https://design.ros2.org/articles/roslaunch_xml.html

#### publish_urdf.launch.py
* launches the robot_state_publisher node to publish the urdf on the /robot_description topic
* launches xacro to parse the urdf file

#### spawn_robot_description.launch.py
* launches the robot thanks to spawn_entity.py from gazebo_ros package & run the different controllers
* spawn_entity.py requires some arguments like:
    * the robot name
    * the robot position and orientation
    * the topic from which the urdf is gathered
* the controllers instentiated are:
    * joint_trajectory_controller
    * joint_state_controller
    * grasping controller
* event handlers like RegisterEventHandler() are used to sequence and make sure one action is executed at a time (e.g. the configuration is loaded only once the node instentiated)

#### init_rviz.launch.py
* initializes rviz node using the associated config file
* launches the import3D files used to generate markers (3D shapes) in the rviz scene

#### my_pose_name.launch.py
* launch the command_robot.py node with specific position parameters to move the robot to a defined position

#### gripper_close.launch.py
* launch the command_gripper.py node to create the action client and tell the FollowJointTrajectory server to close fingers, it also activates the grasping_controller.py node called 'gripper_lifecycle_node' to force the position of the part
* launch file arguments are defined using: `my_arg1 = DeclareLaunchArgument('my_argName', default_value='my_argDedault')`
* the activated 'gripper_lifecycle_node' is a lifecycle node, as explained: https://design.ros2.org/articles/node_lifecycle.html
* two lifecycle modes are used here:
    * configure: allow the node to load its configuration and conduct any required setup.The configuration of a node will typically involve those tasks that must be performed once during the node’s life time, such as obtaining permanent memory buffers and setting up topic publications/subscriptions that do not change.
    * activate: do any final preparations to start executing. This may include acquiring resources that are only held while the node is actually active, such as access to hardware.
* some parameters are loaded to configure 'gripper_lifecycle_node' using the command: ros2 param load /my_node my_node.yaml
* the 'gripper_lifecycle_node' can be configured in three ways to close the gripper based on the argument 'config':
    * Grasp_Template(Grasp_Template.yaml): grasp PART and forces it to the inputed pose
    * PART (Grasp_PART.yaml): grasp PART and forces it to its current location in the gripper reference frame
    * PART_clone (Grasp_PART_clone.yaml): grasp PART_clone and forces it to the inputed pose

#### gripper_open.launch.py
* launch the command_gripper.py node to create the action client and tell the FollowJointTrajectory server to open fingers, it also deactivates the grasping_controller.py node called 'gripper_lifecycle_node' to unlock the part position
* two lifecycle modes are used here:
    * deactivate: do any cleanup to start executing, and should reverse the onActivate changes.
    * clean up: clear all state and return the node to a functionally equivalent state as when first created. If the cleanup cannot be successfully achieved it will transition to ErrorProcessing.


### --- SCRIPT ---

#### command_gripper.py
* create an action client, from rclpy library, that will communicate with FollowJointTrajectory server from control_msgs.action library
* this action client create a list of points based on the pose inputs and communicated to the server
* the FollowJointTrajectory action is defined by the control_msgs package and makes the robot move along a trajectory. Mainly, it specifies the goal action as a trajectory composed of list of joint configurations and some information on the tolerance in its following, defines some result states (ranging from SUCCESS to different error situations like GOAL_TOLERANCE_VIOLATED), and sets the action feedback as the desired and the actual geometry_msgs/JointTrajectoryPoint.
* doc to create an action server/client: https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html#writing-an-action-server
* in the end, the return of the send_goal() method is the answer from the FollowJointTrajectory server, meaning this is the interpolation between the points = trajectory, which is published on the topic /test_namespace/joint_trajectory_controller/follow_joint_trajectory
* --> this node is called by the open/close gripper launch files

#### command_robot.py
* create an action client, from rclpy library, that will communicate with FollowJointTrajectory server from control_msgs.action library
* same script as command_gripper.py but to command the robot
* in the end, the return of the send_goal() method is the answer from the FollowJointTrajectory server, meaning this is the interpolation between the points = trajectory, which is published on the topic /test_namespace/joint_trajectory_controller/follow_joint_trajectory
* --> this node is called by the poses launch files when the robot is asked to move to a position

#### Listen_Command_robot_example.py
* this script is the same code than command_robot.py but the difference is that in addition to the action client, there is a subscriber that listens to /joint_states
* this code has no use in the overall training but is simply an example of what a action_client/subscriber could look like
* the subscriber takes the current states of the joints and give the information, in addition to the target points, to the server FollowJointTrajectory (but the reality is that the server already knows the current state by listening on its own to the same topic)


#### import_3D_1.py
* create a publisher that published the 3Dshape mesh on a topic until the environment is shutdown
* this script inheritate from the minimal publisher class thanks to super()
* publishes the structure of the environment

#### import_3D_2.py
* create a publisher that published the 3Dshape mesh on a topic until the environment is shutdown
* this script inheritate from the minimal publisher class thanks to super()
* publishes the parts of the environment


### --- rviz ---

#### example.rviz
* it is a rviz generated file, saving a certain configuration of the window layout and the rviz parameters


### --- world ---

#### example.world
* it is a gazebo generated file, saving a certain configuration of the gazebo environment
* the gazebo example.world defines, for each gazebo entity, the path to its mesh, some other properties as the position, and its name wich can be used as an identifier
* the meshes are stored in the model>gazebo folder






