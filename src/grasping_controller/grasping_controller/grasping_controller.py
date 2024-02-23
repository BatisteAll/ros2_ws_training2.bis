import rclpy
import time

from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import ContactsState

class GraspingController(Node):



  def __init__(self, node_name,**kwargs) -> None:
    super().__init__(node_name,**kwargs)
    
    # Declare all the parameters used to define (declare_parameter() is from the node class of rclpy) :
    # Object to grasp
    self.declare_parameter('mobile_element', 'PART')
    # Reference frame that define the fixed position
    self.declare_parameter('fix_frame', 'test_robot::link_Lgripper')
    # This parameter is used to define if the position will be set to the current position (true)
    # or forced to the Position parameter (false)
    self.declare_parameter('get_state', False)
    # Position
    self.declare_parameter('position_x', 0.0)
    self.declare_parameter('position_y', 0.0)
    self.declare_parameter('position_z', 0.0)
    self.declare_parameter('orientation_x', 0.0)
    self.declare_parameter('orientation_y', 0.0)
    self.declare_parameter('orientation_z', 0.0)
    self.declare_parameter('orientation_w', 1.0)

    # Create a service_client action that allows to set or get the entity state. 
    # This is used to force or get the position of the grasped object.
    self.set_entity_state_client = self.create_client(SetEntityState, '/set_entity_state')
    self.get_entity_state_client = self.create_client(GetEntityState, '/get_entity_state')





  # Callback of the contact subscriber that call the SetEntityState client 
  # at each timestep to force the position of the grasped object
  def contact_callback(self, msg):
    # The request (i.e. the command) of the setEntityState method is set based on the node parameters
    request = SetEntityState.Request()
    # The name here is important because it identifies the object to move in the gazebo elements
    request.state.name = mobile_element
    # If the get_state option is false, the position is forced to the declared one (e.g. center of the gripper fingers)
    if get_state == False:
       request.state.pose = self.gripped_object_pose_declared
    # If the get_state option is true, the position is forced to the actual one (in the frame of the gripper)
    elif get_state == True:
       request.state.pose = gripped_object_pose_actual_state
    request.state.reference_frame = fix_frame
    # Set the position of the part. call_async make a service request and asyncronously get the result.
    self.set_entity_state_client.call_async(request)





  # Definition of the on-configure mode of the lifecycle node. 
  # This configuration create the variable associated to the parameters and asign them values
  def on_configure(self, state: State) -> TransitionCallbackReturn:
    # The TransitionCallbackReturn gives the Callback/status/output of the transition

    # Set the log message
    self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'configure'")

    # Define parameters
    global position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w, mobile_element, fix_frame, get_state
    # Set parameters
    mobile_element = self.get_parameter('mobile_element').get_parameter_value().string_value
    fix_frame = self.get_parameter('fix_frame').get_parameter_value().string_value
    get_state = self.get_parameter('get_state').get_parameter_value().bool_value

    # If the get_state option is false, the position is forced to the one declared as arguement
    if get_state == False:
      position_x = self.get_parameter('position_x').get_parameter_value().double_value
      position_y = self.get_parameter('position_y').get_parameter_value().double_value
      position_z = self.get_parameter('position_z').get_parameter_value().double_value
      orientation_x = self.get_parameter('orientation_x').get_parameter_value().double_value
      orientation_y = self.get_parameter('orientation_y').get_parameter_value().double_value
      orientation_z = self.get_parameter('orientation_z').get_parameter_value().double_value
      orientation_w = self.get_parameter('orientation_w').get_parameter_value().double_value

      # Set the pose to which the part will be forced.
      # A pose is: a positon + an orientation
      self.gripped_object_pose_declared = Pose(
        position=Point(x=float(position_x), y=float(position_y), z=float(position_z)),
        orientation=Quaternion(x=float(orientation_x), y=float(orientation_y), z=float(orientation_z), w=float(orientation_w))
    )

    # If the get_state option is true, the part is forced to its actual position in the gripper reference frame
    elif get_state == True:
      self.response = self.get_object_coordinates()

    # The Callback of the transition is set to success and returned
    return TransitionCallbackReturn.SUCCESS




  # Function that request the state of the object to grasp
  def get_object_coordinates(self):
    # The request (i.e. the command) of the getEntityState method is set based on the node parameters
    self.request = GetEntityState.Request()
    self.request.name = mobile_element
    self.request.reference_frame = fix_frame
    # Get the pose of the part. call_async make a service request and asyncronously get the result.
    self.future = self.get_entity_state_client.call_async(self.request)
    # Add a callback that gets the result of the request
    self.future.add_done_callback(self.get_entity_state_callback)





  # Callback function that get the result of the request, store it in a global variable and write the state in the log
  def get_entity_state_callback(self, future):
        # Define the global variable to stor the state
        global gripped_object_pose_actual_state
        # Try to get the state, or return fail
        try:
            response = future.result()
            if response.success:
                gripped_object_pose_actual_state = response.state.pose
                self.get_logger().info(f"Box position: x={gripped_object_pose_actual_state.position.x}, \
                    y={gripped_object_pose_actual_state.position.y}, \
                    z={gripped_object_pose_actual_state.position.z}")
                self.get_logger().info(f"Box orientation: x={gripped_object_pose_actual_state.orientation.x}, \
                    y={gripped_object_pose_actual_state.orientation.y}, \
                    z={gripped_object_pose_actual_state.orientation.z}, \
                    w={gripped_object_pose_actual_state.orientation.w}")
            else:
                self.get_logger().info('>>> Failed to get entity state: ', response.status_message)
        except Exception as e:
            self.get_logger().info('>>> Service call failed to get entity state', str(e))





  # Definition of the on-activate mode of the lifecycle node. 
  # This mode creates a subscriber in which the callback function fixes the position of the grasped object
  def on_activate(self, state: State) -> TransitionCallbackReturn:
    # Set the log message
    self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'activate'")
    # Create the subscriber to the contact between the left finger and any object
    self.contact_state_subscription = self.create_subscription(
        ContactsState,
        '/test_namespace/bumper_link_Lgripper',
        self.contact_callback, # Whenever there is a publish on the topic (i.e. there is contact or not), the contact method is called and the part position is forced
        10 # QoS keep the 10 last samples for reliability
    )
    # Instentiate the subscriber  
    self.contact_state_subscription

    # The Callback of the transition is set to success and returned
    return TransitionCallbackReturn.SUCCESS





  # Definition of the on-deactivate mode of the lifecycle node. 
  # This mode destroys the subscriber so it releases part
  def on_deactivate(self, state: State) -> TransitionCallbackReturn:
    # Set the log message
    self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'deactivate'")
    # Destroys the subscription
    self.destroy_subscription(self.contact_state_subscription)

    # The Callback of the transition is set to success and returned
    return TransitionCallbackReturn.SUCCESS




  # Definition of the on-shutdown mode of the lifecycle node. 
  # This mode shutdown the node 
  def on_shutdown(self, state: State) -> TransitionCallbackReturn:
    # Set the log message
    self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'shutdown'")

    # The Callback of the transition is set to success and returned
    return TransitionCallbackReturn.SUCCESS





def main(args=None) -> None:
  rclpy.init(args=args)

  # Instentiate the GraspingController lifecycle node and execute it until the context is shutdown
  # The name given to the node is grasping_lifecycle_node
  grasping_controller = GraspingController("grasping_lifecycle_node")
  rclpy.spin(grasping_controller)

  rclpy.shutdown()



if __name__ == "__main__":
  main()

