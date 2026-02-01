import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from omni.isaac.core.articulations import Articulation
import numpy as np

class LinkerL20Controller(Node):
    def __init__(self, hand_articulation):
        super().__init__('linker_l20_controller')
        
        # Store reference to the hand articulation
        self.hand = hand_articulation
        
        # Get joint names from the hand
        self.joint_names = self.hand.dof_names
        
        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            '/linker_l20/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Store target positions
        self.target_positions = None
        
        self.get_logger().info(f'Linker L20 Controller initialized with joints: {self.joint_names}')
    
    def joint_state_callback(self, msg):
        """Callback for receiving joint state commands"""
        # Map incoming joint names to positions
        position_dict = dict(zip(msg.name, msg.position))
        
        # Create target position array in the correct order
        self.target_positions = []
        for joint_name in self.joint_names:
            if joint_name in position_dict:
                self.target_positions.append(position_dict[joint_name])
            else:
                # Keep current position if joint not specified
                current_pos = self.hand.get_joint_positions()
                idx = self.joint_names.index(joint_name)
                self.target_positions.append(current_pos[idx])
        
        self.target_positions = np.array(self.target_positions)
    
    def apply_action(self):
        """Apply the target positions to the hand"""
        if self.target_positions is not None:
            self.hand.set_joint_position_targets(self.target_positions)


# Integration into your follow_target script
class FollowTargetWithHand:
    def __init__(self):
        # Initialize ROS2
        rclpy.init()
        
        # Your existing xarm7 setup
        self.robot = self.setup_xarm7()
        
        # Get the Linker L20 hand articulation
        # Adjust the path based on your USD structure
        self.hand = world.scene.get_object("linker_l20_hand")
        # Or if it's part of the robot:
        # from omni.isaac.core.utils.prims import get_prim_at_path
        # hand_prim = get_prim_at_path("/World/xarm7/linker_l20")
        # self.hand = Articulation(prim_path="/World/xarm7/linker_l20")
        # self.hand.initialize()
        
        # Create the hand controller
        self.hand_controller = LinkerL20Controller(self.hand)
        
    def physics_step(self):
        # Your existing follow_target logic for xarm7
        self.update_xarm7()
        
        # Process ROS2 callbacks
        rclpy.spin_once(self.hand_controller, timeout_sec=0)
        
        # Apply hand commands
        self.hand_controller.apply_action()
    
    def cleanup(self):
        self.hand_controller.destroy_node()
        rclpy.shutdown()