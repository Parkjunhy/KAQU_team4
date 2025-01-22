import numpy as np
import rclpy
from rclpy.node import Node
from kaqu_msgs.msg import JoyCtrlCmds  # Message type for joystick commands
from ..KaquCmdManager.CmdManagerNode import RobotState, BehaviorState, RobotCommand
from std_msgs.msg import String

# Existing Gait Controllers
class TrotGaitController:
    def run(self, state, command):
        return "Running Trot Gait"

class SideMoveController:
    def run(self, state, command):
        return "Running Side Move"

class StartController:
    def run(self, state, command):
        return "Starting Robot"

class RobotController(Node):
    def __init__(self, body, legs):
        super().__init__('robot_controller')  # Node initialization
        self.body = body
        self.legs = legs

        self.delta_x = self.body[0] * 0.5
        self.delta_y = self.body[1] * 0.5 + self.legs[1]
        self.x_shift_front = 0.001
        self.x_shift_back = -0.001
        self.default_height = 0.5

        self.state = RobotState(self.default_height)
        self.command = RobotCommand(self.default_height)
        self.state.foot_location = self.default_stance()

        # Initialize Gait Controllers
        self.trot_controller = TrotGaitController()
        self.sidemove_controller = SideMoveController()
        self.start_controller = StartController()

        self.current_controller = self.start_controller  # Default controller

        # JoyCtrlCmds subscriber
        self.joy_subscriber = self.create_subscription(
            JoyCtrlCmds,
            'robot_state',  # Topic name to subscribe
            self.joy_callback,  # Callback function
            10  # Queue size
        )

        # Publisher to send current controller info for testing
        self.controller_pub = self.create_publisher(String, '/current_controller', 10)

        # Timer to publish current controller status every 1 second
        self.timer = self.create_timer(1.0, self.publish_current_controller)

    def joy_callback(self, msg):
        """Process JoyCtrlCmds message and select gait based on joystick input."""
        self.get_logger().info(f"Received JoyCtrlCmds: {msg}")

        # Assign commands based on joystick states
        if msg.states[0]:  # START state
            self.command.start_event = True
            self.command.trot_event = False
            self.command.side_event = False
        elif msg.states[1]:  # TROT state
            self.command.start_event = False
            self.command.trot_event = True
            self.command.side_event = False
        elif msg.states[2]:  # SIDEMOVE state
            self.command.start_event = False
            self.command.trot_event = False
            self.command.side_event = True

        # Select gait based on the state
        self.select_gait()

    def default_stance(self):
        """Define the default stance based on body and leg dimensions."""
        return np.array([[self.delta_x + self.x_shift_front, self.delta_x + self.x_shift_front,
                          -self.delta_x + self.x_shift_back, -self.delta_x + self.x_shift_back],
                         [-self.delta_y, self.delta_y, -self.delta_y, self.delta_y],
                         [0, 0, 0, 0]])

    def select_gait(self):
        """Select the appropriate gait controller based on the current command."""
        if self.command.trot_event:
            self.current_controller = self.trot_controller
            self.state.trot_event = False   
        elif self.command.start_event:
            self.current_controller = self.start_controller
            self.state.start_event = False  
        elif self.command.side_event:
            self.current_controller = self.sidemove_controller
            self.state.side_event = False

    def publish_current_controller(self):
        """Publish the name of the current controller."""
        if self.current_controller:
            msg = String()
            msg.data = str(self.current_controller)  # Convert controller to string
            self.get_logger().info(f"Publishing current controller: {msg.data}")
            self.controller_pub.publish(msg)

    def run(self):
        """Execute the logic of the current controller."""
        if self.current_controller:
            result = self.current_controller.run(self.state, self.command)
            return result
        else:
            self.get_logger().warn("No controller to run.")
            return None

def main(args=None):
    rclpy.init(args=args)

    # Initialize body and leg dimensions
    body_dimensions = [0.5, 0.3]  # [length, width]
    leg_dimensions = [0.2, 0.1]  # [length, offset]

    # Create the RobotController node
    robot_controller = RobotController(body_dimensions, leg_dimensions)

    try:
        while rclpy.ok():
            rclpy.spin_once(robot_controller, timeout_sec=0.1)

            # Run the active controller logic
            result = robot_controller.run()
            if result is not None:
                robot_controller.get_logger().info(f"Controller result: {result}")
            else:
                robot_controller.get_logger().warn("No controller running.")
    except KeyboardInterrupt:
        robot_controller.get_logger().info("Shutting down RobotController.")

    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
