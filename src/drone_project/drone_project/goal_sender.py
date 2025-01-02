import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import cv2
import numpy as np 
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, VehicleOdometry
from goal_msg.msg import GoalMsg
from std_msgs.msg import Float32MultiArray
#from drone_project.CV_library import find_circles_with_most_pixels_generator, find_next_circle


class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')

                # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        #Initialize subscribers
        self.process_image_sub = self.create_subscription(
            Image,
            'processed_image_topic',
            self.process_image_callback,
            10
        )

        self.command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        self.px4_odometry_subscriber = self.create_subscription(
            VehicleOdometry,
            'fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile
        )
        
        self.end_publisher = self.create_publisher(
            Bool,
            'end_topic',
            qos_profile
        )
        
        self.goal_publisher = self.create_publisher(
            GoalMsg,
            'goal_topic',
            qos_profile
        )
        """
        self.takeoff_subscriber = self.create_subscription(Bool,
                                                       'takeoff_topic',
                                                       self.takeoff_callback,
                                                       qos_profile)
        """
        self.bridge = CvBridge()
        self.x_g = 0.0
        self.y_g = 0.0

        self.offboard_set = False
        self.armed = False

        self.current_goal = [0.0, 0.0] 
        self.current_goal_index = 0

        self.processed_image = None
        self.generator = None

        self.drone_position = None

        self.updated_map = None

        self.takeoff = False
        self.end_flag = False

        self.center = None

        self.goals = []
        self.goal_sent = False
        self.update = False

    def process_image_callback(self, msg):

        self.get_logger().info('Received processed_image')
        self.processed_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        self.updated_map = self.processed_image.copy()

        self.offboard_set = True
        self.pixel_coordinate_mapping()

    def takeoff_callback(self, msg):
        self.get_logger().info('The drone has takeoff')
        self.takeoff = msg.data

    
    def pixel_coordinate_mapping(self):
        if self.processed_image is not None and self.updated_map is not None:
            if not self.update:
                x1, y1, self.updated_map = self.find_next_circle(self.processed_image)
                if x1 is not None and y1 is not None:
                    print(f"Sending agent to (x={x1}, y={y1})")
                    self.x_g = float(x1)
                    self.y_g = float(y1)
                    self.update = True
                    self.send_goal()
            else:
                xd, yd, self.updated_map = self.find_next_circle(self.updated_map)
                if xd is not None and yd is not None:
                    self.x_g = float(xd)
                    self.y_g = float(yd)
                    self.send_goal()
                else:
                    if self.end_flag:
                        msg = Bool()
                        msg.data = self.end_flag
                        self.end_publisher.publish(msg)
        else:
            self.get_logger().info('Received processed_image not working')           
                                

    def find_next_circle(self, binary_mask, radius = 150, step=10, reference_point=None):


        #binary_mask = (binary_mask == 255).astype(np.uint8)

        h, w = binary_mask.shape
        """
        if not np.any(binary_mask):
            # No pixels to process
            self.get_logger().info('No return')
            return None, None, binary_mask
        """
        # Create circular kernel
        diameter = 2 * radius + 1
        y_indices, x_indices = np.ogrid[-radius:radius+1, -radius:radius+1]
        circular_kernel = (x_indices**2 + y_indices**2 <= radius**2).astype(np.uint8)

        # Generate grid indices
        y_grid = np.arange(0, h, step)
        x_grid = np.arange(0, w, step)

        # Convolution output array
        conv_output = np.zeros((len(y_grid), len(x_grid)), dtype=np.int32)

        for i, y in enumerate(y_grid):
            for j, x in enumerate(x_grid):
                # Define the region of interest (ROI)
                y_min = max(y - radius, 0)
                y_max = min(y + radius + 1, h)
                x_min = max(x - radius, 0)
                x_max = min(x + radius + 1, w)

                roi = binary_mask[y_min:y_max, x_min:x_max]

                # Define the corresponding kernel region
                ky_min = radius - (y - y_min)
                ky_max = radius + (y_max - y)
                kx_min = radius - (x - x_min)
                kx_max = radius + (x_max - x)

                kernel_roi = circular_kernel[ky_min:ky_max, kx_min:kx_max]

                # Compute the sum within the circle at this grid point
                conv_output[i, j] = np.sum(roi * kernel_roi)
                #self.get_logger().info(f'Value at position {i}, {j} is: {conv_output}')

        # Find the maximum value in the conv_output
        max_value = conv_output.max()

        if max_value == 0:
            # No more pixels can be covered
            self.get_logger().info('End of pixels')
            self.end_flag = True
            return None, None, binary_mask

        # Find all positions where the convolution output is equal to max_value
        positions = np.argwhere(conv_output == max_value)

        # Map grid positions back to image coordinates
        y_grid_positions = y_grid[positions[:, 0]]
        x_grid_positions = x_grid[positions[:, 1]]
        positions_in_image = np.stack((y_grid_positions, x_grid_positions), axis=-1)

        # Define the reference point (default to center of the image)
        if reference_point is None:
            reference_point = np.array([h // 2, w // 2])

        # Compute distances from positions to the reference point
        distances = np.linalg.norm(positions_in_image - reference_point, axis=1)

        # Choose the position with minimum distance
        min_index = np.argmin(distances)
        center = positions_in_image[min_index]  # (y, x)

        self.get_logger().info(f"Selected center at (x={center[1]}, y={center[0]}) covering {max_value} pixels")

        # Exclude the pixels within the circle centered at this point
        # Create a mask with a circle at the selected center
        temp_mask = np.zeros_like(binary_mask)
        cv2.circle(temp_mask, (int(center[1]), int(center[0])), radius, 1, thickness=-1)  # Note: (x, y)

        # Set the pixels within the circle to 0 in binary_mask
        updated_mask = binary_mask.copy()
        updated_mask[temp_mask == 1] = 0

        # Convert center to (x, y) tuple
        center_xy = (int(center[1]), int(center[0]))

        return (center[1]),(center[0]), (updated_mask * 255).astype(np.uint8)

    def send_goal(self):
        #if self.current_goal_index < len(self.goals):
        #    goal = self.goals[self.current_goal_index]
            #goal_msg = Float32MultiArray(data=[goal[0], goal[1]]) #GoalMsg()
        
        goal_msg = GoalMsg()
        goal_msg.x_p = self.x_g
        goal_msg.y_p = self.y_g
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f'Sent goal {self.current_goal_index + 1}: goal {self.x_g}, {self.y_g}')
        self.goal_sent = True


    def odometry_callback(self, msg):
        self.drone_position = msg
        if self.goal_sent and self.goal_reached():
            #self.get_logger().info(f'Goal x:{self.current_goal[0]}, y:{self.current_goal[1]}')
            self.goal_sent = False
            self.current_goal_index += 1
            self.pixel_coordinate_mapping()

    def goal_reached(self):
        if self.drone_position is None:
            return False
        
        #self.current_goal = self.goals[self.current_goal_index]
        drone_x = self.drone_position.position[0]
        drone_y = self.drone_position.position[1]
        drone_z = self.drone_position.position[2]
        #x_g, y_g = self.current_goal
        distance = ((drone_x- self.x_g)**2 + (drone_y - self.y_g)**2)**0.5

        if distance < 2.0:
            self.get_logger().info(f'Goal {self.x_g}, {self.y_g} reached')

        return distance < 2.0

    def send_goal_position(self):
        self.x_g = self.current_goal[0]
        self.y_g = self.current_goal[1]
        position = [self.x_g, self.y_g, -5.0]
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position = position
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        msg.yawspeed = float('nan')
        self.px4_publisher.publish(msg)
        self.get_logger().info(f'Sent goal position: {msg.position}')

def main(args=None):
    rclpy.init(args=args)
    goal_sender = GoalSender()
    rclpy.spin(goal_sender)
    goal_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()