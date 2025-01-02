import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import os

class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'risk_image_topic', 10)
        #self.timer = self.create_timer(60.0, self.timer_callback)  # Set timer to 60 seconds
        self.sync_subscriber = self.create_subscription(
            Bool,
            'control_topic',
            self.sync_callback,
            10)
        
        self.image_folder = '/home/dc7/CPS/Project/ros2_ws/Risk_images'  
        self.image_list = sorted(os.listdir(self.image_folder))

        self.declare_parameter('scenario', 0)
        self.scenario = self.get_parameter('scenario').value

        #self.image_index = 0
        self.bridge = CvBridge()
        self.image_published = False

    """
    def timer_callback(self):
        if self.image_index >= len(self.image_list):
            self.image_index = 0  # Loop back to the first image

        image_path = os.path.join(self.image_folder, self.image_list[self.image_index])
        cv_image = cv2.imread(image_path)
        cv_image = cv2.blur(cv_image, (5,5), anchor=(-1, -1), borderType=cv2.BORDER_DEFAULT)
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

        self.publisher_.publish(ros_image)
        self.get_logger().info(f'Publishing: {self.image_list[self.image_index]}')

        self.image_index += 1
    """

    def sync_callback(self, msg):
        if not self.image_published:
            self.publish_image()

    def publish_image(self):
        image_name = self.image_list[self.scenario]
        image_path = os.path.join(self.image_folder, image_name)
        cv_image = cv2.imread(image_path)
        
        if cv_image is not None:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.publisher_.publish(ros_image)
            self.get_logger().info(f'Published risk image: {image_name}')
            self.image_published = True
        else:
            self.get_logger().error(f'Failed to read image: {image_name}')    

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
