import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np 
from px4_msgs.msg import TrajectorySetpoint
from drone_project.CV_library import color_threshold

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        
        # Initialize subscribers
        self.standard_image_sub = self.create_subscription(
            Image,
            'standard_image_topic',
            self.standard_image_callback,
            10
        )
        self.risk_image_sub = self.create_subscription(
            Image,
            'risk_image_topic',
            self.risk_image_callback,
            10
        )
        
        # Initialize publisher
        self.control_publisher = self.create_publisher(
            Bool,
            'control_topic',
            10
        )

        self.processed_image_pub = self.create_publisher(
            Image,
            'processed_image_topic',
            10
        )

        #self.px4_publisher = self.create_publisher(TrajectorySetpoint, 'fmu/trajectory_setpoint/in', 10)
        
        # CvBridge to convert ROS images to OpenCV format
        self.bridge = CvBridge()

        #images
        self.standard_image = None
        self.risk_image = None
        self.standard_gray = None
        self.risk_gray = None

        self.publish_control_signal(True)

    def standard_image_callback(self, msg):
        self.get_logger().info('Received standard_image')
        self.standard_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.process_images()

    def risk_image_callback(self, msg):
        self.get_logger().info('Received risk_image')
        self.risk_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.process_images()

    def color_threshold(self, image, ref_color, threshold, rgb=True):
    
        if rgb:
            
            ref_color_image = np.copy(image)
            ref_color_image[:,:] = ref_color 

            diff = cv2.absdiff(image, ref_color_image)
            distance = np.linalg.norm(diff, axis=2)
            # Create binary mask
            binary_image = np.zeros_like(distance, dtype=np.uint8)
            binary_image[distance <= threshold] = 255
        else:
            # Ensure the image is grayscale
            if len(image.shape) == 3:
                gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray_image = image.copy()
            # Compute absolute difference
            diff = cv2.absdiff(gray_image, np.uint8(ref_color))
            # Create binary mask
            binary_image = np.zeros_like(diff, dtype=np.uint8)
            binary_image[diff <= threshold] = 255

        return binary_image

    def process_images(self):
        # Ensure both images are received before processing
        if self.standard_image is not None and self.risk_image is not None:
            #Perform computer vision tasks here
            if np.size(self.standard_image == self.risk_image):
                #blur images
                standard_blur = cv2.blur(self.standard_image, (5,5), borderType = cv2.BORDER_DEFAULT)
                risk_blur = cv2.blur(self.risk_image, (5,5), borderType = cv2.BORDER_DEFAULT)
                processed_image = cv2.absdiff(standard_blur, risk_blur)

                self.standard_gray = cv2.cvtColor(standard_blur, cv2.COLOR_RGB2GRAY)
                self.risk_gray = cv2.cvtColor(risk_blur, cv2.COLOR_RGB2GRAY)

                processed_gray_image = cv2.absdiff(self.standard_gray, self.risk_gray)
             
                ret, thresh = cv2.threshold(processed_gray_image, 20, 255, cv2.THRESH_BINARY)

                burn_color_r = 55
                burn_color_g = 43
                burn_color_b = 33
                burn_color = np.array([np.int32(burn_color_b), np.int32(burn_color_g), np.int32(burn_color_r)])

                mask_image = self.color_threshold(risk_blur, burn_color, 15)
                
                masked_diff = cv2.bitwise_and(thresh, mask_image)

                #now open and dilate it
                kernel = np.ones((3,3), np.uint8)
                binary_mask = cv2.morphologyEx(masked_diff, cv2.MORPH_OPEN, kernel)


                dilated_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_DILATE, kernel, iterations=1)
                

            else:
                self.get_logger().info('Standard and risk images not received')
            # Publish the processed image

            processed_image_msg = self.bridge.cv2_to_imgmsg(dilated_mask, encoding = 'mono8')
            self.processed_image_pub.publish(processed_image_msg)
            self.get_logger().info('Published processed_image')

            # Reset images
            self.standard_image = None
            self.risk_image = None
        
    """

    def send_goal_position(self, area):
        x_p = np.float32(area[0])
        y_p = np.float32(area[1])
        position = [x_p, y_p, 15.0]
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position = position
        self.px4_publisher.publish(msg)
        self.get_logger().info(f'Sent goal position: {position}')
        """

    def publish_control_signal(self, signal):
        control_msg = Bool()
        control_msg.data = signal
        self.control_publisher.publish(control_msg)
        self.get_logger().info('Sent synchronization signal')

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
