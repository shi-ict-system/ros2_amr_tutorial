import rclpy
import cv2

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageSubProcessing(Node):
    def __init__(self):
        super().__init__('image_sub_processing_node')

        self.subscription = self.create_subscription(Image, 'image_raw', self.image_callback, 10)
        self.subscription  # prevent unused variable warning

        self.br = CvBridge()

    def image_callback(self, msg):
        image_frame = self.br.imgmsg_to_cv2(msg)

        gray_image = cv2.cvtColor(image_frame, cv2.COLOR_BGR2GRAY)
        blur_image = cv2.blur(gray_image, (3, 3))
        edges = cv2.Canny(blur_image, 0, 0, 3)

        cv2.imshow("result", edges)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    image_sub_processing = ImageSubProcessing()

    rclpy.spin(image_sub_processing)

    image_sub_processing.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
