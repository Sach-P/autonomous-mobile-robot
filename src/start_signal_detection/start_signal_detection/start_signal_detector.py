import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class StartSignalDetector(Node):
    def __init__(self):
        super().__init__('start_signal_detector')

        self._image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self._image_pub = self.create_publisher(Image, '/start_signal_detection/debug_image', 10)

        self._cv_bridge = CvBridge()


    def image_callback(self, msg: Image):

        try:
            bgr_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

            roix = 100
            roiy = 100
            roisize = 25

            roi = hsv_image[roiy-roisize:roiy+roisize, roix-roisize:roix+roisize]

            hsv_color = cv2.mean(roi)

            color = StartSignalDetector.classify_hsv_color(hsv_color[0], hsv_color[1], hsv_color[2])

            self.get_logger().info(f'ROI mean color: {color}')

            cv2.rectangle(bgr_image, (roix-roisize, roiy-roisize), (roix+roisize, roiy+roisize), (0, 255, 0), 2)
            cv2.putText(bgr_image, color, (roix-roisize, roiy-roisize-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            debug_msg = self._cv_bridge.cv2_to_imgmsg(bgr_image, encoding='bgr8')
            debug_msg.header = msg.header

            self._image_pub.publish(debug_msg)


        except CvBridgeError as e:
            self.get_logger().error(f'Error processing image: {e}')

    @staticmethod
    def classify_hsv_color(h, s, v):

        if v < 40:
            return "Black"
        if s < 40 and v > 200:
            return "White"
        if s < 40:
            return "Gray"

        if h < 10 or h >= 170:
            return "Red"
        elif h < 25:
            return "Orange"
        elif h < 35:
            return "Yellow"
        elif h < 85:
            return "Green"
        elif h < 130:
            return "Blue"
        elif h < 170:
            return "Violet/Purple"
        
        return "Unknown"


def main():
    rclpy.init()
    node = StartSignalDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt as e:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()