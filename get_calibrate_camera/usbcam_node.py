import rclpy
from rclpy.node import Node  
import cv2  
from cv_bridge import CvBridge  
from sensor_msgs.msg import Image 
import argparse  

class CameraNode(Node):
    def __init__(self, cam):
        super().__init__('usbcam_node')  
        self.publisher_ = self.create_publisher(Image, 'usbcam_image', 1)  
        self.timer = self.create_timer(0.001, self.timer_callback) 
        self.cap = cv2.VideoCapture(cam)  
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) 
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640) 
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) 
        self.cap.set(cv2.CAP_PROP_FPS, 60)  
        self.bridge = CvBridge() 

    def timer_callback(self):
        ret, frame = self.cap.read()  
        if ret:  
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")  
            self.publisher_.publish(msg)  
            # cv2.imshow('Webcam', frame)  
            if cv2.waitKey(1) & 0xFF == ord('q'):  
                rclpy.shutdown()  
        else:
            self.get_logger().error('Failed to capture image')  

def main(args=None):
    parser = argparse.ArgumentParser(description='ROS2 USB Camera Node')
    parser.add_argument('--cam', type=int, default=0, help='Index of the camera (default is 0)')
    cli_args = parser.parse_args()

    rclpy.init(args=args)  
    node = CameraNode(cli_args.cam)  
    try:
        rclpy.spin(node)  
    except KeyboardInterrupt:
        pass  
    finally:
        node.cap.release()  
        cv2.destroyAllWindows()  
        node.destroy_node()  
        rclpy.shutdown()  

if __name__ == '__main__':
    main()  
