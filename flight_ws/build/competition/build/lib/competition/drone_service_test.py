import sys
from tello_msgs.srv import TelloAction
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from .utils.flight_control import fly_drone
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image

class TestClient(Node):

    def __init__(self):
        super().__init__('test_client')
        self.subscription = self.create_subscription(
            Int32,
            '/drone1/flight_data',
            self.listener_callback,
            1
        )
        self.img_subscription = self.create_subscription(
            Image,
            '/drone1/image_raw',
            self.img_listener_callback,
            1
        )
        self.cli = self.create_client(TelloAction, '/drone1/tello_action')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TelloAction.Request()
        self.dist = 0
        self.br = CvBridge()
        self.frame = Image()
    
    def send_command(self, cmd): 
        self.req.cmd = cmd
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        try:
            # Get the response from the server
            response = self.future.result()
            if response.rc == TelloAction.Response.OK:
                self.get_logger().info('Command sent successfully.')
                
            else:
                self.get_logger().info(f'Command failed. Response code: {response.rc}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def send_request(self):
        x, y, z, yaw = fly_drone(self.frame, self.dist)
        command = f'rc {y} {x} {z} {yaw}'
        print(command)
        print("that was the command yeet")
        self.req.cmd = command
        self.future = self.cli.call_async(self.req)
        # rclpy.spin_until_future_complete(self, self.future)
        # try:
        #     # Get the response from the server
        #     response = self.future.result()
        #     if response.rc == TelloAction.Response.OK:
        #         self.get_logger().info('Command sent successfully.')
                
        #     else:
        #         self.get_logger().info(f'Command failed. Response code: {response.rc}')
        # except Exception as e:
        #     self.get_logger().error(f'Service call failed: {e}')
    
    def listener_callback(self, msg):
        self.dist = msg.tof
    
    def img_listener_callback(self, data):
        # Display the message on the console
        self.get_logger().info('Receiving video frame')
    
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)
        # # Display image
        # cv2.imshow("camera", current_frame)
        cv2.waitKey(1)
        self.frame = current_frame
        self.send_request()


def main(args=None):
    rclpy.init(args=args)
    drone_client = TestClient()
    drone_client.send_command("takeoff")
    drone_client.send_command("up 50")
    # minimal_client.get_logger().info(
    #     'Result of test client: for %d' %
    #     (response))
    rclpy.spin(drone_client)
    drone_client.send_command("land")
    drone_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    minimal_client = TestClient()
    minimal_client.send_command("takeoff")
    main()
    minimal_client.send_command("land")