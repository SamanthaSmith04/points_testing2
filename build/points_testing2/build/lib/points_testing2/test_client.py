#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from services.srv import DownsampleSrvInfo
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from std_msgs.msg import Header


class TestClient(Node): 
    def __init__(self):
        super().__init__('test_client')
        self.cli = self.create_client(DownsampleSrvInfo, 'downsample')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DownsampleSrvInfo.Request()

    def send_request(self):
        self.req.input_file = "full_points.txt"
        self.req.epsilon =  0.1
        self.future = self.cli.call_async(self.req)

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(PoseArray, '/downsampled_points', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.pose_array = None

    def timer_callback(self):
        if self.pose_array is not None:
            self.publisher_.publish(self.pose_array)
            print("pub")
            self.i += 1

    def set_point_array(self, point_array):
        self.pose_array = PoseArray()
        self.pose_array = point_array
        self.pose_array.header = Header()
        self.pose_array.header.frame_id = "map"
        print("usccess")
        

def main(args=None):
    rclpy.init(args=args)

    test_client = TestClient()
    test_client.send_request()

    pub_to_rviz = PublisherNode()


    while rclpy.ok():
        rclpy.spin_once(test_client)
        if test_client.future.done():
            try:
                response = test_client.future.result()
            except Exception as e:
                test_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                #print(response.corrected_poses)
                print("points corrected!")
                global point_array
                point_array = response.corrected_poses
                pub_to_rviz.set_point_array(point_array)
            break
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(pub_to_rviz)
    executor.spin()
    executor.shutdown()

    test_client.destroy_node()
    pub_to_rviz.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
