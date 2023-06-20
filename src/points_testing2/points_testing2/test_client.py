#! /usr/bin/env python3
#A group of nodes that can be used to test the downsample service
#Author: Samantha Smith, smith.15485@osu.edu

import rclpy
from rclpy.node import Node
from services.srv import DownsampleSrvInfo
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from points_testing2 import corrector_functions

class TestClient(Node): 
    def __init__(self):
        super().__init__('test_client')
        self.cli = self.create_client(DownsampleSrvInfo, 'downsample')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DownsampleSrvInfo.Request()

    def send_request(self):
        self.req.input_file = "points2.txt"
        self.req.epsilon = 0.05 #meters
        self.req.angle_threshold = 95.0 #degrees
        self.future = self.cli.call_async(self.req)

class PublishOrientationNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(PoseArray, '/downsampled_orientations', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.pose_array = None

    def timer_callback(self):
        if self.pose_array is not None:
            self.publisher_.publish(self.pose_array)
            print("pub orientation \n")
            self.i += 1

    def set_pose_array(self, point_array):
        self.pose_array = PoseArray()
        self.pose_array = point_array
        self.pose_array.header = Header()
        self.pose_array.header.frame_id = "map"
        print("usccess")
        
class PublishPositionNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(MarkerArray, '/downsampled_positions', 10)
        self.publisher_lines = self.create_publisher(MarkerArray, '/downsampled_lines', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.line_timer = self.create_timer(timer_period, self.timer_callback_lines)
        self.i = 0
        self.j = 0
        self.pose_array = None
    
    def timer_callback(self):
        if self.pose_array is not None:
            self.publisher_.publish(self.points)
            
            print("pub pos \n")
            self.i += 1
    
    def timer_callback_lines(self):
        if self.pose_array is not None:
            self.publisher_lines.publish(self.lines)
            print("pub lines \n")
            self.j += 1

    def set_points(self, point_array):

        self.points = MarkerArray()
        self.lines = MarkerArray()
        self.pose_array = point_array
        self.points_from_poses = []
        self.points_from_poses = point_array.poses
        for i in range(len(self.points_from_poses)):
            point = Marker()

            point.type = Marker.SPHERE
            point.header.frame_id = "map"
            point.id = i
            point.action = Marker.ADD
            point.pose.position.x = self.points_from_poses[i].position.x
            point.pose.position.y = self.points_from_poses[i].position.y
            point.pose.position.z = self.points_from_poses[i].position.z
            point.pose.orientation.x = 0.0
            point.pose.orientation.y = 0.0
            point.pose.orientation.z = 0.0
            point.pose.orientation.w = 1.0
            point.scale.x = 0.1
            point.scale.y = 0.1
            point.scale.z = 0.1

            point.color.r = 0.0
            point.color.g = 1.0
            point.color.b = 1.0
            point.color.a = 1.0

            self.points.markers.append(point)

        for i in range(len(self.points_from_poses) -1):
            line = Marker()
            line.type = Marker.LINE_STRIP
            line.header.frame_id = "map"
            line.id = i
            line.action = Marker.ADD
            start_point = Point()
            start_point.x = self.points_from_poses[i].position.x
            start_point.y = self.points_from_poses[i].position.y
            start_point.z = self.points_from_poses[i].position.z

            end_point = Point()
            end_point.x = self.points_from_poses[i+1].position.x
            end_point.y = self.points_from_poses[i+1].position.y
            end_point.z = self.points_from_poses[i+1].position.z

            line.scale.x = 0.01
            line.scale.y = 0.01
            line.scale.z = 0.01

            line.color.r = 1.0
            line.color.g = 1.0
            line.color.b = 0.0
            line.color.a = 1.0

            line.points.append(start_point)
            line.points.append(end_point)
            self.lines.markers.append(line)

class PublishInitialPoses(Node):
    def __init__(self):
        super().__init__('initial_publisher_node')
        self.publisher_ = self.create_publisher(MarkerArray, '/initial_positions', 10)
        self.publisher_lines = self.create_publisher(MarkerArray, '/initial_lines', 10)
        self.publisher_orientation = self.create_publisher(PoseArray, '/initial_orientations', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.line_timer = self.create_timer(timer_period, self.timer_callback_lines)
        self.orien_timer = self.create_timer(timer_period, self.timer_callback_orientation)
        self.i = 0
        self.j = 0
        self.k = 0
        self.pose_array = None
        
    def timer_callback(self):
        if self.pose_array is not None:
            self.publisher_.publish(self.points)
            
            print("pub initial pos \n")
            self.i += 1
    
    def timer_callback_lines(self):
        if self.pose_array is not None:
            self.publisher_lines.publish(self.lines)
            print("pub iniital lines \n")
            self.j += 1

    def timer_callback_orientation(self):
        if self.pose_array is not None:
            self.publisher_orientation.publish(self.pose_array)
            print("pub initial orientation \n")
            self.k += 1

    def set_points(self):
        self.pose_array = PoseArray()
        self.pose_array.poses = corrector_functions.get_points_from_file("points2.txt")
        
        self.points = MarkerArray()
        self.lines = MarkerArray()  
        self.pose_array.header = Header()
        self.pose_array.header.frame_id = "map"
        self.points_from_poses = []
        self.points_from_poses = self.pose_array.poses
        for i in range(len(self.points_from_poses)):
            point = Marker()

            point.type = Marker.SPHERE
            point.header.frame_id = "map"
            point.id = i
            point.action = Marker.ADD
            point.pose.position.x = self.points_from_poses[i].position.x
            point.pose.position.y = self.points_from_poses[i].position.y
            point.pose.position.z = self.points_from_poses[i].position.z
            point.pose.orientation.x = 0.0
            point.pose.orientation.y = 0.0
            point.pose.orientation.z = 0.0
            point.pose.orientation.w = 1.0
            point.scale.x = 0.01
            point.scale.y = 0.01
            point.scale.z = 0.01

            point.color.r = 1.0
            point.color.g = 0.0
            point.color.b = 0.0
            point.color.a = 1.0

            self.points.markers.append(point)

        for i in range(len(self.points_from_poses) -1):
            line = Marker()
            line.type = Marker.LINE_STRIP
            line.header.frame_id = "map"
            line.id = i
            line.action = Marker.ADD
            start_point = Point()
            start_point.x = self.points_from_poses[i].position.x
            start_point.y = self.points_from_poses[i].position.y
            start_point.z = self.points_from_poses[i].position.z

            end_point = Point()
            end_point.x = self.points_from_poses[i+1].position.x
            end_point.y = self.points_from_poses[i+1].position.y
            end_point.z = self.points_from_poses[i+1].position.z

            line.scale.x = 0.01
            line.scale.y = 0.01
            line.scale.z = 0.01

            line.color.r = 0.0
            line.color.g = 1.0
            line.color.b = 0.0
            line.color.a = 1.0

            line.points.append(start_point)
            line.points.append(end_point)
            self.lines.markers.append(line)

def main(args=None):
    rclpy.init(args=args)

    test_client = TestClient()
    test_client.send_request()

    pub_or_to_rviz = PublishOrientationNode()
    pub_pos_to_rviz = PublishPositionNode()
    pub_initial_to_rviz = PublishInitialPoses()

    while rclpy.ok():
        print("hello")
        rclpy.spin_once(test_client) #test code is stopping here
        print("test")
        if test_client.future.done():
            try:
                response = test_client.future.result()
            except Exception as e:
                test_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                print(response.corrected_poses)
                print("points corrected!")
                global point_array
            
                point_array = response.corrected_poses
                pub_or_to_rviz.set_pose_array(point_array)
                pub_pos_to_rviz.set_points(point_array)
                pub_initial_to_rviz.set_points()
                
            break
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(pub_or_to_rviz)
    executor.add_node(pub_pos_to_rviz)
    executor.add_node(pub_initial_to_rviz)
    executor.spin()
    executor.shutdown()

    test_client.destroy_node()
    pub_or_to_rviz.destroy_node()
    pub_pos_to_rviz.destroy_node()
    pub_initial_to_rviz.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
