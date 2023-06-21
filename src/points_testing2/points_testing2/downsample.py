#! /usr/bin/env python3
#A node that sets up a service to downsample a set of poses
#Author: Samantha Smith, smith.15485@osu.edu

import rclpy
from rclpy.node import Node
from services.srv import DownsampleSrv
from points_testing2 import corrector_functions


class DownsampleService(Node):
    def __init__(self):
        super().__init__('downsample_service')
        self.srv = self.create_service(DownsampleSrv, 'downsample', self.downsample_callback)
    
    def downsample_callback(self, request, response):
        self.get_logger().info('Incoming request\n')
        response.corrected_poses = corrector_functions.downsample(request.epsilon, request.angle_threshold, request.initial_poses)
        print("Complete! Response Sent \n")
        return response
    
def main(args=None):
    rclpy.init(args=args)
    
    downsample_service = DownsampleService()
    
    rclpy.spin(downsample_service)
    
    downsample_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

