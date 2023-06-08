#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from services.srv import DownsampleSrvInfo

class TestClient(Node): 
    def __init__(self):
        super().__init__('test_client')
        self.cli = self.create_client(DownsampleSrvInfo, 'downsample')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DownsampleSrvInfo.Request()

    def send_request(self):
        self.req.t = 10
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    test_client = TestClient()

    test_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(test_client)
        if test_client.future.done():
            try:
                response = test_client.future.result()
            except Exception as e:
                test_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                test_client.get_logger().info(
                    'Result of test')
            break

    test_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
