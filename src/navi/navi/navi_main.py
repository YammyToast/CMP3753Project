from rclpy.node import Node
from coplandnavi_interfaces.srv import SetupArgs
import rclpy


import subprocess

REQUIRED_NODES = ["navi_mapping"]
READY = lambda x: all(elem in x for elem in REQUIRED_NODES)

class NaviMain(Node):
    def __init__(self):
        super().__init__('navi_main')
        self.setup_cli = self.create_client(SetupArgs, 'setup_args')
        while not self.setup_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('copland_main service not found. waiting...')
        # Wait for service to connect
        self.setup_req = SetupArgs.Request()
        self.setup_args = self.send_setup_args_request()
        self.get_logger().info('Setup Args Complete.')

    def send_setup_args_request(self):
        # Replace with self node_name attribute if it exists
        self.setup_req.node_name = 'navi_main'
        self.setup_future = self.setup_cli.call_async(self.setup_req)
        rclpy.spin_until_future_complete(self, self.setup_future)
        return self.setup_future.result()

def main(args=None):
    rclpy.init()
    navi_main = NaviMain()
    rclpy.spin(navi_main)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
