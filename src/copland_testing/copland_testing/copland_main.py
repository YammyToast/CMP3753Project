from enum import Enum
import uuid
uuid = uuid.uuid4()
import time

from rclpy.node import Node
from coplandnavi_interfaces.srv import SetupArgs
import rclpy

class CoplandMain(Node):
    def __init__(self, __id: str=uuid, __random_state: int=0):
        super().__init__('copland_main')
        self.id = __id 
        self.random_state = __random_state
        self.setup_service = self.create_service(SetupArgs, 'setup_args', callback=self.callback_setup_args)

    def callback_setup_args(self, __request, __response: SetupArgs):
        self.get_logger().info('Incoming request: {}'.format(__request))
        __response.copland_id = self.id
        __response.start_time = time.time()
        __response.random_state = self.random_state

        return __response

def main(args=None):
    rclpy.init()
    copland_main = CoplandMain()
    rclpy.spin(copland_main)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
