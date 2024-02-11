import rclpy
from rclpy.node import Node
from enum import Enum

from coplandnavi_interfaces.msg import SetupArgs

class State(Enum):
    

class CoplandMain(Node):
    def __init__(self):
        super().__init__('copland_main')
        self.setup_publisher_ = self.create_publisher(SetupArgs, 'setup_args', 1)
        self.setup_timer = self.create_timer(0.5)

def main(args=None):
    copland_main = CoplandMain()
    rclpy.spin(copland_main)

if __name__ == '__main__':
    main()
