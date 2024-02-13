from enum import Enum
from typing import List
import uuid

uuid = uuid.uuid4()
import time

from rclpy.node import Node
from coplandnavi_interfaces.srv import SetupArgs
from coplandnavi_interfaces.msg import NewSim
import rclpy

class SimResult(Enum):
    END = 0
    NOTREADY = 1



REQUIRED_NODES = ["copland_metrics", "copland_environment"]
READY = lambda x : all(elem in REQUIRED_NODES for elem in x)

class CoplandMain(Node):
    def __init__(self, __id: str = uuid, __random_state: int = 0):
        super().__init__("copland_main")
        self.id = __id
        self.random_state = __random_state
        self.setup_service = self.create_service(
            SetupArgs, "setup_args", callback=self.callback_setup_args
        )
        self.service_list: List[str] = []

        self.new_sim_publisher_ = self.create_publisher(NewSim, 'new_sim', 2)

    def callback_setup_args(self, __request, __response: SetupArgs):
        self.get_logger().info("Setup Args Request: {}".format(__request.node_name))
        __response.copland_id = str(self.id)
        __response.start_time = int(time.time())
        __response.random_state = self.random_state
        self.service_list.append(__request.node_name)
        return __response

    def publish_new_sim(self):
        if not READY(self.service_list):
            return SimResult.NOTREADY;
        return SimResult.END

def main(args=None):
    rclpy.init()
    copland_main = CoplandMain()

    while True:
        rclpy.spin_once(copland_main)
        sim_result = copland_main.publish_new_sim()
        match sim_result:
            case SimResult.END:
                break;
            case SimResult.NOTREADY:
                continue;
            case _:
                break;
    rclpy.shutdown()


if __name__ == "__main__":
    main()
