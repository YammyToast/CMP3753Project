from enum import Enum
from typing import List
import uuid

node_uuid = uuid.uuid4()
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
    def __init__(self, __id: str = node_uuid, __random_state: int = 0):
        super().__init__("copland_main")
        self.id = __id
        self.random_state = __random_state
        self.setup_service = self.create_service(
            SetupArgs, "setup_args", callback=self.callback_setup_args
        )
        self.service_list: List[str] = []

        self.new_sim_publisher_ = self.create_publisher(NewSim, 'new_sim', 2)
        self.sim_ids: List[str] = []

    def callback_setup_args(self, __request, __response: SetupArgs):
        self.get_logger().info("Setup Args Request: {}".format(__request.node_name))
        __response.copland_id = str(self.id)
        __response.start_time = int(time.time())
        __response.random_state = self.random_state
        self.service_list.append(__request.node_name)
        return __response

    def publish_new_sim(self, __sim_timeout=600):
        self.get_logger().info("Starting Publish")
        if not READY(self.service_list):
            return SimResult.NOTREADY;
    
        msg = NewSim()
        msg.sim_id = str(uuid.uuid4())
        msg.sim_timeout = __sim_timeout
        msg.algorithm_specifier = "temp"
        self.new_sim_publisher_.publish(msg)
        self.get_logger().info("PUBLISHED! : {}".format(msg.sim_id))
        return SimResult.END

def main(args=None):
    rclpy.init()
    copland_main = CoplandMain()
    while True:
        rclpy.spin_once(copland_main)
        sim_result = copland_main.publish_new_sim()
        time.sleep(5)
        match sim_result:
            case SimResult.END:
                continue;
                # break;
            case SimResult.NOTREADY:
                continue;
            case _:
                break;
    rclpy.shutdown()


if __name__ == "__main__":
    main()
