from enum import Enum
from typing import List
import uuid

node_uuid = uuid.uuid4()
import time

from rclpy.node import Node
from coplandnavi_interfaces.srv import SetupArgs
from coplandnavi_interfaces.msg import NewSim, EndSim
import rclpy




REQUIRED_NODES = ["copland_metrics", "copland_environment"]
READY = lambda x: all(elem in x for elem in REQUIRED_NODES)

class CoplandMain(Node):
    def __init__(self, __id: str = node_uuid, __random_state: int = 0):
        super().__init__("copland_main")
        self.id = __id
        self.random_state = __random_state
        self.setup_service = self.create_service(
            SetupArgs, "setup_args", callback=self.callback_setup_args
        )
        self.service_list: List[str] = []
        self.sim_counter = 0
        self.new_sim_publisher_ = self.create_publisher(NewSim, 'new_sim', 2)
        self.end_sim_subscriber = self.create_subscription(EndSim, 'end_sim', self.handle_end_sim, 2)

        self.startup_sync = self.create_timer(1, self.sync_checker)

    def sync_checker(self):
        self.get_logger().info("Waiting for Nodes...")
        if READY(self.service_list) == True:
            self.get_logger().info("Synced, sending dummy end signal.")
            dummy_msg = EndSim()
            dummy_msg.ok = True
            dummy_msg.sim_id = "null"
            self.handle_end_sim(dummy_msg)
            self.get_logger().info("Dummy signal published. Destroying sync timer.")
            self.startup_sync.destroy()

    def callback_setup_args(self, __request, __response: SetupArgs):
        self.get_logger().info("Setup Args Request: {}".format(__request.node_name))
        __response.copland_id = str(self.id)
        __response.start_time = int(time.time())
        __response.random_state = self.random_state
        self.service_list.append(__request.node_name)
        if READY(self.service_list):
            self.get_logger().info("All Required nodes found, waiting on end_sim falling edge.")
        return __response

    def publish_new_sim(self, __algorithm_coverage: str, __algorithm_pathing: str, __sim_timeout = 600):


        msg = NewSim()
        # Sim_ID is combination of session id and then an incrementing number
        msg.sim_id = str(f"{self.id}{self.sim_counter}")
        msg.sim_timeout = __sim_timeout
        msg.algorithm_coverage = __algorithm_coverage
        msg.algorithm_pathing = __algorithm_pathing
        self.get_logger().info(f"Publishing New Sim. ID: {msg.sim_id}")
        
        self.sim_counter += 1
        self.new_sim_publisher_.publish(msg)
        return True

    def handle_end_sim(self, __msg):
        self.get_logger().info(f"Ending Sim. ID: {__msg.sim_id}, Status: {__msg.ok}")
        while True:
            if self.publish_new_sim('interiorscrew', 'astar') == True:
                break;
            # self.get_logger().info("Not ready, waiting to create new sim.")


def main(args=None):
    rclpy.init()
    copland_main = CoplandMain()
    rclpy.spin(copland_main)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
