from rclpy.node import Node
from coplandnavi_interfaces.srv import SetupArgs, NaviArgs
import rclpy
import subprocess
from typing import List
import time

from .states import (
    States,
    Coins,
    state_qos,
    PathFindingAlgorithms,
    PathCoverageAlgorithms,
)
from std_msgs.msg import UInt8, String

REQUIRED_NODES = ["navi_navigation"]
READY = lambda x: all(elem in x for elem in REQUIRED_NODES)


class NaviMain(Node):
    def __init__(self):
        super().__init__("navi_main")
        # ===== Setup Args
        self.setup_cli = self.create_client(SetupArgs, "setup_args")
        while not self.setup_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("copland_main service not found. waiting...")
        # Wait for service to connect
        self.setup_req = SetupArgs.Request()
        self.setup_args = self.send_setup_args_request()
        # ===== Navi Args
        self.service_list: List[str] = []
        self.navi_args_service = self.create_service(
            NaviArgs, "navi_args", callback=self.navi_args_callback
        )
        self.navi_args_sync = self.create_timer(1, self.sync_checker)
        # ===== DONE

        # ===== Default / Algorithm Specification
        self.path_finding_algorithm = PathFindingAlgorithms.WEIGHTED_A_STAR
        self.path_coverage_algorithm = PathCoverageAlgorithms.INTERIORSCREW

        self.get_logger().info("Setup Args Complete.")

        # ==== Navi States
        self._state_publisher = self.create_publisher(UInt8, "navi_state", state_qos)
        self.create_subscription(UInt8, "navi_coins", callback=self.handle_state_change, qos_profile=state_qos)



    def sync_checker(self):
        self.get_logger().info("Navi Waiting for Nodes...")
        if READY(self.service_list):
            self.get_logger().info("All required navigation nodes found.")
            # INITIALIZE STATE!
            if self.change_state(States.INIT) == False:
                raise Exception("Could not initialize base state.")
            self.navi_args_sync.destroy()

    def change_state(self, __state: States) -> bool:
        try:
            msg = UInt8()
            msg.data = __state.value
            self._state_publisher.publish(msg)
            self.get_logger().info(
                f"Changed state to: {States._member_names_[__state.value]}"
            )
            return True
        except Exception as e:
            return False

    def handle_state_change(self, __coin: Coins):
        try:
            match __coin:
                case Coins.INIT_COMPLETE:
                    self.get_logger().info("Completed Initialization")

                # ========== #
                case Coins.INIT_FAIL:
                    raise Exception("Navi Initialization returned fail.")
                case _:
                    self.get_logger().info(
                        "Unknown state, resolving to States.Evaluate"
                    )

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def send_setup_args_request(self):
        # Replace with self node_name attribute if it exists
        self.setup_req.node_name = "navi_main"
        self.setup_future = self.setup_cli.call_async(self.setup_req)
        rclpy.spin_until_future_complete(self, self.setup_future)
        return self.setup_future.result()

    def action_timer(self):
        pass

    def navi_args_callback(self, __request, __response: NaviArgs):
        self.get_logger().info("Navi Args Request: {}".format(__request.node_name))
        try:
            # ==== Response Building
            __response.finding_algorithm = int(self.path_finding_algorithm.value)
            __response.coverage_algorithm = int(self.path_coverage_algorithm.value)
            # ==== Add client to service list
            self.service_list.append(__request.node_name)
            # ==== Send Response
            return __response
        except Exception as e:
            self.get_logger().error(f"Err: {e}, Likely Unknown path-finding algorithm specified.")

def main(args=None):
    rclpy.init()
    navi_main = NaviMain()
    rclpy.spin(navi_main)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
