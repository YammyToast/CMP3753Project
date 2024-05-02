from rclpy.node import Node
from coplandnavi_interfaces.srv import NaviArgs
import rclpy
from std_msgs.msg import UInt8, String
from .states import (
    States,
    Coins,
    state_qos,
    PathCoverageAlgorithms,
    PathFindingAlgorithms,
    StateError,
)
import subprocess, os, sys
current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.append(parent)


class NaviNavigation(Node):

    def __init__(self):
        super().__init__("navi_navigation")
        self.current_action = None
        # ====== Navi Setup Args
        self.setup_cli = self.create_client(NaviArgs, "navi_args")
        while not self.setup_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("navi_main service not found. waiting...")
        self.setup_req = NaviArgs.Request()
        self.setup_args = self.send_setup_args_request()
        # ====== Navi State Channels
        self.create_subscription(
            UInt8,
            "navi_state",
            callback=self.handle_state_change,
            qos_profile=state_qos,
        )
        self._coins_publisher = self.create_publisher(
            UInt8, "navi_coins", qos_profile=state_qos
        )
        # ==== Action Time and Variables
        self.create_timer(1, self.action_timer)

    def send_setup_args_request(self):
        self.setup_req.node_name = "navi_navigation"
        self.setup_future = self.setup_cli.call_async(self.setup_req)
        rclpy.spin_until_future_complete(self, self.setup_future)
        # Await future to be completed
        # Setup up DB at this point, as acknowledgement has not yet been returned.
        # This keeps the setup sync in check as the setup time is considered.
        res = self.setup_future.result()
        self.finding_algorithm = res.finding_algorithm
        self.finding_algorithm = res.finding_algorithm_parameters
        self.coverage_algorithm = res.coverage_algorithm
        self.coverage_algorithm = res.coverage_algorithm_parameters
        return self.setup_future.result()

    def handle_state_change(self, __state: UInt8):
        try:
            self.get_logger().info(f"Received Nav State: {__state.data}")
            match __state.data:
                case States.INIT.value:
                    self.current_action = self.execute_init
                case _:
                    raise StateError
        except StateError:
            self.get_logger().error(f"Unknown State: {__state.data}")
        except Exception as e:
            self.get_logger().error(f"Unknown State Error: {e}")

    def execute_init(self):
        try:

            self.get_logger().info("Initializing Turtlebot3 Navigation")
            # ==== Initialize Turtlebot3 Navigation Instance
            self.cartographer_handle = subprocess.Popen(["ros2", "launch", "turtlebot3_cartographer", "cartographer.launch.py", "use_sim_time:=True"])
            self.nav2_handle = subprocess.Popen(["ros2", "launch", "turtlebot3_navigation2", "navigation2.launch.py", "use_sim_time:=true"])
            self.get_logger().info(f"cartographer handle: {self.cartographer_handle}")
            self.get_logger().info(f"nav2 handle: {self.nav2_handle}")

            self.current_action = None
        except Exception as e:
            self.get_logger().error(f"Error Occurred in Navigation Initialization: {e}")

    def action_timer(self):
        try:
            # No behavior escape
            if self.current_action == None:
                return
            result = self.current_action()
            if result != None:
                self.get_logger().info(
                    f"Action Loop with action: {self.current} returned: {result}"
                )
        except Exception as e:
            self.get_logger().error(f"Error in Action Timer: {e}")


def main(args=None):
    rclpy.init()
    navi_navigation = NaviNavigation()
    rclpy.spin(navi_navigation)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
