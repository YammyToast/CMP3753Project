from rclpy.node import Node
from coplandnavi_interfaces.srv import NaviArgs
import rclpy


class NaviNavigation(Node):

    def __init__(self):
        super().__init__("navi_navigation")
        # ====== Navi Setup Args
        self.setup_cli = self.create_client(NaviArgs, "navi_args")
        while not self.setup_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("navi_main service not found. waiting...")
        self.setup_req = NaviArgs.Request()
        self.setup_args = self.send_setup_args_request()
        # ====== DONE

    def send_setup_args_request(self):
        self.setup_req.node_name = "navi_navigation"
        self.setup_future = self.setup_cli.call_async(self.setup_req)
        rclpy.spin_until_future_complete(self, self.setup_future)
        # Await future to be completed
        # Setup up DB at this point, as acknowledgement has not yet been returned.
        # This keeps the setup sync in check as the setup time is considered.
        res = self.setup_future.result()
        finding_algorithm_name = res.finding_algorithm_name
        finding_algorithm_params = res.finding_algorithm_parameters
        coverage_algorithm_name = res.coverage_algorithm_name
        coverage_algorithm_params = res.coverage_algorithm_parameters
        return self.setup_future.result()


def main(args=None):
    rclpy.init()
    navi_navigation = NaviNavigation()
    rclpy.spin(navi_navigation)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
