from rclpy.node import Node
from coplandnavi_interfaces.srv import SetupArgs
import rclpy
import subprocess

from .states import States, Coins, state_qos
from std_msgs.msg import UInt8

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

        self._state_publisher = self.create_publisher(UInt8, "navi_state", state_qos)

        # INITIALIZE STATE!
        if self.change_state(States.INIT) == False:
            raise Exception("Could not initialize base state.")


    def change_state(self, __state: States) -> bool:
        try:
            msg = UInt8()
            msg.data = __state.value
            self._state_publisher.publish(msg)
            self.get_logger().info(f"Changed state to: {States._member_names_[__state.value]}")
            return True
        except Exception as e:
            return False


    def handle_state_change(self, __coin: Coins):
        try:
            match __coin:
                case Coins.INIT_COMPLETE:
                    self.get_logger().info("Completed Initialization")
                
                #==========#
                case Coins.INIT_FAIL:
                    raise Exception("Navi Initialization returned fail.")
                case _:
                    self.get_logger().info("Unknown state, resolving to States.Evaluate")

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def send_setup_args_request(self):
        # Replace with self node_name attribute if it exists
        self.setup_req.node_name = 'navi_main'
        self.setup_future = self.setup_cli.call_async(self.setup_req)
        rclpy.spin_until_future_complete(self, self.setup_future)
        return self.setup_future.result()

    def action_timer(self):
        pass

def main(args=None):
    rclpy.init()
    navi_main = NaviMain()
    rclpy.spin(navi_main)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
