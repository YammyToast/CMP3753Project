from rclpy.node import Node
from coplandnavi_interfaces.srv import SetupArgs
from coplandnavi_interfaces.msg import NewSim 
import rclpy

class CoplandMetrics(Node):
    def __init__(self):
        super().__init__('copland_metrics')
        self.setup_cli = self.create_client(SetupArgs, 'setup_args')

        while not self.setup_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('copland_main service not found. waiting...')

        # Wait for service to connect
        self.setup_req = SetupArgs.Request()
        self.setup_args = self.send_setup_args_request()
        self.get_logger().info('Setup Args Complete.')
        self.new_sim_subscriber_ = self.create_subscription(NewSim, 'new_sim', self.handle_new_sim, 2)

    def handle_new_sim(self, __msg):
        self.get_logger().info(f'New Message: {__msg}')

    def send_setup_args_request(self):
        # Replace with self node_name attribute if it exists
        self.setup_req.node_name = 'copland_metrics'
        self.setup_future = self.setup_cli.call_async(self.setup_req)
        rclpy.spin_until_future_complete(self, self.setup_future)
        return self.setup_future.result()

"""
Proposed Metrics
----
Coverage(%)
Percentage Cleaned(%)
Seconds Elapsed(int)
Collision Rate(%)
"""

"""
Group into log files: Each log file represents one simulation.
File name indicates algorithm parameters, i.e. path coverage and pathing algorithms.

astar-interiorscrew-simid.log

---Contents---
filename: 'astar-interiorscrew-123456.log'
sim_id: '123456'
algorithm_pathing: 'astar'
algorithm_coverage: 'interiorscrew'
sim_start: unix-timestamp
sim_end: unix-timestamp
coverage: 50%
percentage_cleaned: 50%
collision_rate: 50%

"""


def main(args=None):
    rclpy.init()
    copland_metrics = CoplandMetrics()
    rclpy.spin(copland_metrics)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
