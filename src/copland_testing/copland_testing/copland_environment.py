from rclpy.node import Node
from coplandnavi_interfaces.srv import SetupArgs
import rclpy

from coplandnavi_interfaces.msg import NewSim, EndSim

import subprocess, os


class CoplandEnvironment(Node):
    def __init__(self):
        super().__init__('copland_environment')
        self.setup_cli = self.create_client(SetupArgs, 'setup_args')

        self.new_sim_subscriber_ = self.create_subscription(
            NewSim, "new_sim", self.handle_new_sim, 2
        )
        self.end_sim_subscriber_ = self.create_subscription(
            EndSim, "end_sim", self.handle_end_sim, 2
        )


        while not self.setup_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('copland_main service not found. waiting...')

        # Wait for service to connect
        self.setup_req = SetupArgs.Request()
        self.setup_args = self.send_setup_args_request()
        self.get_logger().info('Setup Args Complete.')

    def send_setup_args_request(self):
        # Replace with self node_name attribute if it exists
        self.setup_req.node_name = 'copland_environment'
        self.setup_future = self.setup_cli.call_async(self.setup_req)
        rclpy.spin_until_future_complete(self, self.setup_future)
        return self.setup_future.result()

    def handle_new_sim(self, __msg):
        try:
                
            # ==== Set Gazebo Models
            os.environ["GAZEBO_MODEL_PATH"] = "`ros2 pkg prefix turtlebot3_gazebo`/share/turtlebot3_gazebo/models/"
            os.environ["TURTLEBOT3_MODEL"] = "burger"     
            os.environ["ROS_DOMAIN_ID"] = "1"
            # ==== Environment Launch        
            self.sim_handle = subprocess.Popen(["ros2", "launch", "turtlebot3_gazebo", "turtlebot3_world.launch.py"])       
            # ==== END
            self.get_logger().info("Setting up Environment Exports.")

            self.get_logger().info(f"sim_handle: {self.sim_handle}")
        except Exception as e:
            self.get_logger().error(f"Error creating new sim: {e}")

    def handle_end_sim(self, __msg):
        self.get_logger().info(f"End Sim: {__msg}")


def main(args=None):
    rclpy.init()
    copland_environment = CoplandEnvironment()
    rclpy.spin(copland_environment)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
