from rclpy.node import Node
from coplandnavi_interfaces.srv import SetupArgs
from coplandnavi_interfaces.msg import NewSim, EndSim
import rclpy

from tinydb import TinyDB, Query

from dataclasses import dataclass

"""
Use TinyDB to store all logs in one file.
This will make future data-analysis much easier. A database also just makes more sense.

------
Schema
------

| Column Name | Sim-ID | Alg-Pathing | Alg-Coverage | Sim-Start | Sim-End | Coverage | Percentage-Cleaned | Collision-Rate |
|-------------|--------|-------------|--------------|-----------|---------|----------|--------------------|----------------|
| Data Type   | String | String      | String       | Int       | Int     | Float    | Float              | Float          |

"""


@dataclass
class DBSchema:
    sim_id: str
    algorithm_pathing: str
    algorithm_coverage: str
    sim_start: int = 1
    sim_end: int = -1
    coverage: float = -1.0
    percentage_cleaned: float = -1.0
    collision_rate: float = -1.0


class CoplandMetrics(Node):
    def __init__(self):
        super().__init__("copland_metrics")
        self.setup_cli = self.create_client(SetupArgs, "setup_args")

        while not self.setup_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("copland_main service not found. waiting...")

        # Wait for service to connect
        self.setup_req = SetupArgs.Request()
        self.setup_args = self.send_setup_args_request()
        self.get_logger().info(f"Setup Args Complete. ")
        self.new_sim_subscriber_ = self.create_subscription(
            NewSim, "new_sim", self.handle_new_sim, 2
        )
        self.end_sim_subscriber_ = self.create_subscription(
            EndSim, "end_sim", self.handle_end_sim, 2
        )


    def insert_db_row(self, __db_row: DBSchema):
        try:
            self.db.insert(
                {
                    "sim_id": __db_row.sim_id,
                    "algorithm_pathing": __db_row.algorithm_pathing,
                    "algorithm_coverage": __db_row.algorithm_coverage,
                    "sim_start": __db_row.sim_start,
                    "sim_end": __db_row.sim_end,
                    "coverage": __db_row.coverage,
                    "percentage_cleaned": __db_row.percentage_cleaned,
                    "collision_rate": __db_row.collision_rate,
                }
            )
            return True
        except Exception as e:
            self.get_logger().error(e)
            return False

    def handle_new_sim(self, __msg):
        # self.get_logger().info(f'New Sim: {__msg}')
        self.get_logger().info(
            f"Starting New Sim. ID: {__msg.sim_id}, Coverage: {__msg.algorithm_coverage}, Pathing: {__msg.algorithm_pathing}"
        )
        self.current_metrics_row = DBSchema(
            __msg.sim_id,
            __msg.algorithm_coverage,
            __msg.algorithm_pathing
        )

        ## REMOVE
        self.insert_db_row(self.current_metrics_row)

    def handle_end_sim(self, __msg):
        self.get_logger().info(f"End Sim. Current Metrics: {self.current_metrics_row}")

    def send_setup_args_request(self):
        # Replace with self node_name attribute if it exists
        self.setup_req.node_name = "copland_metrics"
        self.setup_future = self.setup_cli.call_async(self.setup_req)
        rclpy.spin_until_future_complete(self, self.setup_future)
        # Await future to be completed
        # Setup up DB at this point, as acknowledgement has not yet been returned.
        # This keeps the setup sync in check as the setup time is considered.
        copland_id = self.setup_future.result().copland_id
        db_name = f"{copland_id}.json"
        self.get_logger().info(f"Creating metrics session database: {db_name}")
        self.db = TinyDB(db_name)
        return self.setup_future.result()


"""
Proposed Metrics
----
Coverage(%)
Percentage Cleaned(%)
Seconds Elapsed(int)
Collision Rate(%)
"""


def main(args=None):
    rclpy.init()
    copland_metrics = CoplandMetrics()
    rclpy.spin(copland_metrics)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
