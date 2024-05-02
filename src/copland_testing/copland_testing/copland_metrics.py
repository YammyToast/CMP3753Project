from rclpy.node import Node
from coplandnavi_interfaces.srv import SetupArgs
from coplandnavi_interfaces.msg import NewSim, EndSim
import rclpy

from tinydb import TinyDB, Query
TinyDB.default_table_name = 'metrics'

from dataclasses import dataclass

"""
Proposed Metrics
----
Coverage(%)
Percentage Cleaned(%)
Seconds Elapsed(int)
Collision Rate(%)
"""

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
    """Metrics node for the automated testing plan.

    Listens on all simulation metrics broadcasting channels, grouping values based on the perceived current simulation.
    Performs calculation of evaluation metrics from the compiled data, and writes results into a session-based TinyDB JSON file.

    Each copland session will spawn a JSON database with the same id as the copland session, which contains rows each corresponding to one iteration of the simulated environment.
    The database is implemented such that each row inserted must follow the DBSchema. Which alongside the evaluation metrics, adds additional Primary and Secondary keys to the records.

    Metrics listens to start_sim and end_sim broadcasts to determine when new entries should be started. On an end_sim call, the current working dataset is flushed into the database, and a new row/dataset is created.
    """

    def __init__(self):
        super().__init__("copland_metrics")
        self.setup_cli = self.create_client(SetupArgs, "setup_args")

        self.new_sim_subscriber_ = self.create_subscription(
            NewSim, "new_sim", self.handle_new_sim, 2
        )
        self.end_sim_subscriber_ = self.create_subscription(
            EndSim, "end_sim", self.handle_end_sim, 2
        )


        while not self.setup_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("copland_main service not found. waiting...")

        # Wait for service to connect
        self.setup_req = SetupArgs.Request()
        self.setup_args = self.send_setup_args_request()
        self.get_logger().info(f"Setup Args Complete. ")


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
            f"Metrics New Sim ID: {__msg.sim_id}, Coverage: {__msg.algorithm_coverage}, Pathing: {__msg.algorithm_pathing}"
        )
        self.current_metrics_row = DBSchema(
            __msg.sim_id, __msg.algorithm_coverage, __msg.algorithm_pathing
        )

        ## REMOVE
        self.insert_db_row(self.current_metrics_row)

    def handle_end_sim(self, __msg):
        self.get_logger().info(f"End Sim. Current Metrics: {self.current_metrics_row}")

        # Complete Remaining Attributes before pushing.
        # ID, and algorithm names are added at simulation start as they are constant.
        # self.current_metrics_row ...

    def send_setup_args_request(self):
        # Replace with self node_name attribute if it exists
        self.setup_req.node_name = "copland_metrics"
        self.setup_future = self.setup_cli.call_async(self.setup_req)
        rclpy.spin_until_future_complete(self, self.setup_future)
        # Await future to be completed
        # Setup up DB at this point, as acknowledgement has not yet been returned.
        # This keeps the setup sync in check as the setup time is considered.
        copland_id = self.setup_future.result().copland_id
        db_name = f"./metrics/{copland_id}.json"
        self.get_logger().info(f"Creating metrics session database: {db_name}")
        self.db = TinyDB(db_name)
        return self.setup_future.result()


def main(args=None):
    rclpy.init()
    copland_metrics = CoplandMetrics()
    rclpy.spin(copland_metrics)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
