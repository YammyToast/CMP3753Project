from enum import Enum
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class States(Enum):
    INIT=0
    EVALUATE=1
    PATH_FINDING=2

class Coins(Enum):
    INIT_COMPLETE=1
    #===============#
    INIT_FAIL=-1

state_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)