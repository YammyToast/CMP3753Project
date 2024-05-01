from enum import Enum
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

# ==============================================
# State Management
# ==============================================
class States(Enum):
    INIT=0
    EVALUATE=1
    PATH_FINDING=2

class Coins(Enum):
    INIT_COMPLETE=1
    #===============#
    INIT_FAIL=-1

# ==============================================
# Algorithms
# ==============================================
class PathFindingAlgorithms(Enum):
    WEIGHTED_A_STAR=0

class PathCoverageAlgorithms(Enum):
    INTERIORSCREW=0

state_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)