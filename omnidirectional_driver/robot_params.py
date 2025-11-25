from dataclasses import dataclass, field
from typing import List

@dataclass
class RobotParams:
    """
    Data structure to hold robot physical parameters.
    Attributes are typed and initialized to empty/zero values.
    """
    # Physical Descriptions
    wheel_names: List[str] = field(default_factory=list)
    robot_radius: float = 0.0
    wheel_radius: float = 0.0
    
    # Geometry Configuration (Input from Config)
    wheel_angles_deg: List[float] = field(default_factory=list)
    roller_angle_deg: float = 0.0
    
    # Computed Math Values (Radians)
    phi: List[float] = field(default_factory=list)
    gamma: float = 0.0