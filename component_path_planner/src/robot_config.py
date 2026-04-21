from enum import Enum
import math

class RobotType(Enum):
    circle = 0
    rectangle = 1

class Robot():
    def __init__(self):
        # ---------------------------------------------------------
        # 1. Kinematics (Fixing the Overshoot)
        # ---------------------------------------------------------
        self.max_speed             = 2.5  # [m/s] 
        self.min_speed             = 0  # [m/s] 
        self.max_accel             = 2.5   # [m/ss] 

        # Increased slightly so it doesn't take a full minute to turn 180 degrees
        self.max_yaw_rate          = 1.0 * math.pi / 180.0  # [rad/s]
        self.max_delta_yaw_rate    = 5.0 * math.pi / 180.0  # [rad/ss] 

        self.v_resolution          = 0.01  # [m/s]
        self.yaw_rate_resolution   = 0.1 * math.pi / 180.0  # [rad/s]
        
        # ---------------------------------------------------------
        # 2. Prediction 
        # ---------------------------------------------------------
        self.dt                    = 0.1   # [s] 
        self.predict_time          = 3.0   # [s] Reduced slightly so it focuses on immediate heading alignment

        # ---------------------------------------------------------
        # 3. Cost Function Gains (Prioritizing Heading)
        # ---------------------------------------------------------
        # INCREASED: Make the robot care 4x more about looking at the path 
        # than it cares about driving fast.
        self.to_goal_cost_gain     = 1.0  
        
        # DECREASED: Allow the robot to slow down to make an accurate turn
        self.speed_cost_gain       = 1.0   
        
        self.obstacle_cost_gain    = 1.0
        self.robot_stuck_flag_cons = 0.001 
        
        # ---------------------------------------------------------
        # 4. Robot Geometry
        # ---------------------------------------------------------
        self.robot_type            = RobotType.rectangle

        self.robot_width  = 0.35  # [m] 
        self.robot_length = 0.50  # [m] 
        self.robot_radius = 0.40  # [m]

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value