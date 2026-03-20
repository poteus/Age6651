import commands2
import commands2.sysid
from phoenix6 import hardware, configs, signals, controls
from wpilib import SmartDashboard, DriverStation
from wpimath.units import seconds
import math
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from telemetry import Telemetry

class Turret(commands2.Subsystem):
    '''
    The turret subsystem controls the yaw of the shooter (subsystems.shooter)
    '''

    LOWER_LIMIT = 0
    UPPER_LIMIT = 1


    blue_hub = Pose2d(0.0, 5.5, 0.0) # Update coordinates based on Alliance
    red_hub = Pose2d(16.55, 8.05, 0.0) # Update coordinates based on Alliance
    top_blue_corner = Pose2d(0.0, 8.05, 0.0) # Update coordinates based on Alliance
    top_red_corner = Pose2d(16.55, 0.0, 0.0) # Update coordinates based on Alliance
    bottom_blue_corner = Pose2d(0.0, 0.0, 0.0) # Update coordinates based on Alliance
    bottom_red_corner = Pose2d(16.55, 8.05, 0.0) # Update coordinates based on Alliance

    def __init__(self, _telemetry: Telemetry):
        super().__init__()

        # Team Color and Hub Position Initialization --------------------------------------------------------------------------------
        self.TEAM_COLOR: str = ""
        _team_color = DriverStation.getAlliance()

        if _team_color == DriverStation.Alliance.kRed:
            self.TEAM_COLOR = "Red"

        else: # Blue is default just in case
            self.TEAM_COLOR = "Blue"

        self.location_state = 1 # 1 is alliance color and current alliance region the same, 0 is neutral, -1 is the opposing team.
        # ---------------------------------------------------------------------------------------------------------------------------

        self.telemetry = _telemetry

        # Gear Ratio: 200 teeth / 20 teeth = 10
        # 10 rotations of the Kraken = 1 rotation of the Turret
        GEAR_RATIO = 10.0 
        CAN_ID_MOTOR = 15
        CAN_ID_ENCODER = 5

        # Initialize Kraken X60 (Talon FX)
        # Assuming CAN ID 15 and on the "rio" or "CANDrive" bus
        self.motor = hardware.TalonFX(CAN_ID_MOTOR)
        self.abs_encoder = hardware.CANcoder(CAN_ID_ENCODER)

        # Config absolute encoder settings
        enc_cfg = configs.CANcoderConfiguration()
        # Ensure 1.0 on CANcoder = 1.0 rotation of the Turret
        # Since it's geared 1:1 with the turret (on the opposite side), ratio is 1.0
        enc_cfg.magnet_sensor.sensor_direction = signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        
        # TO DO: Add your magnet offset here after physically aligning
        enc_cfg.magnet_sensor.magnet_offset = 0.258545
        self.abs_encoder.configurator.apply(enc_cfg)

        # Configure Motor Settings
        cfg = configs.TalonFXConfiguration()
        cfg.feedback.sensor_to_mechanism_ratio = GEAR_RATIO

        # Soft Limits: Prevent rotating past 360 degrees (0.0 to 1.0 rotations)
        # This protects cables!
        cfg.software_limit_switch.forward_soft_limit_threshold = self.UPPER_LIMIT
        cfg.software_limit_switch.forward_soft_limit_enable = True
        cfg.software_limit_switch.reverse_soft_limit_threshold = self.LOWER_LIMIT
        cfg.software_limit_switch.reverse_soft_limit_enable = True

        # Reverse the motor
        cfg.motor_output.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE # or COUNTER_CLOCKWISE_POSITIVE

        # PID Settings for Position Control
        cfg.slot0.k_p = 25 # Placeholder: start low and tune
        cfg.slot0.k_i = 0.0
        cfg.slot0.k_d = 2.5
        cfg.slot0.k_s = 0.3  # Static feedforward (Amps)
        cfg.slot0.k_v = 0.12  # Velocity feedforward (Amps per rotation/sec)
        cfg.slot0.k_a = 0 # Acceleration feedforward (Amps

        # Motion Magic Config
        mm_cfg = cfg.motion_magic
        # Units are rotations of the TURRET (since GEAR_RATIO is applied)
        mm_cfg.motion_magic_cruise_velocity = 2.0 
        mm_cfg.motion_magic_acceleration = 2.5   
        mm_cfg.motion_magic_jerk = 20.0

        # Physical Model
        mm_cfg.motion_magic_expo_k_v = 0.12 # Volts per rotation/sec
        mm_cfg.motion_magic_expo_k_a = 0.01 # Volts per rotation/sec^2

        # Apply the configuration
        self.motor.configurator.apply(cfg)

        # Control Requests
        self.position_request = controls.MotionMagicVoltage(0)
        self.voltage_request = controls.VoltageOut(0)

        # The 'Magic Number' to align your -0.3 reading to a 0.75 turret position
        # (Target Laps * Ratio) - Current Reading 
        # 0.75 - (-0.3/10) = 0.75 - (-0.03) = 0.75 + 0.03 = 0.78
        SENSOR_OFFSET = 0.78

        # 1. Get the current reading from the absolute encoder
        # This is currently -0.3
        raw_abs_val = self.abs_encoder.get_absolute_position().refresh().value

        # 2. Calculate the seeded value
        # -0.03 + 0.78 = 0.75
        seeded_value = raw_abs_val/10 + SENSOR_OFFSET

        # 3. Seed the Motor
        # Since your motor also has a 10:1 ratio set in its config,
        # setting it to 0.75 rotations will correctly represent 0.75 Turret Laps.
        self.motor.set_position(seeded_value)
        self.set_position(.75)

    def set_position(self, rotations: float):
        """Sets turret position (0.0 to 1.0 represents 0 to 360 degrees)"""

        # print(rotations, rotations * 360)

        if self.LOWER_LIMIT > rotations:
            rotations = self.LOWER_LIMIT
        elif rotations > self.UPPER_LIMIT:
            rotations = self.UPPER_LIMIT

        self.motor.set_control(self.position_request.with_position(rotations))

    def stop(self):
        self.motor.stopMotor()

    def reset_position(self):
        """Manually reset the encoder to 0. Call this when turret is at 'home'"""
        self.motor.set_position(0)

    def get_turret_rotation(self) -> float:
        """Returns current turret position in rotations (0 to 1.0)"""
        return self.motor.get_position().value
    
    def aim_at_hub(self, robot_pose: Pose2d):
        ''' Calculates the angle to the hub and moves the turret. 
        Assumes 0 rotations = facing right (positive X) and positive rotations are counterclockwise. '''

        # Vector from robot to hub (x,y)
        target_vector = self.HUB_POSITION - robot_pose.translation()
        
        # Field angle (0 is Right in your system, so we adjust WPILib's atan2)
        # WPILib atan2: 0 is Forward (+X). 
        # To make 0 "Right", we subtract 90 degrees.
        target_field_angle = math.atan2(target_vector.y, target_vector.x) - (math.pi / 2)
        
        robot_heading = robot_pose.rotation().radians()
        relative_angle = target_field_angle - robot_heading
        
        # Normalize
        while relative_angle > math.pi: relative_angle -= 2 * math.pi
        while relative_angle < -math.pi: relative_angle += 2 * math.pi

        # Convert to rotations
        target_rotations = relative_angle / (2 * math.pi)
        
        # Clamp to your -0.25 to 0.25 range to avoid hitting soft limits
        target_rotations = max(self.LOWER_LIMIT, min(self.UPPER_LIMIT, target_rotations))
        
        # Checks if the target is on the corret side of the robot and within the soft limits before moving
        if self.LOWER_LIMIT <= target_rotations <= self.UPPER_LIMIT:
            # Within domain: Move to target
            self.set_position(target_rotations)
        else:
            # Outside domain: Do nothing (or you could call self.stop())
            # This prevents the motor from 'hunting' for a target it can't reach.
            # self.stop()
            pass


    def aim_at_angle(self, target_angle_degrees: float):
        ''' AIms the turret to the angle if possible, otherwise does nothing.
        Angles need to be in the domain of the turret. '''

        # Convert angle to rotations (assuming 0 degrees = facing right and positive is counterclockwise)
        rotation_aim = target_angle_degrees / 360.0
        
        # Normalize to [-0.5, 0.5] range (i.e., -180 to 180 degrees)
        while rotation_aim > 1: rotation_aim -= 1
        while rotation_aim < 0: rotation_aim += 1
        
        # Checks if the target is on the corret side of the robot and within the soft limits before moving
        if self.LOWER_LIMIT <= rotation_aim <= self.UPPER_LIMIT:
            # Within domain: Move to target
            self.set_position(rotation_aim)
        else:
            # Outside domain: Do nothing (or you could call self.stop())
            # This prevents the motor from 'hunting' for a target it can't reach.
            # self.stop()
            pass

    def aim_to_position(self, position: Pose2d, robot_pose: Pose2d):
        ''' Calculates the angle to the hub and moves the turret. 
        Assumes 0 rotations = facing right (positive X) and positive rotations are counterclockwise. '''

        # Vector from robot to hub (x,y)
        target_vector = position.translation() - robot_pose.translation()
        
        # Field angle (0 is Right in your system, so we adjust WPILib's atan2)
        # WPILib atan2: 0 is Forward (+X). 
        # To make 0 "Right", we subtract 90 degrees.
        target_field_angle = math.atan2(target_vector.y, target_vector.x) - (math.pi / 2)
        
        robot_heading = robot_pose.rotation().radians()
        relative_angle = target_field_angle - robot_heading
        
        # Normalize
        while relative_angle > math.pi: relative_angle -= 2 * math.pi
        while relative_angle < -math.pi: relative_angle += 2 * math.pi

        # Convert to rotations
        target_rotations = relative_angle / (2 * math.pi)
        
        # Clamp to your -0.25 to 0.25 range to avoid hitting soft limits
        target_rotations = max(self.LOWER_LIMIT, min(self.UPPER_LIMIT, target_rotations))
        
        # Checks if the target is on the corret side of the robot and within the soft limits before moving
        if self.LOWER_LIMIT <= target_rotations <= self.UPPER_LIMIT:
            # Within domain: Move to target
            self.set_position(target_rotations)
        else:
            # Outside domain: Do nothing (or you could call self.stop())
            # This prevents the motor from 'hunting' for a target it can't reach.
            # self.stop()
            pass   

    def periodic(self):
        """ Placeholder for autorotation, Aim at hub or team corners depending on the position of the robot on the field.
        If on the opposing team's side, aim for our team's corner. If on our side/neutral zone, aim for our hub. """
        
        current_robot_pose: Pose2d = self.telemetry._drive_pose_subscriber.get()
        x_location = current_robot_pose.translation().x
        y_location = current_robot_pose.translation().y

        self.telemetry._turret_rotation_pub.set(self.motor.get_position().value)
        self.telemetry._turret_encoder_pub.set(self.abs_encoder.get_absolute_position().value)

        # team = None

        # if DriverStation.Alliance.kBlue: 
        #     team = self.blue_hub
        # else:
        #     team = self.red_hub

        # distance_to_hub = math.dist(current_robot_pose, team)



        # if self.TEAM_COLOR == "Blue":
        #     if x_location < 3.5: 
        #         # If we are on the team side
                
        #         self.aim_to_position(self.blue_hub, current_robot_pose) # Aim at our corner (Update coordinates)
        #         self.location_state == 1

        #     if 3.5 < x_location < 13.0:
        #         # If we are in the neutral zone

        #         if y_location > 4.0: 
        #             self.aim_to_position(self.top_blue_corner, current_robot_pose)
        #         else:
        #             self.aim_to_position(self.bottom_blue_corner, current_robot_pose)
                
        #         self.location_state == 0

        #     if x_location > 13.0: 
        #         # If we are on the opposite side

        #         if y_location > 4.0: 
        #             self.aim_to_position(self.top_blue_corner, current_robot_pose)
        #         else:
        #             self.aim_to_position(self.bottom_blue_corner, current_robot_pose)
                
        #         self.location_state == -1
        # else:
        #     if x_location > 13.0: 
        #         # If we are on the team side

        #         self.aim_to_position(self.red_hub, current_robot_pose) # Aim at our corner (Update coordinates)
        #         self.location_state == 1

        #     if 3.5 < x_location < 13.0:
        #         # If we are in the neutral zone

        #         if y_location > 4.0: 
        #             self.aim_to_position(self.top_red_corner, current_robot_pose)
        #         else:
        #             self.aim_to_position(self.bottom_red_corner, current_robot_pose)
                
        #         self.location_state == 0
                
        #     if x_location < 3.5: 
        #         # If we are on the opposite side
                
        #         if y_location > 4.0: 
        #             self.aim_to_position(self.top_red_corner, current_robot_pose)
        #         else:
        #             self.aim_to_position(self.bottom_red_corner, current_robot_pose)
                
        #         self.location_state == -1


           

        
