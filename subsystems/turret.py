import commands2
import commands2.sysid
from phoenix6 import hardware, configs, signals, controls
from wpilib import SmartDashboard
from wpimath.units import seconds
import math
from wpimath.geometry import Pose2d, Translation2d

class Turret(commands2.Subsystem):
    '''
    The turret subsystem controls the yaw of the shooter (subsystems.shooter)
    '''
    
    # Target: Center of the Speaker (Update coordinates based on Alliance)
    # Blue Speaker is roughly at (0.0, 5.5)
    SPEAKER_LOCATION = Translation2d(0.0, 5.5)

    def __init__(self):
        super().__init__()

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
        # Absolute range 0 to 1
        enc_cfg.magnet_sensor.absolute_sensor_discontinuity_point = 0.0 # Optional: Set discontinuity at 360 degrees if needed
        # TO DO: Add your magnet offset here after physically aligning
        # enc_cfg.magnet_sensor.magnet_offset = 0.123 
        self.abs_encoder.configurator.apply(enc_cfg)

        # Configure Motor Settings
        cfg = configs.TalonFXConfiguration()

        # Feedback Settings: Set the gear ratio so 1.0 output = 1 full turret rotation
        cfg.feedback.sensor_to_mechanism_ratio = GEAR_RATIO
        
        # Soft Limits: Prevent rotating past 360 degrees (0.0 to 1.0 rotations)
        # This protects cables!
        cfg.software_limit_switch.forward_soft_limit_threshold = 1.0 # 360 degrees
        cfg.software_limit_switch.forward_soft_limit_enable = True
        cfg.software_limit_switch.reverse_soft_limit_threshold = 0.0 # 0 degrees
        cfg.software_limit_switch.reverse_soft_limit_enable = True

        # PID Settings for Position Control
        cfg.slot0.k_p = 12.0 # Placeholder: start low and tune
        cfg.slot0.k_i = 0.0
        cfg.slot0.k_d = 0.1

        # Apply the configuration
        self.motor.configurator.apply(cfg)

        # Control Requests
        self.position_request = controls.MotionMagicVoltage(0)
        self.voltage_request = controls.VoltageOut(0)

    def set_position(self, rotations: float):
        """Sets turret position (0.0 to 1.0 represents 0 to 360 degrees)"""
        self.motor.set_control(self.position_request.with_position(rotations))

    def stop(self):
        self.motor.stopMotor()

    def reset_position(self):
        """Manually reset the encoder to 0. Call this when turret is at 'home'"""
        self.motor.set_position(0)

    def get_turret_rotation(self) -> float:
        """Returns current turret position in rotations (0 to 1.0)"""
        return self.motor.get_position().value
    
    def aim_at_speaker(self, robot_pose: Pose2d):
        """
        Calculates the angle to the speaker and moves the turret.
        """
        # Get vector from robot to speaker
        target_vector = self.SPEAKER_LOCATION - robot_pose.translation()
        
        # Calculate field-relative angle (radians)
        # atan2 returns the angle from the X-axis (0 to pi or -pi)
        target_field_angle = math.atan2(target_vector.y, target_vector.x)
        
        # Make it robot-relative by subtracting robot's current rotation
        robot_heading = robot_pose.rotation().radians()
        relative_angle = target_field_angle - robot_heading
        
        # Normalize the angle to take the shortest path (e.g., -10 deg instead of 350 deg)
        while relative_angle > math.pi: relative_angle -= 2 * math.pi
        while relative_angle < -math.pi: relative_angle += 2 * math.pi

        # Convert radians to rotations (1 rotation = 2*pi radians)
        target_rotations = relative_angle / (2 * math.pi)
        
        self.set_position(target_rotations)