import commands2
import commands2.sysid
import math
from ntcore import NetworkTableInstance
from phoenix6 import hardware, configs, signals, controls
from wpilib import SmartDashboard, DriverStation
from wpimath.units import seconds
from wpimath.geometry import Pose2d, Rotation2d

class Turret(commands2.Subsystem):
    '''
    The turret subsystem controls the yaw of the shooter (subsystems.shooter)
    '''

    # Gear Ratio: 200 teeth / 20 teeth = 10
    # 10 rotations of the Kraken = 1 rotation of the Turret
    GEAR_RATIO = 10.0 

    def __init__(self):
        super().__init__()

        # Initialize Kraken X60 (Talon FX)
        # Assuming CAN ID 15 and on the "rio" or "CANDrive" bus
        self.motor = hardware.TalonFX(15)

        # Configure Motor Settings
        cfg = configs.TalonFXConfiguration()

        # Feedback Settings: Set the gear ratio so 1.0 output = 1 full turret rotation
        cfg.feedback.sensor_to_mechanism_ratio = self.GEAR_RATIO
        
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

        self.target_position = 0.0

    def set_position(self, rotations: float):
        """Sets turret position (0.0 to 1.0 represents 0 to 360 degrees)"""
        self.motor.set_control(self.position_request.with_position(rotations))

    def stop(self):
        self.motor.stopMotor()

    def reset_position(self):
        """Manually reset the encoder to 0. Call this when turret is at 'home'"""
        self.motor.set_position(0)

    def angle_to_alliance_hub(self):
        """Angles turret to hub position (relative to team color)"""

        if not DriverStation.getAlliance(): return print("No alliance color found, method not passed")
        alliance_color = DriverStation.getAlliance()

        # Get robot position as double-array through NetworkTables
        network = NetworkTableInstance.getDefault()
        pose_table = network.getTable("Pose")
        robot_pose = pose_table.getDoubleArrayTopic("robotPose")
        
        # Hub position as Pose2d, based on alliance color
        hub_position = None
        if alliance_color == DriverStation.Alliance.kBlue:
            hub_position = Pose2d(4.625, 4.025, Rotation2d(0))
        else:
            hub_position = Pose2d(11.915, 4.025, Rotation2d(0))

        # Difference between robot and hub pose
        delta_x = (hub_position.X() - robot_pose[0])
        delta_y = (hub_position.Y() - robot_pose[1])

        # Find rotations based on deltas and make sure its under 1 or over 0
        motor_to_hub_rotations = (math.degrees(delta_x, delta_y))/360
        while motor_to_hub_rotations > 1:
            motor_to_hub_rotations -= 1
        while motor_to_hub_rotations < 0:
            motor_to_hub_rotations += 1

        # Unsure about this method, we use this motor method for 3 different turrert methods...
        self.motor.set_control(self.position_request.with_position(motor_to_hub_rotations))
        
    def update(self):
        """Updates the turret position with cable safety considered, rotates in reverse when past 1 rotation or less than 0"""
        # Get current position in rotations (0-1)
        current_pos = self.motor.get_position().value
        
        # Logic to flip direction
        if current_pos >= 0.99: # Almost at 360
            self.target_position = 0.0
        elif current_pos <= 0.01: # Back at 0
            self.target_position = 1.0
        
        # Send the command to the Talon FX
        self.motor.set_control(self.position_request.with_position(self.target_position))

    def periodic(self):
        # Log data to Dashboard
        pos_rotations = self.motor.get_position().value # get_position() returns a 'StatusSignal', so we call .value
        SmartDashboard.putNumber("Turret/Position Degrees", pos_rotations * 360.0)
        SmartDashboard.putNumber("Turret/Motor Velocity", self.motor.get_velocity().value)

        # Periodically updates kraken position
        self.update()