import commands2
import commands2.sysid
from wpilib.sysid import SysIdRoutineLog
from phoenix6 import hardware, configs, controls, signals, SignalLogger
from wpimath.units import seconds
from wpilib import SmartDashboard

from ntcore import NetworkTableInstance

class Shooter(commands2.Subsystem):
    # 5:1 (Gearbox) * (120/10) (Gears) = 60:1
    HOOD_GEAR_RATIO = 60.0
    
    def __init__(self):
        super().__init__()

        # Init network table for speed of indexer
        nt = NetworkTableInstance.getDefault()
        table = nt.getTable("SmartDashboard")

        # Create a Topic and a Subscriber
        # This creates the "box" on the dashboard. Defaulting to 0.0 RPS.
        self.velocity_topic = table.getDoubleTopic("Shooter/ShooterTargetRPS")
        self.velocity_pub = self.velocity_topic.publish()
        self.velocity_pub.set(0.0)
        self.velocity_sub = self.velocity_topic.subscribe(0.0)

        self.position_topic = table.getDoubleTopic("Shooter/HoodEncoder")
        self.position_pub = self.position_topic.publish()
        
        # We also want to publish the ACTUAL speed so we can compare them
        self.actual_pub = table.getDoubleTopic("Shooter/ShooterActualRPS").publish()

        # Flywheel Setup (Kraken X60 - CAN ID 16) ---
        self.flywheel = hardware.TalonFX(16)
        fw_cfg = configs.TalonFXConfiguration()

        fw_cfg.motor_output.inverted = signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE # or COUNTER_CLOCKWISE_POSITIVE
        fw_cfg.motor_output.neutral_mode = signals.NeutralModeValue.COAST

        fw_cfg.slot0.k_s = 16.494  # Amps
        fw_cfg.slot0.k_v = 0.26707  # Amps per RPS
        fw_cfg.slot0.k_a = 0.68816  # Amps per RPS^2
        fw_cfg.slot0.k_p = 1.0869   # Amps per Error(RPS)
        fw_cfg.slot0.k_i = 0.0
        fw_cfg.slot0.k_d = 0.0
        
        # Current Limits are VITAL for Torque Control
        fw_cfg.current_limits.stator_current_limit = 80.0 # Limit to 80 Amps
        fw_cfg.current_limits.stator_current_limit_enable = True

        self.flywheel.configurator.apply(fw_cfg)

        # Hood Setup (Kraken X44 - CAN ID 17) ---
        self.hood = hardware.TalonFX(17)
        hood_cfg = configs.TalonFXConfiguration()

        hood_cfg.motor_output.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE # or COUNTER_CLOCKWISE_POSITIVE
        hood_cfg.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        
        hood_cfg.feedback.sensor_to_mechanism_ratio = self.HOOD_GEAR_RATIO
        
        # Hood PID values
        hood_cfg.slot0.k_s = 6.0413 # Amps
        hood_cfg.slot0.k_v = 11.553  # Amps per RPS
        hood_cfg.slot0.k_a = 1.772  # Amps per RPS^2
        hood_cfg.slot0.k_p = 1.2479   # Amps per Error(RPS)
        hood_cfg.slot0.k_i = 0.0
        hood_cfg.slot0.k_d = 0.0

        # Soft Limits
        hood_cfg.software_limit_switch.forward_soft_limit_threshold = 0.117 
        hood_cfg.software_limit_switch.forward_soft_limit_enable = True
        hood_cfg.software_limit_switch.reverse_soft_limit_threshold = 0.006
        hood_cfg.software_limit_switch.reverse_soft_limit_enable = True
        
        # Motion Magic Settings
        hood_cfg.motion_magic.motion_magic_cruise_velocity = 0.5 
        hood_cfg.motion_magic.motion_magic_acceleration = 1.5
        
        # Current limits for the smaller X44 motor
        hood_cfg.current_limits.stator_current_limit = 40.0 
        hood_cfg.current_limits.stator_current_limit_enable = True
        
        self.hood.configurator.apply(hood_cfg)

        # Torque Control Requests ---
        # FOC (Field Oriented Control) provides the most efficient torque
        self.fw_torque_request_vel = controls.VelocityTorqueCurrentFOC(0)
        self.fw_torque_request = controls.TorqueCurrentFOC(0)
        self.hood_torque_request = controls.MotionMagicTorqueCurrentFOC(0)
        self.torque_current_request = controls.TorqueCurrentFOC(0)
        
        # SysId still uses Voltage for standard characterization
        self.voltage_request = controls.VoltageOut(0)

    # --- Methods ---
    def set_flywheel_rps(self, rps: float):
        """Sets flywheel speed using Torque-Current FOC."""
        self.flywheel.set_control(self.fw_torque_request_vel.with_velocity(rps))

    # def set_hood_position(self, rotations: float):
    #     """Sets hood position using Motion Magic Torque-Current FOC."""
    #     self.hood.set_control(self.hood_torque_request.with_position(rotations))

    def stop(self):
        self.flywheel.stopMotor()
        # self.hood.stopMotor()

    def periodic(self):
        # Tracking torque (current) output is useful for debugging
        SmartDashboard.putNumber("Shooter/Flywheel Torque (Amps)", self.flywheel.get_torque_current().value)
        SmartDashboard.putNumber("Shooter/Flywheel RPS", self.flywheel.get_velocity().value)
        SmartDashboard.putNumber("Shooter/HoodEncoder", self.hood.get_position().value)