import commands2
import commands2.sysid
from wpilib.sysid import SysIdRoutineLog
from phoenix6 import hardware, configs, controls, signals, SignalLogger
from wpimath.units import seconds
from wpilib import SmartDashboard

from ntcore import NetworkTableInstance

class Shooter(commands2.Subsystem):
    '''
    The shooter subsystem controls the output of fuel
    '''

    # 5:1 (Gearbox) * (120/10) (Gears) = 60:1
    HOOD_GEAR_RATIO = 60.0
    last_rps = -1.0
    last_hood_rot = -1.0
    
    def __init__(self):
        super().__init__()

        # Flywheel Setup (Kraken X60 - CAN ID 16) ---
        self.flywheel = hardware.TalonFX(16)
        fw_cfg = configs.TalonFXConfiguration()

        fw_cfg.motor_output.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE # or COUNTER_CLOCKWISE_POSITIVE
        fw_cfg.motor_output.neutral_mode = signals.NeutralModeValue.COAST

        fw_cfg.slot0.k_s = 9.0554  # Amps
        fw_cfg.slot0.k_v = 0.081473  # Amps per RPS
        fw_cfg.slot0.k_a = 0.55985  # Amps per RPS^2
        fw_cfg.slot0.k_p = 10 # 0.14981 # 1.0869   # Amps per Error(RPS)
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

        # --- Data Points for Tuning ---
        # Format: (Distance in Meters, Flywheel RPS, Hood Rotations)
        self.tuning_table = [
            (1.0, 50.0, 0.00), # Close
            (2, 55.0, 0.023)
        ]
        
    # --- Methods ---
    def set_flywheel_rps(self, rps: float):
        """Sets flywheel speed using Torque-Current FOC."""
        self.flywheel.set_control(self.fw_torque_request_vel.with_velocity(rps))

    def set_hood_position(self, rotations: float):
        """Sets hood position using Motion Magic Torque-Current FOC."""
        self.hood.set_control(self.hood_torque_request.with_position(rotations))

    def stop(self):
        self.flywheel.stopMotor()

    def reset_hood_position(self):
        """Manually reset the hood encoder to 0. Call this when hood is at 'home'."""
        self.hood.set_position(0)

    def interpolate(self, x, x1, y1, x2, y2):
        """Standard linear interpolation formula: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)"""
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1)
    
    def get_values_for_distance(self, distance_meters: float):
        """Finds the two closest points in the table and calculates the RPS and Hood position."""
        # Sort table by distance just in case
        table = sorted(self.tuning_table)
        
        # If too close, return the first point
        if distance_meters <= table[0][0]:
            return table[0][1], table[0][2]
        
        # If too far, return the last point
        if distance_meters >= table[-1][0]:
            return table[-1][1], table[-1][2]

        # Find the two points to interpolate between
        for i in range(len(table) - 1):
            d1, rps1, hood1 = table[i]
            d2, rps2, hood2 = table[i+1]
            
            if d1 <= distance_meters <= d2:
                interp_rps = self.interpolate(distance_meters, d1, rps1, d2, rps2)
                interp_hood = self.interpolate(distance_meters, d1, hood1, d2, hood2)
                return interp_rps, interp_hood
        
        return 60.0, 0.0  # Safe default
    
    def aim_at_distance(self, distance_meters: float):
        ''' Sets speed and hood position based on distance using interpolation from the tuning table. '''
        rps, hood_pos = self.get_values_for_distance(distance_meters)
        self.set_flywheel_rps(rps)
        self.set_hood_position(hood_pos)

    def shoot_control_dash(self):
        ''' Read the speed and the angle to shoot from the dashboard and apply them.
        Sets the setpoint of both the flywheel and the hood based on the values from the dashboard.
        Once it has been set, it waits until any of the two values change before updating again. '''
        current_rps = SmartDashboard.getNumber("Shooter Speed", 0.0)
        current_hood_rot = SmartDashboard.getNumber("Hood Position", 0.0)
        if current_rps != self.last_rps:
            self.set_flywheel_rps(current_rps)
            self.last_rps = current_rps
        if current_hood_rot != self.last_hood_rot:
            self.set_hood_position(current_hood_rot)
            self.last_hood_rot = current_hood_rot
    
    def stop_shooter_indexer(self):
        ''' Stops both the shooter and indexer motors. '''
        self.stop()

    def actual_rps(self):
        ''' Returns the actual RPS of the flywheel based on the encoder velocity. '''
        return self.flywheel.get_velocity().refresh().value
    
    def actual_hood_position(self):
        ''' Returns the actual position of the hood in rotations based on the encoder position. '''
        return self.hood.get_position().refresh().value

    def reach_rps(self, tolerance: float = 3.0):
        ''' Returns True if the flywheel is within the specified tolerance of the target RPS. '''
        return abs(self.actual_rps() - self.last_rps) < tolerance

    def reach_hood_position(self, tolerance: float = 0.01):
        ''' Returns True if the hood is within the specified tolerance of the target position. '''
        return abs(self.actual_hood_position() - self.last_hood_rot) < tolerance