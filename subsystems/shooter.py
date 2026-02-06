import commands2
import commands2.sysid
from phoenix6 import hardware, configs, controls, signals
from wpimath.units import seconds
from wpilib import SmartDashboard

class Shooter(commands2.Subsystem):
    # 5:1 (Gearbox) * (120/10) (Gears) = 60:1
    HOOD_GEAR_RATIO = 60.0
    
    def __init__(self):
        super().__init__()

        # Flywheel Setup (Kraken X60 - CAN ID 16) ---
        self.flywheel = hardware.TalonFX(16)
        fw_cfg = configs.TalonFXConfiguration()
        
        # Torque Current specific tuning
        # kP for Torque Current is often much smaller than Voltage kP
        fw_cfg.slot0.k_p = 5.0  # Amps per rps of error
        fw_cfg.slot0.k_i = 0.0
        fw_cfg.slot0.k_d = 0.0
        
        # Current Limits are VITAL for Torque Control
        fw_cfg.current_limits.stator_current_limit = 80.0 # Limit to 80 Amps
        fw_cfg.current_limits.stator_current_limit_enable = True

        self.flywheel.configurator.apply(fw_cfg)

        # Hood Setup (Kraken X44 - CAN ID 17) ---
        self.hood = hardware.TalonFX(17)
        hood_cfg = configs.TalonFXConfiguration()
        
        hood_cfg.feedback.sensor_to_mechanism_ratio = self.HOOD_GEAR_RATIO
        
        # Soft Limits
        hood_cfg.software_limit_switch.forward_soft_limit_threshold = 0.25 
        hood_cfg.software_limit_switch.forward_soft_limit_enable = True
        hood_cfg.software_limit_switch.reverse_soft_limit_threshold = 0.0
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
        self.fw_torque_request = controls.VelocityTorqueCurrentFOC(0)
        self.hood_torque_request = controls.MotionMagicTorqueCurrentFOC(0)
        
        # SysId still uses Voltage for standard characterization
        self.voltage_request = controls.VoltageOut(0)

        # SysId Routines ---
        self.flywheel_sys_id = commands2.sysid.SysIdRoutine(
            commands2.sysid.SysIdRoutine.Config(rampRate=1.0, stepVoltage=7.0, timeout=seconds(10)),
            commands2.sysid.SysIdRoutine.Mechanism(
                lambda volts: self.flywheel.set_control(self.voltage_request.with_output(volts)),
                None, self
            )
        )

        self.hood_sys_id = commands2.sysid.SysIdRoutine(
            commands2.sysid.SysIdRoutine.Config(rampRate=1.0, stepVoltage=4.0, timeout=seconds(5)),
            commands2.sysid.SysIdRoutine.Mechanism(
                lambda volts: self.hood.set_control(self.voltage_request.with_output(volts)),
                None, self
            )
        )

    # --- Methods ---
    def set_flywheel_rps(self, rps: float):
        """Sets flywheel speed using Torque-Current FOC."""
        self.flywheel.set_control(self.fw_torque_request.with_velocity(rps))

    def set_hood_position(self, rotations: float):
        """Sets hood position using Motion Magic Torque-Current FOC."""
        self.hood.set_control(self.hood_torque_request.with_position(rotations))

    def stop(self):
        self.flywheel.stopMotor()
        self.hood.stopMotor()

    def periodic(self):
        # Tracking torque (current) output is useful for debugging
        SmartDashboard.putNumber("Shooter/Flywheel Torque (Amps)", self.flywheel.get_torque_current().value)
        SmartDashboard.putNumber("Shooter/Flywheel RPS", self.flywheel.get_velocity().value)
        SmartDashboard.putNumber("Shooter/Hood Position", self.hood.get_position().value)

    # --- SysId Factories ---
    def sysIdFlywheelQuasistatic(self, direction):
        return self.flywheel_sys_id.quasistatic(direction)

    def sysIdFlywheelDynamic(self, direction):
        return self.flywheel_sys_id.dynamic(direction)

    def sysIdHoodQuasistatic(self, direction):
        return self.hood_sys_id.quasistatic(direction)

    def sysIdHoodDynamic(self, direction):
        return self.hood_sys_id.dynamic(direction)