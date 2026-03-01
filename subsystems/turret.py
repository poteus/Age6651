import commands2
import commands2.sysid
from phoenix6 import SignalLogger, hardware, configs, signals, controls
from wpilib.sysid import SysIdRoutineLog
from wpilib import SmartDashboard
from wpimath.units import seconds

class Turret(commands2.Subsystem):
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
        # cfg.software_limit_switch.forward_soft_limit_threshold = 1.0 # 360 degrees
        # cfg.software_limit_switch.forward_soft_limit_enable = True
        # cfg.software_limit_switch.reverse_soft_limit_threshold = 0.0 # 0 degrees
        # cfg.software_limit_switch.reverse_soft_limit_enable = True

        # PID Settings for Position Control
        cfg.slot0.k_p = 12.0 # Placeholder: start low and tune
        cfg.slot0.k_i = 0.0
        cfg.slot0.k_d = 0.1

        # Apply the configuration
        self.motor.configurator.apply(cfg)

        # Control Requests
        self.position_request = controls.MotionMagicVoltage(0)
        self.voltage_request = controls.VoltageOut(0)

        # SysId Characterization Routine
        self.sys_id_routine = commands2.sysid.SysIdRoutine(
            commands2.sysid.SysIdRoutine.Config(
                rampRate=0.20,
                stepVoltage=1.0,
                timeout=seconds(10),
                recordState=lambda state: SignalLogger.write_string("SysIdTurret_State", SysIdRoutineLog.stateEnumToString(state))
            ),
            commands2.sysid.SysIdRoutine.Mechanism(
                # Logic to apply voltage for SysId
                lambda volts: self.motor.set_control(self.voltage_request.with_output(volts)),
                    #if 0.05 < self.motor.get_position().value < 0.95 else self.motor.stopMotor() ), # Only apply voltage if we're within the soft limits, otherwise stop the motor
                self.log_turret,
                self
            )
        )

    def log_turret(self, log):
        """Helper to log motor data for SysId"""
        log.motor("turret") \
            .voltage(self.motor.get_motor_voltage().value) \
            .position(self.motor.get_position().value) \
            .velocity(self.motor.get_velocity().value)

    def set_position(self, rotations: float):
        """Sets turret position (0.0 to 1.0 represents 0 to 360 degrees)"""
        self.motor.set_control(self.position_request.with_position(rotations))

    def stop(self):
        self.motor.stopMotor()

    def reset_position(self):
        """Manually reset the encoder to 0. Call this when turret is at 'home'"""
        self.motor.set_position(0)

    def periodic(self):
        # Log data to Dashboard
        # get_position() returns a 'StatusSignal', so we call .value
        pos_rotations = self.motor.get_position().value
        SmartDashboard.putNumber("Turret/Position Degrees", pos_rotations * 360.0)
        SmartDashboard.putNumber("Turret/Motor Velocity", self.motor.get_velocity().value)

    # --- SysId Command Factories ---
    def sysIdQuasistatic(self, direction):
        return self.sys_id_routine.quasistatic(direction)

    def sysIdDynamic(self, direction):
        return self.sys_id_routine.dynamic(direction)