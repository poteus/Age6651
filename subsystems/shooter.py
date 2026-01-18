from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
import wpilib
from wpilib import SmartDashboard, reportError
from wpilib.sysid import SysIdRoutineLog
from phoenix6 import hardware, controls, configs, SignalLogger, signals
import phoenix6
from wpimath.filter import Debouncer

class Shooter(Subsystem):
    """
    Shooter subsystem for controlling the robot's shooter mechanism.
    """

    def __init__(self):
        """
        Initialize the shooter subsystem.
        """
        super().__init__()

        self.shooter_motor = hardware.TalonFX(50)      # Talon FX motor controller with CAN ID 50
        self.shooter_motor.setInverted(False)          # Set motor inversion as needed

        # Control requests
        # self.voltage_request = controls.VoltageOut(0)                 # For open-loop voltage control (Not needed)      
        self.velocity_request = controls.VelocityTorqueCurrentFOC(0)    # For closed-loop velocity control (CURRENT FOC)
        self.torque_request = controls.TorqueCurrentFOC(0)              # For SysId routines (CURRENT FOC)
        self.break_request = controls.VoltageOut(0)                     # For stopping the motor (Coast)    

        # Dashboard Setup
        SmartDashboard.putNumber("Shooter/TargetVelocity", 0.0)         # Target velocity in units/sec
                                                                        # So we can set it up from Dashboard

        # Configuration for motor
        cfg = configs.TalonFXConfiguration()                            # Create a new configuration object
        cfg.current_limits.stator_current_limit_enable = True           # Enable stator current limit
        cfg.current_limits.stator_current_limit = 60.0                  # Set stator current limit to 60A   
        cfg.motor_output.neutral_mode = signals.NeutralModeValue.COAST  # Set neutral mode to Coast 
        # NOT BRAKE or it will break!

        # 0.5 seconds to reach full torque (slower acceleration for shooter wheel)
        cfg.closed_loop_ramps.torque_closed_loop_ramp_period = 0.5

        # PID Gains (Tune these for the Kraken X60)
        # These are starting values; you may need to adjust kP
        slot0 = cfg.slot0
        slot0.k_s = 2.0  # Amps to overcome friction
        slot0.k_v = 1.5  # Amps per unit of velocity
        slot0.k_p = 3.0  # Amps per unit of error
        slot0.k_i = 0.0
        slot0.k_d = 0.0

        # Apply configuration
        self.shooter_motor.configurator.apply(cfg)

        # SysId Routine Setup
        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate=1.5,       # Amps increase per second (Quasistatic)
                stepVoltage=12.0,   # Constant Amps for Dynamic test (Note: SysId calls it 'stepVoltage' but it sends units)
                timeout=10.0,       # Safety timeout
                # Link SysId state to Phoenix 6 SignalLogger
                recordState=lambda state: SignalLogger.write_string(
                    "state", SysIdRoutineLog.state_enum_to_string(state)
                )
            ),
            SysIdRoutine.Mechanism(
                # How to apply the amps
                lambda amps: self.shooter_motor.set_control(self.torque_request.with_output(amps)),
                # How to log (SignalLogger handles the heavy lifting, so we return None here)
                lambda log: None, 
                self
            )
        )

        # Jam detection: Must be at current limit for 2.0 continuous seconds
        self.jam_debouncer = Debouncer(2.0, Debouncer.DebounceType.kRising)
        self.is_jammed = False
    
    def periodic(self):
        """Standard Subsystem method that runs every 20ms"""
        # Log actual velocity to help with tuning
        actual_vel = self.shooter_motor.get_velocity().value
        stator_amps = self.shooter_motor.get_stator_current().value
        
        SmartDashboard.putNumber("Shooter/ActualVelocity", actual_vel) # Log actual velocity
        SmartDashboard.putNumber("Shooter/StatorCurrent", stator_amps) # Log stator current

        # Jam Detection Logic
        # If amps > 58 for 2 seconds, jam_detected becomes True
        jam_detected = self.jam_debouncer.calculate(stator_amps > 58.0)
        
        if jam_detected:
            self.is_jammed = True # Set jam state so we can read it later
            wpilib.reportError("SHOOTER JAM DETECTED - SHUTTING DOWN", False)
            
        SmartDashboard.putBoolean("Shooter/IsJammed", self.is_jammed)

    def run_shooter_pid(self):
        """Reads target from Dashboard and applies PID control"""
        if self.is_jammed: # If jammed, do not run shooter
            self.stop_shooter()
            return

        target = SmartDashboard.getNumber("Shooter/TargetVelocity", 0.0)            # Read target velocity
        self.shooter_motor.set_control(self.velocity_request.with_velocity(target)) # Apply velocity control

    def stop_shooter(self):
        """Stops the motor and lets it coast"""
        self.shooter_motor.set_control(self.break_request.with_output(0))

    def reset_jam(self):
        """Call this to clear the jam state"""
        self.is_jammed = False # Clear jam state

    # SysId Commands
    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.dynamic(direction)