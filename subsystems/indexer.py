import commands2
import commands2.sysid
from rev import SparkMax, SparkLowLevel, SparkMaxConfig, ResetMode, PersistMode

from wpilib import SmartDashboard
from wpimath.units import seconds

class Indexer(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        # 1. Initialize Motors (Assuming CAN IDs 10 and 11)
        self.leader = SparkMax(10, SparkLowLevel.MotorType.kBrushless)
        self.follower = SparkMax(11, SparkLowLevel.MotorType.kBrushless)

        # 2. Create Configuration
        # We set the conversion factor to 1/60 to turn RPM into RPS
        # Velocity = (RPM / 60) = Revolutions per Second
        config = SparkMaxConfig()
        
        config.encoder.velocityConversionFactor(1.0 / 60.0) 
        config.encoder.positionConversionFactor(1.0) # 1 rotation = 1 unit

        # Set PID Gains (Placeholder values - update after tuning)
        config.closedLoop.P(0.1).I(0).D(0).velocityFF(0.12)
        
        # Follower logic
        config.follow(10) # Follow the leader on CAN ID 10

        # Apply configuration to both motors
        self.leader.configure(
            config,
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters)
        self.follower.configure(
            config, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters)

        self.closed_loop = self.leader.getClosedLoopController()
        self.encoder = self.leader.getEncoder()

        # 3. SysId Characterization Routine
        self.sys_id_routine = commands2.sysid.SysIdRoutine(
            commands2.sysid.SysIdRoutine.Config(
                rampRate=1.0, # 1V per second
                stepVoltage=7.0,      # 7V for dynamic test
                timeout=seconds(10)
            ),
            commands2.sysid.SysIdRoutine.Mechanism(
                lambda volts: self.leader.setVoltage(volts),
                None, # Log consumer (handled automatically in 2026)
                self
            )
        )

    def set_velocity(self, rps: float):
        """Sets the indexer speed in Revolutions per Second."""
        self.closed_loop.setReference(rps, SparkMax.ControlType.kVelocity)

    def stop(self):
        self.leader.stopMotor()

    def periodic(self):
        # Log data for debugging
        SmartDashboard.putNumber("Indexer/Velocity RPS", self.encoder.getVelocity())
        SmartDashboard.putNumber("Indexer/Applied Output", self.leader.getAppliedOutput())

    # --- SysId Command Factories ---
    def sysIdQuasistatic(self, direction):
        return self.sys_id_routine.quasistatic(direction)

    def sysIdDynamic(self, direction):
        return self.sys_id_routine.dynamic(direction)