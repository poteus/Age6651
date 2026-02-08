import commands2
import commands2.sysid
from rev import SparkMax, SparkLowLevel, SparkMaxConfig, ResetMode, PersistMode

from wpilib import SmartDashboard
from wpimath.units import seconds

from ntcore import NetworkTableInstance

class Indexer(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        # Init network table for speed of indexer
        nt = NetworkTableInstance.getDefault()
        table = nt.getTable("SmartDashboard")

        # Create a Topic and a Subscriber
        # This creates the "box" on the dashboard. Defaulting to 0.0 RPS.
        self.velocity_topic = table.getDoubleTopic("Indexer/IndexerTargetRPS")
        self.velocity_pub = self.velocity_topic.publish()
        self.velocity_pub.set(0.0)
        self.velocity_sub = self.velocity_topic.subscribe(0.0)
        
        # We also want to publish the ACTUAL speed so we can compare them
        self.actual_pub = table.getDoubleTopic("Indexer/IndexerActualRPS").publish()

        # Initialize Motors (Assuming CAN IDs 10 and 11)
        self.front = SparkMax(11, SparkLowLevel.MotorType.kBrushless)
        self.back = SparkMax(10, SparkLowLevel.MotorType.kBrushless)

        # Create Configuration
        # We set the conversion factor to 1/60 to turn RPM into RPS
        # Velocity = (RPM / 60) = Revolutions per Second
        config10 = SparkMaxConfig()
        
        config10.encoder.velocityConversionFactor(1.0 / 60.0) 
        config10.encoder.positionConversionFactor(1.0) # 1 rotation = 1 unit
        config10.inverted(False)
      
        # Set PID Gains (Placeholder values - update after tuning)
        # kS = 0.14395
        # kV = 0.12408
        # kA = 0.012706
        #kP = 0.050463
        config10.closedLoop.P(0.0001).I(0).D(0).velocityFF(0.12408) # kV
        
        # Velocity = (RPM / 60) = Revolutions per Second
        config11 = SparkMaxConfig()
        
        config11.encoder.velocityConversionFactor(1.0 / 60.0) 
        config11.encoder.positionConversionFactor(1.0) # 1 rotation = 1 unit
        config11.inverted(True)

        # Set PID Gains (Placeholder values - update after tuning)
        # kS = 0.47088
        # kV = 0.13267
        # kA = 0.0068033
        #kP = 0.00026339
        config11.closedLoop.P(0.0001).I(0).D(0).velocityFF(0.13267)
        
        # Apply configuration to both motors
        self.front.configure(
            config10,
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters)
        self.back.configure(
            config11, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters)

        self.front_loop = self.front.getClosedLoopController()
        self.back_loop = self.back.getClosedLoopController()
        self.front_encoder = self.front.getEncoder()
        self.back_encoder = self.front.getEncoder()

        # 3. SysId Characterization Routine
        self.sys_id_routine = commands2.sysid.SysIdRoutine(
            commands2.sysid.SysIdRoutine.Config(
                rampRate=1.0, # 1V per second
                stepVoltage=7.0,      # 7V for dynamic test
                timeout=seconds(10)
            ),
            commands2.sysid.SysIdRoutine.Mechanism(
                # Drive both motors at the same voltage
                lambda volts: (
                    self.front.setVoltage(volts),
                    self.back.setVoltage(volts)
                ),
                # Log BOTH motors as separate "log.motor" entries
                lambda log: (
                    log.motor("front-rollers")
                        .voltage(self.front.getAppliedOutput() * self.front.getBusVoltage())
                        .position(self.front.getEncoder().getPosition())
                        .velocity(self.front.getEncoder().getVelocity()),
                    log.motor("back-rollers")
                        .voltage(self.back.getAppliedOutput() * self.back.getBusVoltage())
                        .position(self.back.getEncoder().getPosition())
                        .velocity(self.back.getEncoder().getVelocity())
                ),
                self
            )
        )

    def set_velocity(self, rps: float):
        """Sets the indexer speed in Revolutions per Second."""
        self.front_loop.setReference(rps, SparkMax.ControlType.kVelocity)
        back_target = rps * (7.0 / 5.0)
        self.back_loop.setReference(back_target, SparkMax.ControlType.kVelocity)

    def stop(self):
        self.front.stopMotor()
        self.back.stopMotor()

    def periodic(self):
        # Log data for debugging
        SmartDashboard.putNumber("Indexer/Velocity RPS", self.front_encoder.getVelocity())
        SmartDashboard.putNumber("Indexer/Applied Output", self.front.getAppliedOutput())
         # Read the value from the Dashboard
        target_rps = self.velocity_sub.get()
            
        # Apply the speed (only if you want it constantly running for testing)
        # In a real match, you'd use a command, but for bench testing this works:
        # self.set_velocity(target_rps)

        # Publish the actual velocity for the graph
        self.actual_pub.set(self.front_encoder.getVelocity())

    # --- SysId Command Factories ---
    def sysIdQuasistatic(self, direction):
        return self.sys_id_routine.quasistatic(direction)

    def sysIdDynamic(self, direction):
        return self.sys_id_routine.dynamic(direction)