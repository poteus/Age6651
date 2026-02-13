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

        # Initialize Motors (Assuming CAN IDs 50 and 51)
        self.front = SparkMax(51, SparkLowLevel.MotorType.kBrushless)
        self.back = SparkMax(50, SparkLowLevel.MotorType.kBrushless)

        # Create Configuration
        # We set the conversion factor to 1/60 to turn RPM into RPS
        # Velocity = (RPM / 60) = Revolutions per Second
        config50 = SparkMaxConfig()
        
        config50.encoder.velocityConversionFactor(1.0 / 60.0) 
        config50.encoder.positionConversionFactor(1.0) # 1 rotation = 1 unit
        config50.inverted(True)
      
        # Set PID Gains (Placeholder values - update after tuning)
        # kS = 0.14395
        # kV = 0.12408
        # kA = 0.012706
        #kP = 0.050463
        config50.closedLoop.P(0.0001).I(0).D(0).velocityFF(0.12408) # kV
        
        # Velocity = (RPM / 60) = Revolutions per Second
        config51 = SparkMaxConfig()
        
        config51.encoder.velocityConversionFactor(1.0 / 60.0) 
        config51.encoder.positionConversionFactor(1.0) # 1 rotation = 1 unit
        config51.inverted(False)

        # Set PID Gains (Placeholder values - update after tuning)
        # kS = 0.47088
        # kV = 0.13267
        # kA = 0.0068033
        #kP = 0.00026339
        config51.closedLoop.P(0.0001).I(0).D(0).velocityFF(0.13267)
        
        # Apply configuration to both motors
        self.front.configure(
            config50,
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters)
        self.back.configure(
            config51, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters)

        self.front_loop = self.front.getClosedLoopController()
        self.back_loop = self.back.getClosedLoopController()
        self.front_encoder = self.front.getEncoder()
        self.back_encoder = self.back.getEncoder()

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