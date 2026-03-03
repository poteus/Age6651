import commands2
import commands2.sysid
from phoenix5 import TalonSRX, TalonSRXConfiguration
from rev import SparkMax, SparkLowLevel, SparkMaxConfig, ResetMode, PersistMode

from wpilib import SmartDashboard
from wpimath.units import seconds

from ntcore import NetworkTableInstance

class Intake(commands2.Subsystem):
    '''
    The intake subsystem controls the intake of fuel
    '''

    def __init__(self):
        super().__init__()

        # Init network table for speed of indexer
        nt = NetworkTableInstance.getDefault()
        table = nt.getTable("SmartDashboard")
        # Create a Topic and a Subscriber
        # This creates the "box" on the dashboard. Defaulting to 0.0 RPS.
        self.shoulder_position_topic = table.getDoubleTopic("Shoulder Position")
        self.shoulder_position_pub = self.shoulder_position_topic.publish()
        self.shoulder_position_pub.set(0.0)
        self.shoulder_position_sub = self.shoulder_position_topic.subscribe(0.0)

        # We also want to publish the ACTUAL speed so we can compare them
        self.actual_pub = table.getDoubleTopic("Shoulder Position").publish()

        # Shoulder and intake motors CHECK THESE
        self.intake = TalonSRX(510)
        self.shoulder = SparkMax(500, SparkLowLevel.MotorType.kBrushless)

        # Create Configurations for both intake and sholder

        # Intake Configuration -----------------------------------
        # Factory Default to ensure no old settings interfere
        self.intake.configFactoryDefault()

        # Set Neutral Mode to Coast or Brake
        # For an intake/indexer, Brake is usually better to stop fuel instantly
        self.intake.setNeutralMode(NeutralMode.Brake)
        # Voltage Compensation
        # This ensures 50% power feels the same even if the battery drops from 12V to 10V
        self.intake.configVoltageCompSaturation(12.0)
        self.intake.enableVoltageCompensation(True)

        # Open Loop Ramp
        # This prevents the 775pro from "snapping" from 0 to 50% instantly,
        # which saves your 4:1 gearbox from high shock loads.
        self.intake.configOpenloopRamp(0.1) # 0.1 seconds to reach full speed
        # Intake Configuration -----------------------------------

        # Shoulder Configuration -----------------------------------
        # We set the conversion factor to 1/60 to turn RPM into RPS
        # Velocity = (RPM / 60) = Revolutions per Second
        shoulder_config = SparkMaxConfig()
        
        shoulder_config.encoder.velocityConversionFactor(1.0)# / 60.0)
        shoulder_config.encoder.positionConversionFactor(1.0) # 1 rotation = 1 unit
        shoulder_config.inverted(True)
        shoulder_config.IdleMode(SparkMaxConfig.IdleMode.kCoast)

        # Set PID Gains (Placeholder values - update after tuning)
        # kS = 0.47088
        # kV = 0.13267
        # kA = 0.0068033
        #kP = 0.00026339
        shoulder_config.closedLoop.P(0.0001).I(0).D(0.00005).velocityFF(0.13267)
        shoulder_config.closedLoop.feedForward.kS(0.47088)
        
        # Apply configuration to both motors
        self.shoulder.configure(
            shoulder_config, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters)
        # Shoulder Configuration -----------------------------------

        self.shoulder_loop = self.shoulder.getClosedLoopController()
        self.shoulder_encoder = self.shoulder.getEncoder()

        # written by dan and chris - not tested yet 🤪😎
        
    def set_shoulder_position(self, rotations: float):
        self.shoulder_loop.setReference(rotations, SparkMax.ControlType.kPosition)

    def set_intake_velocity(self, rps: float):
        self.intake.setVoltage(rps * 12.0 / 40.0) # Placeholder conversion from RPS to voltage

    def set_velocity(self, rps: float):
        """Sets the intake speed in Revolutions per Second."""
        self.intake_loop.setReference(rps, SparkMax.ControlType.kVelocity)
        shoulder_target = rps * (7.0 / 5.0)
        self.shoulder_loop.setReference(shoulder_target, SparkMax.ControlType.kVelocity)

    def stop(self):
        self.intake.stopMotor()
        self.shoulder.stopMotor()