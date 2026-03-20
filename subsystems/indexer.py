import commands2
import commands2.sysid
from rev import SparkMax, SparkLowLevel, SparkMaxConfig, ResetMode, PersistMode

from wpilib import SmartDashboard
from wpimath.units import seconds

from ntcore import NetworkTableInstance

from subsystems.shooter import Shooter

class Indexer(commands2.Subsystem):
    '''
    The indexer subsystem funnels the fuel from wherever we store fuel all the way to the shooter
    '''
    last_indexer_rps = -1.0
    state_hopper_forward = True

    def __init__(self, _shooter:Shooter):
        super().__init__()

        self.shooter = _shooter

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

        # Initialize Motors (Assuming CAN IDs 50, 51 for indexer, 52 for ChakaChaka Hopper)
        self.front = SparkMax(51, SparkLowLevel.MotorType.kBrushless)
        self.back = SparkMax(50, SparkLowLevel.MotorType.kBrushless)
        self.hopper = SparkMax(52, SparkLowLevel.MotorType.kBrushless)


        # Create Configuration
        # We set the conversion factor to 1/60 to turn RPM into RPS
        # Velocity = (RPM / 60) = Revolutions per Second
        # Set the Idle Mode to Coast
   
        config50 = SparkMaxConfig()
        
        config50.encoder.velocityConversionFactor(1.0 / 60.0) 
        config50.encoder.positionConversionFactor(1.0) # 1 rotation = 1 unit
        config50.inverted(False)
        config50.IdleMode(SparkMaxConfig.IdleMode.kCoast)
      
        # Set PID Gains (Placeholder values - update after tuning)
        # kS = 0.14395
        # kV = 0.12408
        # kA = 0.012706
        #kP = 0.050463
        config50.closedLoop.P(0.0001).I(0).D(0.00005).velocityFF(0.12408) # kV
        config50.closedLoop.feedForward.kS(0.14395)
        
        # Velocity = (RPM / 60) = Revolutions per Second
        config51 = SparkMaxConfig()
        
        config51.encoder.velocityConversionFactor(1.0)# / 60.0) 
        config51.encoder.positionConversionFactor(1.0) # 1 rotation = 1 unit
        config51.inverted(True)
        config51.IdleMode(SparkMaxConfig.IdleMode.kCoast)

        # Set PID Gains (Placeholder values - update after tuning)
        # kS = 0.47088
        # kV = 0.13267
        # kA = 0.0068033
        #kP = 0.00026339
        config51.closedLoop.P(0.0001).I(0).D(0.00005).velocityFF(0.13267)
        config51.closedLoop.feedForward.kS(0.47088)
        
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

        # Hopper Configuration -----------------------------------
        self.hopper_config = SparkMaxConfig()
        self.hopper_config.inverted(False)

        # Apply configuration to both motors
        self.hopper.configure(
            self.hopper_config,
             ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters)
        
        self.last_hopper_encoder_pos = self.hopper.getEncoder().getPosition()

    # Methods ------------------------------------

    def set_duty_cycle_hopper(self, DC: float):
        ''' Sets Duty Cycle to Hopper motor '''
        if DC > .4:
            DC = .4
        elif DC < -.4:
            DC =-.4

        self.hopper.set(DC)

    def stop_hopper(self):
        ''' Stops hopper '''
        self.hopper.stopMotor()

    def set_velocity(self, rps: float):
        """Sets the indexer speed in Revolutions per Second."""
        self.front_loop.setReference(rps, SparkMax.ControlType.kVelocity)
        back_target = rps * (7.0 / 5.0)
        self.back_loop.setReference(back_target, SparkMax.ControlType.kVelocity)

    def run(self):
        ''' Starts the indexer and the hopper (ChakaChaka Bum Bum)'''
        self.set_velocity(40)
        self.set_duty_cycle_hopper(.6)
        #self.chakachaka()

    def stop_all(self):
        ''' Stop indexer and hopper '''
        self.stop()
        self.stop_hopper()

    def stop(self):
        self.front.stopMotor()
        self.back.stopMotor()
        self.last_indexer_rps = 0

    def indexer_control_rps(self):
        ''' Activates the indexer control using the RPS value from the dashboard,
            applying changes only when the target or the shooter readiness changes. '''

        # Check if the shooter is actually ready
        is_ready = self.shooter.reach_rps() #and self.shooter.reach_hood_position()

        if is_ready:
            # Only send the CAN frame if the target has actually changed
            # OR if we were previously stopped and now we are starting
            if self.last_indexer_rps == 0:
                self.run()
                self.last_indexer_rps = 40
        else:
            # If the shooter isn't ready, we MUST stop.
            # We check if last_indexer_rps != 0 so we don't spam 'stop' repeatedly.
            if self.last_indexer_rps != 0:
                self.stop_all()
                self.last_indexer_rps = 0

    def chakachaka(self):
        ''' Goes forward 2 laps then backward 2 lap to shake loose any stuck fuel. '''
        position = self.hopper.getEncoder().getPosition()
        print(f"position - {position}")
        if self.state_hopper_forward:
            if position < 2:
                self.hopper.set(0.1)
            else:
                self.state_hopper_forward = False
        else:
            if position > 0:
                self.hopper.set(-0.1)
            else:
                self.state_hopper_forward = True

    # def periodic(self):
    #     # Log data for debugging
    #     SmartDashboard.putNumber("Indexer/Velocity RPS", self.front_encoder.getVelocity())
    #     SmartDashboard.putNumber("Indexer/Applied Output", self.front.getAppliedOutput())
    #      # Read the value from the Dashboard
    #     # target_rps = self.velocity_sub.get()
            
    #     # Apply the speed (only if you want it constantly running for testing)
    #     # In a real match, you'd use a command, but for bench testing this works:
    #     # self.set_velocity(target_rps)

    #     # Publish the actual velocity for the graph
    #     self.actual_pub.set(self.front_encoder.getVelocity())