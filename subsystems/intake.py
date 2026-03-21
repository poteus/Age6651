import commands2

from phoenix5 import ControlMode, TalonSRX, NeutralMode
from rev import SparkMax, SparkLowLevel, SparkMaxConfig, ResetMode, PersistMode
import rev
class Intake(commands2.Subsystem):
    '''
    The intake subsystem controls the intake of fuel
    '''

    is_shoulder_braking = True
    last_shoulder_position = False

    def __init__(self):
        super().__init__()

        # Shoulder and intake motors CHECK THESE
        self.intake = TalonSRX(18)
        self.shoulder = SparkMax(53, SparkLowLevel.MotorType.kBrushless)

        # Create Configurations for both intake and shoulder

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
        self.shoulder_config = SparkMaxConfig()
        self.shoulder_config.inverted(False)

        # Gearbox is 92.5:1. 
        # Position factor: 1 / 92.5 (Output rotations per motor rotation)
        # Velocity factor: 1 / 92.5 (Output RPM per motor RPM)
        self.shoulder_config.encoder.positionConversionFactor(1.0 / 92.5)
        self.shoulder_config.encoder.velocityConversionFactor(1.0 / 92.5)

        # --- Limit Switch Setup ---
        # 0 Degrees (Inside) is Reverse, 90 Degrees (Bumper) is Forward
        self.shoulder_config.limitSwitch.reverseLimitSwitchEnabled(True)
        self.shoulder_config.limitSwitch.reverseLimitSwitchType(rev.LimitSwitchConfig.Type.kNormallyClosed)
        
        # When reverse limit switch is reached, it stops the motors and set encoder position to 0.
        # self.shoulder_config.limitSwitch.reverseLimitSwitchTriggerBehavior( # type: ignore
        #     rev.LimitSwitchConfig.Behavior.kStopMovingMotorAndSetPosition
        # )
        # self.shoulder_config.limitSwitch.reverseLimitSwitchPosition(0.0)
        
        self.shoulder_config.limitSwitch.forwardLimitSwitchEnabled(True)
        self.shoulder_config.limitSwitch.forwardLimitSwitchType(rev.LimitSwitchConfig.Type.kNormallyClosed)

        # Soft Limits act as a "virtual" wall before the physical switch
        # If your 90 deg switch is at 0.25 rotations, set soft limit to 0.24
        # self.shoulder_config.softLimit.forwardSoftLimit(0.34)
        # self.shoulder_config.softLimit.forwardSoftLimitEnabled(True)
        
        # self.shoulder_config.softLimit.reverseSoftLimit(0.01)
        # self.shoulder_config.softLimit.reverseSoftLimitEnabled(True)

        # Set PID Gains (Placeholder values - update after tuning)
        # kS = 0.47088
        # kV = 0.13267
        # kA = 0.0068033
        # kP = 0.00026339
        self.shoulder_config.closedLoop.P(2
                                          ).I(0).D(0.1)    #.velocityFF(0.016)
        self.shoulder_config.closedLoop.feedForward.kS(0.2).kG(0.4).kV(0.16)
        self.shoulder_config.IdleMode(SparkMaxConfig.IdleMode.kCoast)


        # Apply configuration to NEO
        status = self.shoulder.configure(
            self.shoulder_config, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters)
        
        if status != rev.REVLibError.kOk:
            print(f"ERROR: Shoulder config failed with code: {status}")
        else:
            print("Shoulder configuration applied successfully!")

        # Config object for Braking mode
        # self.brake_config = SparkMaxConfig()
        # self.brake_config.apply(self.shoulder_config) # Copy all master settings
        # self.brake_config.IdleMode(SparkMaxConfig.IdleMode.kBrake)

        # #Config object for Coasting mode
        # self.coast_config = SparkMaxConfig()
        # self.coast_config.apply(self.shoulder_config) # Copy all master settings
        # self.coast_config.IdleMode(SparkMaxConfig.IdleMode.kCoast)

        # Apply configuration to TalonSRX
        # self.shoulder.configure(
        #     self.brake_config, 
        #     ResetMode.kResetSafeParameters, 
        #     PersistMode.kNoPersistParameters)
        
        self.is_braking = True

        # Shoulder Configuration -----------------------------------

        self.shoulder_loop = self.shoulder.getClosedLoopController()
        self.shoulder_encoder = self.shoulder.getEncoder()

        
    def set_shoulder_position(self, rotations: float):
        ''' Set the shoulder to a specific position in rotations. 0 rotations is the "home" position, and positive rotations are clockwise.
        '''
        requested_position = rotations
        if requested_position != self.last_shoulder_position:

            self.shoulder_loop.setReference(rotations, SparkMax.ControlType.kPosition)
            
            self.last_shoulder_position = requested_position
            
    def set_intake_dutyCycle(self, DC: float = .6):
        '''Set the DutyCycle for the Intake Motor. DC should be between -1.0 and 1.0, where 1.0 is full forward and -1.0 is full reverse.
        '''

        self.shoulder.stopMotor()
        if DC > 1.0:
            DC = 1.0
        elif DC < -1.0:
            DC = -1.0
        self.intake.set(ControlMode.PercentOutput, -DC)

    def stop(self):
        self.intake.set(ControlMode.PercentOutput, 0)
        self.shoulder.stopMotor()


    def periodic(self):
        ''' Sets the idle mode to Brake when the shoulder is inside (0 rotations) and Coast when it's deployed (0.25 rotations). 
        This allows the intake to be "floating" when deployed, but tight and secure when stowed.
        '''
        # 0.0 is Stowed (Inside), 0.25 is Deployed (Outside/90 deg)
        # current_pos = self.shoulder_encoder.getPosition()

        # # If we are past x degrees (approx 0.2 rotations), switch to Coast
        # # This allows the "floating" intake to be bumped by game pieces
        # if current_pos > 0.2 and self.is_shoulder_braking:
        #     self.set_idle_mode(False) # Coast
        #     self.is_shoulder_braking = False
        # elif current_pos <= 0.2 and not self.is_shoulder_braking:
        #     # When inside, use Brake to keep it tight and prevent rattling
        #     self.set_idle_mode(True) # Brake
        #     self.is_shoulder_braking = True
        pass
            

    def set_idle_mode(self, use_brake: bool):
        """Swaps the motor between Coast and Brake by re-applying config"""
        # if use_brake == self.is_braking:
        #     return # Don't re-apply if we are already in that mode

        # target_config = self.brake_config if use_brake else self.coast_config
        
        # # We use kNoPersistParameters so we don't wear out the flash memory
        # self.shoulder.configure(target_config, 
        #                         ResetMode.kNoResetSafeParameters, 
        #                         PersistMode.kNoPersistParameters)
        # self.is_braking = use_brake
        pass