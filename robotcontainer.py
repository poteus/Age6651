
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.cmd
from commands2.sysid import SysIdRoutine
from commands2.button import CommandXboxController, Trigger

from telemetry import Telemetry

from phoenix6 import swerve, SignalLogger
from subsystems.vision import Vision
from subsystems.indexer import Indexer
from subsystems.shooter import Shooter
from subsystems.turret import Turret

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathConstraints

import wpilib
from wpilib import DriverStation
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.units import rotationsToRadians
from wpimath import units


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        # Detect which roborio is running to know if it is Murphy or Crimson
        serial = wpilib.RobotController.getSerialNumber()
        print(f"Robot Serial Number: {serial}")
        if serial == "03415952":
            print("This is Crimson.")
            # Crimson has two cameras
            #self.vision = Vision(["limelight-front", "limelight-back"])
            from generated.Crimson_tuner_constants import TunerConstants 
        else:
            print("This is Murphy.")
            # Murphy (or anything else) has one
            self.vision = Vision(["limelight-front"])
            from generated.Murphy_tuner_constants import TunerConstants 

        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.5
        )  # 3/4 of a rotation per second max angular velocity

        print("With MOTION MAGIC.")
        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(self._max_angular_rate * 0.1)
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY
            )  # Use open-loop control for drive motors
            .with_steer_request_type(swerve.requests.SwerveModule.SteerRequestType.MOTION_MAGIC_EXPO)
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        self._logger = Telemetry(self._max_speed)

        self._joystick = CommandXboxController(0)

        self.indexer = Indexer()
        self.shooter = Shooter()
        self.turret = Turret()
        self.drivetrain = TunerConstants.create_drivetrain()

        # Build the auto chooser and put it on the dashboard
        self.auto_chooser = AutoBuilder.buildAutoChooser()
        wpilib.SmartDashboard.putData("Auto Chooser", self.auto_chooser)

        self.target_pose = Pose2d(2.5, 4.0, Rotation2d.fromDegrees(0))

        self.constraints = PathConstraints(
            3.0, 3.0,
            units.rotationsToRadians(2), units.rotationsToRadians(1)
        )

        # Configure the button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        # --- DRIVE LOCKDOWN ---
        # Instead of joystick control, we force the drivetrain to stay in Brake mode.
        # This prevents any accidental movement from the joysticks.
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(lambda: swerve.requests.SwerveDriveBrake())
        )

        # --- Indexer Characterization (SysId) ---
        # Keep these active for your testing
        self._joystick.start().and_(self._joystick.y()).whileTrue(
            self.indexer.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        )
        self._joystick.start().and_(self._joystick.x()).whileTrue(
            self.indexer.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        )
        self._joystick.start().and_(self._joystick.b()).whileTrue(
            self.indexer.sysIdDynamic(SysIdRoutine.Direction.kForward)
        )
        self._joystick.start().and_(self._joystick.a()).whileTrue(
            self.indexer.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        )

        # --- Logging Control ---
        self._joystick.leftStick().onTrue(commands2.cmd.runOnce(SignalLogger.start))
        self._joystick.rightStick().onTrue(commands2.cmd.runOnce(SignalLogger.stop))

       # --- GLOBAL KILL SWITCH ---
        # Pressing the 'Back' (View) button will cancel ALL running commands.
        # This will immediately stop the Indexer and Drivetrain.
        self._joystick.back().onTrue(
            commands2.cmd.runOnce(
                lambda: commands2.CommandScheduler.getInstance().cancelAll()
            ).ignoringDisable(True)
        )

    # Updates the pose from Vision subsystem
    def update_vision_odometry(self):
        #This sends the data of the pigeon to the limelight
        #Gets the latest data of the pigeon
        imu = self.drivetrain.pigeon2
        yaw = imu.get_yaw().value
        pitch = imu.get_pitch().value
        roll = imu.get_roll().value
        yaw_rate = imu.get_angular_velocity_z_world().value
        #Pushes it to the limelight
        self.vision.patch_limelight_orientation([yaw, yaw_rate, pitch, 0, roll, 0])
        print("yaw =", yaw)
        # Reads all available vision updates
        vision_updates = self.vision.get_estimated_global_pose()
        std_devs = (0.7, 0.7, 999999.0)  # Standard deviations for x, y, and theta
        
        # For each vision update, we pass it to the drivetrain for fusion
        for pose, timestamp in vision_updates:
            # We pass the vision pose and timestamp to the drivetrain
            # This is where the Kalman Filter fusion happens
            self.drivetrain.add_vision_measurement(pose, timestamp, std_devs)

        
    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        # return commands2.cmd.print_("No autonomous command configured")
        #return self.auto_chooser.getSelected()
        return commands2.cmd.none()