
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
        #self.shooter = Shooter()
        #self.turret = Turret()
        self.drivetrain = TunerConstants.create_drivetrain()

        # Register the telemetry callback to log swerve, turret, and hood data
        # self.drivetrain.register_telemetry(
        #     lambda state: self._logger.telemeterize(
        #         state, 
        #         self.turret.motor.get_position().value, 
        #         self.shooter.hood.get_position().value
        #     )
        # )

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
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(lambda: swerve.requests.SwerveDriveBrake())
        )

        # --- LOGGER CONTROL ---
        # Using Left Stick Click to Start and Right Stick Click to Stop
        self._joystick.leftStick().onTrue(commands2.cmd.runOnce(lambda: SignalLogger.start()))
        self._joystick.rightStick().onTrue(commands2.cmd.runOnce(lambda: SignalLogger.stop()))

        # --- SHOOTER FLYWHEEL (Hold Right Bumper) ---
        rb = self._joystick.rightBumper()
        rb.and_(self._joystick.y()).whileTrue(self.shooter.sysIdFlywheelQuasistatic(SysIdRoutine.Direction.kForward))
        rb.and_(self._joystick.a()).whileTrue(self.shooter.sysIdFlywheelQuasistatic(SysIdRoutine.Direction.kReverse))
        rb.and_(self._joystick.b()).whileTrue(self.shooter.sysIdFlywheelDynamic(SysIdRoutine.Direction.kForward))
        rb.and_(self._joystick.x()).whileTrue(self.shooter.sysIdFlywheelDynamic(SysIdRoutine.Direction.kReverse))

        # --- INDEXER (Hold Left Bumper) ---
        lb = self._joystick.leftBumper()
        lb.and_(self._joystick.y()).whileTrue(self.indexer.sysIdQuasistatic(SysIdRoutine.Direction.kForward))
        lb.and_(self._joystick.a()).whileTrue(self.indexer.sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
        lb.and_(self._joystick.b()).whileTrue(self.indexer.sysIdDynamic(SysIdRoutine.Direction.kForward))
        lb.and_(self._joystick.x()).whileTrue(self.indexer.sysIdDynamic(SysIdRoutine.Direction.kReverse))

        # --- TURRET (Hold Left Trigger) ---
        lt = self._joystick.leftTrigger()
        # lt.and_(self._joystick.y()).whileTrue(self.turret.sysIdQuasistatic(SysIdRoutine.Direction.kForward))
        # lt.and_(self._joystick.a()).whileTrue(self.turret.sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
        # lt.and_(self._joystick.b()).whileTrue(self.turret.sysIdDynamic(SysIdRoutine.Direction.kForward))
        # lt.and_(self._joystick.x()).whileTrue(self.turret.sysIdDynamic(SysIdRoutine.Direction.kReverse))

        # --- HOOD (Hold Right Trigger) ---
        rt = self._joystick.rightTrigger()
        # rt.and_(self._joystick.y()).whileTrue(self.shooter.sysIdHoodQuasistatic(SysIdRoutine.Direction.kForward))
        # rt.and_(self._joystick.a()).whileTrue(self.shooter.sysIdHoodQuasistatic(SysIdRoutine.Direction.kReverse))
        # rt.and_(self._joystick.b()).whileTrue(self.shooter.sysIdHoodDynamic(SysIdRoutine.Direction.kForward))
        # rt.and_(self._joystick.x()).whileTrue(self.shooter.sysIdHoodDynamic(SysIdRoutine.Direction.kReverse))


        
    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        # return commands2.cmd.print_("No autonomous command configured")
        #return self.auto_chooser.getSelected()
        return commands2.cmd.none()