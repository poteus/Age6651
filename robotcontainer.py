
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
from subsystems.shooter import Shooter

import wpilib
from wpilib import DriverStation
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians


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
            #self.vision = Vision(["limelight-front"])
            from generated.Murphy_tuner_constants import TunerConstants 

        self._max_speed = (
            0
        )  # speed_at_12_volts desired top speed

        self._max_angular_rate = rotationsToRadians(
            0
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(
                self._max_angular_rate * 0.1
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
            .with_steer_request_type(swerve.requests.SwerveModule.SteerRequestType.MOTION_MAGIC_EXPO)
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        self._logger = Telemetry(self._max_speed)

        self._joystick = CommandXboxController(0)

        self.drivetrain = TunerConstants.create_drivetrain()

        self.shooter = Shooter()

        # Configure the button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        # --- LOGGER CONTROL ---
        # Press "Start" button to begin logging
        # self._joystick.start().onTrue(
        #     commands2.cmd.runOnce(lambda: SignalLogger.start())
        # )
        # # We'll use the Back button for both stopping the logger AND resetting jams
        # self._joystick.back().onTrue(
        #     commands2.cmd.runOnce(lambda: SignalLogger.stop())
        # ).onTrue(
        #     commands2.cmd.runOnce(lambda: self.shooter.reset_jam())
        # )

        # --- Characterization Bindings ---
        # Quasistatic: Motor ramps up speed slowly
        # self._joystick.y().whileTrue(self.shooter.sys_id_quasistatic(SysIdRoutine.Direction.kForward))
        # self._joystick.x().whileTrue(self.shooter.sys_id_quasistatic(SysIdRoutine.Direction.kReverse))
        
        # # Dynamic: Motor gets a sudden 7V "step"
        # self._joystick.b().whileTrue(self.shooter.sys_id_dynamic(SysIdRoutine.Direction.kForward))
        # self._joystick.a().whileTrue(self.shooter.sys_id_dynamic(SysIdRoutine.Direction.kReverse))


        self._joystick.rightBumper().whileTrue(
                    self.shooter.run(self.shooter.run_shooter_pid)
                                      ).onFalse(
                    self.shooter.runOnce(self.shooter.stop_shooter)
                )

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return commands2.cmd.print_("No autonomous command configured")
