
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
from subsystems.intake import Intake
from generated.Crimson_tuner_constants import TunerConstants 

import wpilib
from wpilib import DriverStation, SmartDashboard
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

        # Limelight Initialization
        self.vision = Vision(["limelight-right", "limelight-back"])

        # Subsystem Drive System ------------------------------
        self.drivetrain = TunerConstants.create_drivetrain()
        self.drivetrain.vision = self.vision
        
        # Constants
        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed

        self._max_angular_rate = rotationsToRadians(
            0.75
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

        # Subsystems ---------------------------------
        self.shooter = Shooter(self._logger)
        self.indexer = Indexer(self.shooter)
        self.intake = Intake()
        self.turret = Turret()

        # Configure the button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -self._joystick.getLeftY() * self._max_speed
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        -self._joystick.getLeftX() * self._max_speed
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -self._joystick.getRightX() * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )

        self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._joystick.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
                )
            )
        )

        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        (self._joystick.back() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.back() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self._joystick.start() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.start() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )

        # Right Trigger -> Shoot Only
        self._joystick.rightTrigger().whileTrue(
            commands2.cmd.run(lambda: self.shooter.shoot_control_dash(), self.shooter).alongWith(
                commands2.cmd.run(lambda: self.indexer.indexer_control_rps(),self.indexer)).alongWith(
                    commands2.cmd.run(lambda: self.intake.set_intake_dutyCycle(.4), self.intake))
        ).onFalse(commands2.cmd.runOnce(
                lambda:self.shooter.stop(), self.shooter).alongWith(
                    commands2.cmd.runOnce(lambda: self.indexer.stop_all(), self.indexer)).alongWith(
                        commands2.cmd.runOnce(lambda:self.intake.stop())))
            
        # Left Trigger -> Index Only
        self._joystick.leftTrigger().whileTrue(
            commands2.cmd.run(
                lambda: self.indexer.set_velocity(40.0))  # Placeholder RPS value for indexing
        ).onFalse(commands2.cmd.runOnce(
                lambda: self.indexer.stop()))

        # Left Bumper -> Intake Only
        self._joystick.leftBumper().whileTrue(
            commands2.cmd.run(
                lambda: self.intake.set_intake_dutyCycle(0.40))
        ).onFalse(commands2.cmd.runOnce(lambda: self.intake.stop()
        ))

        # Right Bumper -> ChakaChaka Only
        self._joystick.rightBumper().whileTrue(
            commands2.cmd.run(lambda: self.shooter.shoot_control_dash(), self.shooter).alongWith(
                commands2.cmd.run(lambda: self.indexer.indexer_control_rps(),self.indexer))
        ).onFalse(commands2.cmd.runOnce(
                lambda:self.shooter.stop(), self.shooter).alongWith(
                    commands2.cmd.runOnce(lambda: self.indexer.stop_all(), self.indexer)))

        # self._joystick.leftBumper().whileTrue(
        #     commands2.cmd.run(
        #         lambda: self.intake.set_velocity(40.0))  # Placeholder RPS value for intaking
        # ).onFalse(commands2.cmd.run(lambda: self.intake.stop()
        # ))

        # self._joystick.povUp().whileTrue(
        #     commands2.cmd.runOnce(lambda: self.intake.set_shoulder_position(0))  # Placeholder position for "up"
        # )

        # self._joystick.povDown().whileTrue(
        #     commands2.cmd.runOnce(lambda: self.intake.set_shoulder_position(180))  # Placeholder position for "down"
        # )

        self._joystick.povLeft().whileTrue(
            commands2.cmd.runOnce(lambda: self.turret.aim_at_angle(40), self.turret)  # Placeholder angle for "left"
        )

        self._joystick.povRight().whileTrue(
            commands2.cmd.runOnce(lambda: self.turret.aim_at_angle(-15), self.turret)  # Placeholder angle for "right"
        )

        # --- LOGGER CONTROL ---
        # Using Left Stick Click to Start and Right Stick Click to Stop
        self._joystick.leftStick().onTrue(commands2.cmd.runOnce(lambda: SignalLogger.start()))
        self._joystick.rightStick().onTrue(commands2.cmd.runOnce(lambda: SignalLogger.stop()))

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

        # Use a button (e.g., Back button) to manually seed the Pigeon if it drifts
        self._joystick.back().onTrue(
            commands2.cmd.runOnce(lambda: self.drivetrain.seed_pigeon_with_vision())
        )

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return commands2.cmd.print_("No autonomous command configured")