
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

       

        self._joystick.leftBumper().whileTrue(
            AutoBuilder.pathfindToPose(
                self.target_pose,
                self.constraints,
                goal_end_vel=0.0
            )
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
        self.vision.patch_limelight_orientation([yaw+180, yaw_rate, pitch, 0, roll, 0])

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
        return self.auto_chooser.getSelected()
