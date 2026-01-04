from commands2 import Command, Subsystem
import math
from phoenix6 import swerve, units, utils
from typing import Callable, overload
from wpilib import DriverStation
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds
from wpilib import SmartDashboard
from wpimath.controller import PIDController
from wpilib import Timer
import commands2
from choreo_utils import ChoreoTrajectory


class CommandSwerveDrivetrain(Subsystem, swerve.SwerveDrivetrain):
    """
    Class that extends the Phoenix 6 SwerveDrivetrain class and implements
    Subsystem so it can easily be used in command-based projects.
    """

    _BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0)
    """Blue alliance sees forward as 0 degrees (toward red alliance wall)"""
    _RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180)
    """Red alliance sees forward as 180 degrees (toward blue alliance wall)"""

    @overload
    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        modules: list[swerve.SwerveModuleConstants],
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drive_motor_type:     Type of the drive motor
        :type drive_motor_type:      type
        :param steer_motor_type:     Type of the steer motor
        :type steer_motor_type:      type
        :param encoder_type:         Type of the azimuth encoder
        :type encoder_type:          type
        :param drivetrain_constants: Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:  swerve.SwerveDrivetrainConstants
        :param modules:              Constants for each specific module
        :type modules:               list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        odometry_update_frequency: units.hertz,
        modules: list[swerve.SwerveModuleConstants],
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drive_motor_type:            Type of the drive motor
        :type drive_motor_type:             type
        :param steer_motor_type:            Type of the steer motor
        :type steer_motor_type:             type
        :param encoder_type:                Type of the azimuth encoder
        :type encoder_type:                 type
        :param drivetrain_constants:        Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:         swerve.SwerveDrivetrainConstants
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :type odometry_update_frequency:    units.hertz
        :param modules:                     Constants for each specific module
        :type modules:                      list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        odometry_update_frequency: units.hertz,
        odometry_standard_deviation: tuple[float, float, float],
        vision_standard_deviation: tuple[float, float, float],
        modules: list[swerve.SwerveModuleConstants],
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drive_motor_type:            Type of the drive motor
        :type drive_motor_type:             type
        :param steer_motor_type:            Type of the steer motor
        :type steer_motor_type:             type
        :param encoder_type:                Type of the azimuth encoder
        :type encoder_type:                 type
        :param drivetrain_constants:        Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:         swerve.SwerveDrivetrainConstants
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :type odometry_update_frequency:    units.hertz
        :param odometry_standard_deviation: The standard deviation for odometry calculation
                                            in the form [x, y, theta]ᵀ, with units in meters
                                            and radians
        :type odometry_standard_deviation:  tuple[float, float, float]
        :param vision_standard_deviation:   The standard deviation for vision calculation
                                            in the form [x, y, theta]ᵀ, with units in meters
                                            and radians
        :type vision_standard_deviation:    tuple[float, float, float]
        :param modules:                     Constants for each specific module
        :type modules:                      list[swerve.SwerveModuleConstants]
        """
        ...

    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        arg0=None,
        arg1=None,
        arg2=None,
        arg3=None,
    ):
        Subsystem.__init__(self)
        swerve.SwerveDrivetrain.__init__(
            self, drive_motor_type, steer_motor_type, encoder_type,
            drivetrain_constants, arg0, arg1, arg2, arg3
        )

        self._has_applied_operator_perspective = False
        """Keep track if we've ever applied the operator perspective before or not"""
        
        # Swerve request to apply during path following
        self._apply_robot_speeds = swerve.requests.ApplyRobotSpeeds()
        
        # PID for Choreo path following
        self.x_controller = PIDController(3.0, 0.0, 0.0) 
        self.y_controller = PIDController(3.0, 0.0, 0.0)
        self.heading_controller = PIDController(2.0, 0.0, 0.0)
        self.heading_controller.enableContinuousInput(-math.pi, math.pi)

        # Pre-created chassis speeds request for performance
        self._chassis_speeds_request = swerve.requests.ApplyRobotSpeeds()

    def apply_request(
        self, request: Callable[[], swerve.requests.SwerveRequest]
    ) -> Command:
        """
        Returns a command that applies the specified control request to this swerve drivetrain.

        :param request: Lambda returning the request to apply
        :type request: Callable[[], swerve.requests.SwerveRequest]
        :returns: Command to run
        :rtype: Command
        """
        return self.run(lambda: self.set_control(request()))

    
    def periodic(self):
        # Periodically try to apply the operator perspective.
        # If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
        # This allows us to correct the perspective in case the robot code restarts mid-match.
        # Otherwise, only check and apply the operator perspective if the DS is disabled.
        # This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
        if not self._has_applied_operator_perspective or DriverStation.isDisabled():
            alliance_color = DriverStation.getAlliance()
            if alliance_color is not None:
                self.set_operator_perspective_forward(
                    self._RED_ALLIANCE_PERSPECTIVE_ROTATION
                    if alliance_color == DriverStation.Alliance.kRed
                    else self._BLUE_ALLIANCE_PERSPECTIVE_ROTATION
                )
                self._has_applied_operator_perspective = True

    def add_vision_measurement(self, vision_robot_pose: Pose2d, timestamp: units.second, vision_measurement_std_devs: tuple[float, float, float] | None = None):
        """
        Adds a vision measurement to the Kalman Filter. This will correct the
        odometry pose estimate while still accounting for measurement noise.

        Note that the vision measurement standard deviations passed into this method
        will continue to apply to future measurements until a subsequent call to
        set_vision_measurement_std_devs or this method.

        :param vision_robot_pose:           The pose of the robot as measured by the vision camera.
        :type vision_robot_pose:            Pose2d
        :param timestamp:                   The timestamp of the vision measurement in seconds.
        :type timestamp:                    second
        :param vision_measurement_std_devs: Standard deviations of the vision pose measurement
                                            in the form [x, y, theta]ᵀ, with units in meters
                                            and radians.
        :type vision_measurement_std_devs:  tuple[float, float, float] | None
        """
        swerve.SwerveDrivetrain.add_vision_measurement(self, vision_robot_pose, utils.fpga_to_current_time(timestamp), vision_measurement_std_devs)

    def get_choreo_command(self, trajectory:ChoreoTrajectory, trajectory_name: str):
        ''' Returns a command that follows the specified Choreo trajectory. '''
       
        timer = Timer()
        return commands2.FunctionalCommand(
            # On Start: Reset the robot pose to the start of the path
            lambda: (
                timer.restart(),
                self.reset_pose(trajectory.get_initial_pose(
                    DriverStation.getAlliance() == DriverStation.Alliance.kRed))
            ),
            # On Execute: Drive using our helper
            lambda: self._drive_from_choreo(trajectory, timer.get()),
            # On End: Stop the robot
            lambda interrupted: self.set_control(swerve.requests.Idle()),
            # Is Finished: When the timer exceeds trajectory duration
            lambda: timer.hasElapsed(trajectory.get_total_time()),
            self
        )

    def _drive_from_choreo(self, trajectory:ChoreoTrajectory, time):
        ''' Follows the trajectory from Choreo with PID corrections. '''
        is_red = DriverStation.getAlliance() == DriverStation.Alliance.kRed
        sample = trajectory.sample_at(time, is_red)
        pose = self.get_state().pose

        # Calculate corrections (Feedback) for the 3 velocities
        vx_feedback = self.x_controller.calculate(pose.X(), sample.x)
        vy_feedback = self.y_controller.calculate(pose.Y(), sample.y)
        omega_feedback = self.heading_controller.calculate(
            pose.rotation().radians(), sample.heading
        )

        # Combine with Choreo's velocities (Feedforward)
        vx_total = sample.vx + vx_feedback
        vy_total = sample.vy + vy_feedback
        omega_total = sample.omega + omega_feedback

        # Convert Field-Relative to Robot-Relative
        # This tells the robot: "I want to go X meters/sec toward the Red Wall, 
        # regardless of which way I am currently facing."
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vx_total,
            vy_total,
            omega_total,
            pose.rotation() # The robot uses its current gyro heading to do the math
        )
        
        # Optional: Add discretization to prevent "curving"
        speeds = ChassisSpeeds.discretize(speeds, 0.020)

        self.set_control(self._chassis_speeds_request.with_speeds(speeds))

        # Log the errors (Target - Actual)
        SmartDashboard.putNumber("Auto/X_Error", sample.x - pose.X())
        SmartDashboard.putNumber("Auto/Y_Error", sample.y - pose.Y())
        SmartDashboard.putNumber("Auto/Target_VX", sample.vx)
        SmartDashboard.putNumber("Auto/Actual_VX", self.get_state().speeds.vx)
