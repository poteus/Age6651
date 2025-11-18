import wpilib
import commands2
import math
from commands2 import waitcommand
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState

# Measurements from center of robot to each of the wheeles; FL, FR, BL, BR ###

# Field oriented drive example given by WPILIB
speeds = ChassisSpeeds.fromFieldRelativeSpeeds(2.0, 2.0, math.pi / 2.0, Rotation2d.fromDegrees(45.0))

