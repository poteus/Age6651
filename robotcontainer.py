# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib

import commands2
import commands2.button
from commands2 import waitcommand

from commands.SwerveJoystickCmd2 import SwerveJoystickCmd2

from constants import OIConstants

from wpilib import SendableChooser
from wpimath.geometry import Pose2d, Rotation2d

class RobotContainer:
    '''
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    '''

    def __init__(self):
        '''The container for the robot. Contains subsystems, OI devices, and commands.'''
        self.swerveSubsystem = SwerveSubsystem()

        # The driver's controller
        self.driverController = wpilib.PS5Controller(OIConstants.kDriverControllerPort)
  
        self.swerveSubsystem.setDefaultCommand(
           SwerveJoystickCmd2(
               self.swerveSubsystem,
               lambda : self.driverController.getRawAxis(OIConstants.kDriverYAxis),
               lambda : self.driverController.getRawAxis(OIConstants.kDriverXAxis),
               lambda : self.driverController.getRawAxis(OIConstants.kDriverXRotAxis),
               lambda : self.driverController.getRawAxis(OIConstants.kDriverYRotAxis),
               lambda : self.driverController.getSquareButton(),
               lambda : self.driverController.getCircleButton()
           )
        )

        # Build an auto chooser. This will use Commands.none() as the default option.
        # self.autoChooser = AutoBuilder.buildAutoChooser()
        # wpilib.SmartDashboard.putData("Auto Chooser", self.autoChooser)

        autocommand0: commands2.cmd.Command = None

        #Poses for Autonomous
        self.Pose21R = Pose2d(5.9, 4.2, Rotation2d.fromDegrees(0)) # Pose for Right 22
        self.Pose10R = Pose2d(11, 3.8, Rotation2d.fromDegrees(180)) # Pose for Right 10
        self.Pose22R = Pose2d(4.8, 2.8, Rotation2d.fromDegrees(0)) # Pose for Right 22
          

        self.sendableChooser = SendableChooser()
        
        self.sendableChooser.setDefaultOption("Nothing",autocommand0)
        
        # Configure the button bindings
        self.configureButtonBindings()
        
    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created via the button
        factories on commands2.button.CommandGenericHID or one of its
        subclasses (commands2.button.CommandJoystick or command2.button.CommandXboxController).
        """
        pass
    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main :class:`.Robot` class.
        there's more at the door
        :returns: the command to run in autonomous
        """
        return self.sendableChooser.getSelected()