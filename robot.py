# TODO make robot class

import wpilib
import wpilib.drive
import phoenix6

class robot(wpilib.TimedRobot):
    def robotInit(self):
        #init drive motors

        self.leftDrive = phoenix6.hardware.TalonFX(1)
        self.rightDrive = phoenix6.hardware.TalonFX(2)
        self.rightDrive.setInverted(True)
        self.robotDrive = wpilib.drive.DifferentialDrive(self.leftDrive, self.rightDrive)
        
        self.controller = wpilib.XboxController(0)
        
        self.timer = wpilib.Timer() 

    def autoInit(self):
        #prepare for auto
        self.timer.restart()

    def autoPeriodic(self):
        #autonomous movement
        if self.timer.get() < 2.0:
            self.robotDrive.arcadeDrive(.5, 0, False)
        else:
            self.robotDrive.stopMotor()

    def teleopInit(self):
        #init for teleop
        pass

    def teleopPeriodic(self):
        #teleop loop
        self.robotDrive.arcadeDrive(
            -self.controller.getLeftY(), -self.controller.getRightX()
        )

    def testInit(self):
        pass
    def testPeriodic(self):
        pass