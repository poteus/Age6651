from commands2 import Command, Subsystem
from phoenix6 import Hardware, Controls
import phoenix6

class Shooter(Subsystem):
    """
    Shooter subsystem for controlling the robot's shooter mechanism.
    """

    def init(self):
        """
        Initialize the shooter subsystem.
        """
        self.shooter_motor = Hardware.TalonFX(51)
        self.shooter_motor.setInverted(False)

        self.voltage_request = Controls.VoltageOut(0) 

        # Configuration for motor
        configurator = self.shooter_motor.Configurator
        
        CFG = phoenix6.configs.TalonFXConfiguration()
        CFG.current_limits.stator_current_limit_enable = True
        CFG.current_limits.stator_current_limit = 60.0
        # Apply configuration
        configurator.apply(CFG)
    

    def run_shooter(self, voltage : float):
        """
        Run the shooter motor at the specified voltage.
        
        :param voltage: Voltage to set the shooter motor to.
        """

        self.shooter_motor.control(self.voltage_request.with_output(voltage))

    def stop_shooter(self):
        """
        Stop the shooter motor.
        """

        self.shooter_motor.control(self.voltage_request.with_output(0))