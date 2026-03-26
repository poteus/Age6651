from commands2 import Command, waitcommand
from subsystems.intake import Intake

class RunIntake(Command):
    def __init__(self, intake:Intake):
        Command.__init__(self)

        self.intake = intake
        
    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        self.intake.set_intake_dutyCycle()
       
    def end(self, interrupted: bool) -> None:
        self.intake.stop()
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return False

