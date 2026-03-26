from commands2 import Command
from subsystems.indexer import Indexer
from subsystems.shooter import Shooter
from subsystems.intake import Intake

class ShootDistance(Command):
    def __init__(self, shooter:Shooter, indexer:Indexer, intake:Intake, intakeOn:bool):
        Command.__init__(self)

        self.shooter = shooter
        self.indexer = indexer
        self.intake = intake
        self.intakeOn = intakeOn
        
    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        
        self.shooter.shoot_with_distance()
        self.indexer.indexer_control_rps()
        if self.intakeOn:
            self.intake.set_intake_dutyCycle()
       
    def end(self, interrupted: bool) -> None:
        self.shooter.stop()
        self.indexer.stop_all()
        self.intake.stop()
        # return super().end(interrupted)

    def isFinished(self) -> bool:
        return False

