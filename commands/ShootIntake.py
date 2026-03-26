from commands2 import Command, waitcommand
from subsystems.indexer import Indexer
from subsystems.shooter import Shooter
from subsystems.intake import Intake

class ShootIntake(Command):
    def __init__(self, shooter:Shooter, indexer:Indexer, intake:Intake):
        Command.__init__(self)

        self.shooter = shooter
        self.indexer = indexer
        self.intake = intake
        
    def initialize(self) -> None:
        self.shooter.shoot_rps(32, 0)    
        # return super().initialize()
    
    def execute(self) -> None:
        self.indexer.indexer_control_rps()
        self.intake.set_intake_dutyCycle()
    
    def end(self, interrupted: bool) -> None:
        self.shooter.stop()
        self.indexer.stop_all()
        self.intake.stop()
        # return super().end(interrupted)

    def isFinished(self) -> bool:
        return False

