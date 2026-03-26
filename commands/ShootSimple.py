from commands2 import Command, waitcommand
from subsystems.indexer import Indexer
from subsystems.shooter import Shooter
from subsystems.intake import Intake

class ShootSimple(Command):
    def __init__(self, shooter:Shooter, indexer:Indexer):
        Command.__init__(self)

        self.shooter = shooter
        self.indexer = indexer
        
    def initialize(self) -> None:
        self.shooter.shoot_rps(32, 0)
        # return super().initialize()
    
    def execute(self) -> None:
        self.indexer.indexer_control_rps()
       
    def end(self, interrupted: bool) -> None:
        self.shooter.stop()
        self.indexer.stop_all()
        # return super().end(interrupted)

    def isFinished(self) -> bool:
        return False

