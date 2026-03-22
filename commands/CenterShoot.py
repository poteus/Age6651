from commands2 import Command, waitcommand
from subsystems.indexer import Indexer
from subsystems.shooter import Shooter

class CenterShoot(Command):
    def __init__(self, shooter:Shooter, indexer:Indexer):
        Command.__init__(self)

        self.shooter = shooter
        self.indexer = indexer
        
    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        # self.elevator.setElevatorFloor(self.floor)
        self.shooter.shoot_rps(40, 0)
        self.indexer.indexer_control_rps()
        waitcommand.WaitCommand(10)
        self.shooter.stop()
        self.indexer.stop_all()

    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return True

