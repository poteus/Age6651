import wpilib
from phoenix6 import swerve
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
import ntcore # NetworkTables to talk to Limelights

class Vision:
    def __init__(self, limelight_names):
        self.limelight_names = limelight_names
        self.inst = ntcore.NetworkTableInstance.getDefault()
        
        # Dictionary to hold the tables for each camera
        self.tables = {name: self.inst.getTable(name) for name in limelight_names}

    def get_estimated_global_pose(self):
        """
        Iterates through all configured limelights and returns 
        the best pose estimate found.
        """
        all_vision_updates = []
        
        for name, table in self.tables.items():
            # botpose_wpiblue is the standard for 2026 
            # (X, Y, Z, Roll, Pitch, Yaw, Latency, TagCount, TagSpan, AvgDist, AvgArea)
            botpose = table.getEntry("botpose_wpiblue").getDoubleArray([0]*11)
            
            # Only use the data if the Limelight actually sees a tag
            if botpose[7] > 0: 
                timestamp = wpilib.Timer.getFPGATimestamp() - (botpose[6] / 1000.0)
                pose = Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]))
                all_vision_updates.append((pose, timestamp))
        
        return all_vision_updates