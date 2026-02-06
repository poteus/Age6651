import wpilib
from phoenix6 import swerve
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
import ntcore # NetworkTables to talk to Limelights

class Vision:
    def __init__(self, limelight_names):
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.subscribers = {}
        
        self.limelight_names = limelight_names
        self.orientation_publishers = {}

        # Dictionary to hold the tables for each camera
        for name in limelight_names:
            table = self.inst.getTable(name)
            # We create a subscriber for the "botpose_wpiblue" topic
            # We use FloatArray because that is how Limelight sends botpose
            self.subscribers[name] = table.getFloatArrayTopic("botpose_wpiblue").subscribe([])
            self.orientation_publishers[name] = table.getFloatArrayTopic("robot_orientation_set").publish()

            table.getIntegerTopic("imumode_set").publish().set(1)

    def get_estimated_global_pose(self):
        """
        Iterates through all configured limelights and returns 
        the best pose estimate found.
        """
        all_vision_updates = []
        
        for name, sub in self.subscribers.items():
            # botpose_wpiblue is the standard for 2026 
            # (X, Y, Z, Roll, Pitch, Yaw, Latency, TagCount, TagSpan, AvgDist, AvgArea)
            botpose = sub.get()
            
            # Only use the data if the Limelight actually sees a tag
            if len(botpose) >= 8 and botpose[7] > 0: 
                # Calculate the timestamp of the measurement
                timestamp = wpilib.Timer.getFPGATimestamp() - (botpose[6] / 1000.0)
                pose = Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]))
                all_vision_updates.append((pose, timestamp))
        
        return all_vision_updates
    
    def patch_limelight_orientation(self,pigeon_stats):
        """
        
        """
        for name in self.limelight_names:
            if name in self.orientation_publishers:
                self.orientation_publishers[name].set(pigeon_stats)