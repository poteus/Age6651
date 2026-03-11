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
        self.mt1_subscribers = {}
        self.mode_publishers = {}

        for name in limelight_names:
            table = self.inst.getTable(name)
            self.subscribers[name] = table.getFloatArrayTopic("botpose_wpiblue").subscribe([])
            self.orientation_publishers[name] = table.getFloatArrayTopic("robot_orientation_set").publish()

            # Create and store the publisher so it doesn't get garbage collected
            self.mode_publishers[name] = table.getIntegerTopic("imumode_set").publish()

            # Subscribe to the MT1 topic specifically for seeding
            self.mt1_subscribers[name] = table.getFloatArrayTopic("botpose_orb_wpiblue").subscribe([])

            # --- IMU MODE CONFIGURATION ---
            # Create a publisher for the imumode_set topic
            mode_pub = table.getIntegerTopic("imumode_set").publish()
            
        self.force_imu_modes()

    def get_mt1_pose(self):
        """Specifically returns the MegaTag 1 (Pure Vision) pose for seeding."""
        updates = []
        for name, sub in self.mt1_subscribers.items():
            botpose = sub.get()
            # MT1 array is usually shorter or formatted differently, 
            # but usually contains 6 values [X, Y, Z, Roll, Pitch, Yaw]
            if len(botpose) >= 6:
                pose = Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]))
                # Use a dummy area since MT1 topics might not include it
                updates.append((name, pose, 1.0)) 
        return updates

    def force_imu_modes(self):
        """Forces cameras into their respective IMU modes"""
        for name, pub in self.mode_publishers.items():
            if name == "limelight-right":
                pub.set(2) # Mode 2: Internal + External Fusion (LL4)
            else:
                pub.set(1) # Mode 1: External IMU only (LL3)

    def get_estimated_global_pose(self):
        ''' Returns a list of tuples containing the name, pose, timestamp, and area of each valid vision update from the Limelights.'''
        all_vision_updates = []
        
        for name, sub in self.subscribers.items():
            botpose = sub.get()
            
            # botpose[7] = Tag Count
            # botpose[10] = Average Tag Area (% of image)
            if len(botpose) >= 11 and botpose[7] > 0: 
                timestamp = wpilib.Timer.getFPGATimestamp() - (botpose[6] / 1000.0)
                pose = Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]))
                area = botpose[10]
                
                # Return Name, Pose, Timestamp, and Area
                all_vision_updates.append((name, pose, timestamp, area))
        
        return all_vision_updates
    
    
    def patch_limelight_orientation(self, yaw: float, yaw_rate: float, pitch: float, roll: float):
        """
        Sends Pigeon 2.0 data to the Limelights.
        LL3 will use this as its primary source.
        LL4 will fuse this with its internal high-speed gyro.
        """
        # Limelight expects: [Yaw, YawRate, Pitch, PitchRate, Roll, RollRate]
        # We provide 0.0 for Pitch/Roll rates as they are less critical for 2D pose
        orientation_data = [yaw, yaw_rate, pitch, 0.0, roll, 0.0]

        for name in self.limelight_names:
            if name in self.orientation_publishers:
                self.orientation_publishers[name].set(orientation_data)