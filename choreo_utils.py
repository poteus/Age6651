import json
import os
from math import pi
from wpilib import getDeployDirectory
from wpimath.geometry import Pose2d, Rotation2d

class ChoreoTrajectory:
    '''Loads a Choreo trajectory from a .traj file and provides methods to access samples.'''
    def __init__(self, name):
        # Path to deploy/choreo/name.traj
        file_path = os.path.join(getDeployDirectory(), "choreo", f"{name}.traj")
        
        with open(file_path, "r") as f:
            data = json.load(f)
            
            # Choreo files can have different structures. Let's check for them:
            if "samples" in data:
                self.samples = data["samples"]
            elif "paths" in data and name in data["paths"]:
                self.samples = data["paths"][name]["samples"]
            elif "params" in data and "waypoints" in data["params"]:
                self.samples = data["params"]["waypoints"]
            else:
                # If it's a newer format, the first key might be the path name
                first_key = list(data.keys())[0]
                if isinstance(data[first_key], dict) and "samples" in data[first_key]:
                    self.samples = data[first_key]["samples"]
                else:
                    raise KeyError(f"Could not find 'samples' in {name}.traj. Check Choreo export.")
    
    def get_initial_pose(self, is_red: bool):
        first = self.samples[0]
        # Extract the "val" from the dictionary
        raw_x = first["x"]["val"] if isinstance(first["x"], dict) else first["x"]
        raw_y = first["y"]["val"] if isinstance(first["y"], dict) else first["y"]
        raw_heading = first["heading"]["val"] if isinstance(first["heading"], dict) else first["heading"]

        x = raw_x if not is_red else 16.54 - raw_x
        y = raw_y if not is_red else 8.21 - raw_y
        heading = raw_heading if not is_red else raw_heading + pi
        return Pose2d(x, y, Rotation2d(heading))

    def sample_at(self, time: float, is_red: bool):
        '''Returns a sample at the given time, adjusted for alliance color.'''
        # Find the sample closest to the current time
        index = min(int(time / 0.02), len(self.samples) - 1)
        sample = self.samples[index]
        
        # Helper function to extract the 'val' if it's a dict, otherwise return the value
        def get_val(data):
            return data["val"] if isinstance(data, dict) else data

        # Define a helper object for the drivetrain to use
        class Sample:
            pass
        s = Sample()

        # Extract values using the helper
        raw_x = get_val(sample["x"])
        raw_y = get_val(sample["y"])
        raw_heading = get_val(sample["heading"])
        
        # Choreo trajectories usually provide velocity (vx, vy, omega)
        # We extract them safely as well
        raw_vx = get_val(sample.get("vx", 0))
        raw_vy = get_val(sample.get("vy", 0))
        raw_omega = get_val(sample.get("omega", 0))

        # Apply Alliance Flipping
        # Field width is ~16.54m, height is ~8.21m
        s.x = raw_x if not is_red else 16.54 - raw_x
        s.y = raw_y if not is_red else 8.21 - raw_y
        s.heading = raw_heading if not is_red else raw_heading + pi
        
        s.vx = raw_vx if not is_red else -raw_vx
        s.vy = raw_vy if not is_red else -raw_vy
        s.omega = raw_omega 
        
        return s

    def get_total_time(self):
        '''Returns the total duration of the trajectory in seconds.'''
        return len(self.samples) * 0.02