from wpimath.geometry import Translation2d

class SwerveConstants:
    # Base of robot to use
    robotBase = "Crimson"

    # Dictionary of all the wheel locations in a 2dplane
    wheelLocations2D = { 
        "Crimson" : {      
            "FrontLeft" : Translation2d(.286, .286),
            "FrontRight" : Translation2d(.286, -.286),
            "BackLeft" : Translation2d(-.286, .286),
            "BackRight" : Translation2d(-.286, -.286),
        },
    
        "Murphy" : {
            "FrontLeft" : Translation2d(.267, .267),
            "FrontRight" : Translation2d(.267, -.267),
            "BackLeft" : Translation2d(-.267, .267),
            "BackRight" : Translation2d(-.267, -.267), 
        }, 
    }
