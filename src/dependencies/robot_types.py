from dataclasses import dataclass

@dataclass
class Position():
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

@dataclass 
class Quaternion():
    def __init__(self, w=0, x=0, y=0, z=0):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

@dataclass
class Euler():
    def __init__(self, yaw=0, pitch=0, roll=0):
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll

@dataclass 
class Pose():
    quaternion: Quaternion = Quaternion()
    position: Position = Position()

@dataclass 
class LinearVelocity():
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

@dataclass 
class AngularVelocity():
    def __init__(self, x=0, y=0, z=0):
        self.x = x 
        self.y = y
        self.z = z
