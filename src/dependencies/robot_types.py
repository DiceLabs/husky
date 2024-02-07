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
