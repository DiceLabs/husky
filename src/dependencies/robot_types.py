from dataclasses import dataclass

@dataclass
class Position():
    def __init__(self, x:float=0.0, y:float=0.0, z:float=0.0):
        self.x :float = x
        self.y :float = y
        self.z :float = z

@dataclass 
class Quaternion():
    def __init__(self, w:float=0.0, x:float=0.0, y:float=0.0, z:float=0.0):
        self.w :float = w
        self.x :float = x
        self.y :float = y
        self.z :float = z

@dataclass
class Euler():
    def __init__(self, yaw:float=0.0, pitch:float=0.0, roll:float=0.0):
        self.yaw :float = yaw
        self.pitch :float = pitch
        self.roll :float = roll

@dataclass 
class PoseM():
    def __init__(self, orientation:Euler=Euler(), position:Position=Position()):
        self.orientation = Euler(orientation.yaw, orientation.pitch, orientation.roll)
        self.position = Position(position.x, position.y, position.z)

@dataclass 
class LinearVelocity():
    def __init__(self, x:float=0.0, y:float=0.0, z:float=0.0):
        self.x = x
        self.y = y
        self.z = z

@dataclass 
class AngularVelocity():
    def __init__(self, x:float=0.0, y:float=0.0, z:float=0.0):
        self.x = x 
        self.y = y
        self.z = z
