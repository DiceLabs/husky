#!/usr/bin/env python3

from dataclasses import dataclass

@dataclass
class CameraRequest():
    trigger: int

@dataclass
class CameraResponse():
    depth: float
    dx: float
    dy: float

@dataclass
class Object():
    x : int
    y : int
    z : int

