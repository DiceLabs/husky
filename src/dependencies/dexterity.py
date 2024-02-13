#!/usr/bin/env python

from enum import Enum

class Dexterity(Enum):
    LEFT = 0,
    RIGHT = 1
    def __str__(self):
        return self.name.lower()