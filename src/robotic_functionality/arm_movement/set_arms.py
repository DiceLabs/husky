#!/usr/bin/env python3

import subprocess

PY_INTERPRETER = 'python3'
L_ARM_SCRIPT = ["set_arm.py", "-d", "l"]
R_ARM_SCRIPT = ["set_arm.py", "-d", "r"]
left_arm_process = subprocess.Popen([PY_INTERPRETER, L_ARM_SCRIPT])
right_arm_process = subprocess.Popen([PY_INTERPRETER, R_ARM_SCRIPT])
