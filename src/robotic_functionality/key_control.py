#!/usr/bin/env python3

import os
import subprocess
import signal
from pynput import keyboard

INVOKE_PY_INTERPRETER = "python3"
L_ARM_SCRIPT = "key_arms.py -d l"
R_ARM_SCRIPT = "key_arms.py -d r"
BASE_SCRIPT = "key_base.py"
GRIPPERS_SCRIPT = "key_grippers.py"

scripts = [L_ARM_SCRIPT, R_ARM_SCRIPT, BASE_SCRIPT, GRIPPERS_SCRIPT]

processes = []
for script in scripts:
    process = subprocess.Popen([script])
    processes.append(process)

def on_key_press(key):
    if hasattr(key, 'char') and key.char is not None:
        if key.char == 'v':
            for pid in processes:
                os.kill(int(pid), signal.SIGTERM)

with keyboard.Listener(
        on_press=lambda key: on_key_press(key)
    ) as listener:
        listener.join()