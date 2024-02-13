#!/usr/bin/env python3

import subprocess
from pynput import keyboard

INVOKE_PY_INTERPRETER = "python3"
L_ARM_SCRIPT = ["key_arms.py", "-d", "l"]
R_ARM_SCRIPT = ["key_arms.py", "-d", "r"]
BASE_SCRIPT = ["key_base.py"]
GRIPPERS_SCRIPT = ["key_grippers.py"]
EXIT_KEY = 'v'

def key_valid(key):
     return hasattr(key, 'char') and key.char is not None

def on_key_press(key, processes):
    if key_valid(key):
        if key.char == EXIT_KEY:
            for pid in processes:
                pid.kill()
            return False # Exit

def fork_processes(scripts):
    processes=[]
    for script in scripts:
        process = subprocess.Popen(script)
        processes.append(process)
    return processes

if __name__ == "__main__":
    scripts = [L_ARM_SCRIPT, R_ARM_SCRIPT, BASE_SCRIPT, GRIPPERS_SCRIPT]
    processes = fork_processes(scripts)
    with keyboard.Listener(
        on_press=lambda key: on_key_press(key, processes)
    ) as listener:
        listener.join()