#!/usr/bin/env python

import subprocess

INVOKE_PY_INTERPRETER = "python3"
ARMS_SCRIPT = "key_arms.py"
BASE_SCRIPT = "key_base.py"
GRIPPERS_SCRIPT = "key_grippers.py"

scripts = [ARMS_SCRIPT, BASE_SCRIPT, GRIPPERS_SCRIPT]

processes = []
for script in scripts:
    process = subprocess.Popen([INVOKE_PY_INTERPRETER, script])
    processes.append(process)

for process in processes:
    process.wait()