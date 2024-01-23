#!/usr/bin/env python

import subprocess
import key_arms, key_base, key_grippers

PROGRAM_TO_INVOKE = "python3"
ARMS_SCRIPT = "key_arms.py"
BASE_SCRIPT = "key_base.py"
GRIPPERS_SCRIPT = "key_grippers"

scripts = [ARMS_SCRIPT, BASE_SCRIPT, GRIPPERS_SCRIPT]

processes = []
for script in scripts:
    process = subprocess.Popen([PROGRAM_TO_INVOKE, script])
    processes.append(process)

for process in processes:
    process.wait()