import sys
import os
import time
import subprocess
from typing import List
import glob
import re
import shutil

if len(sys.argv) != 2:
    print("Usage: sort-logs.py /path/to/logs/directory")

tasks: List[subprocess.Popen] = []

DEFAULT_DESTINATION = "E:\\quarantine"

rules = [
    ("^Log_[0-9-]+_[0-9-]+_[a-zA-Z]\d+.wpilog$", "E:\\event logs\\wpilogs"),
    ("^Log_[0-9-]+_[0-9-]+\.wpilog$", "E:\\practice logs\\wpilogs"),
    ("^[A-Z]+_[A-Z]\d+_[a-zA-Z0-9]+_[0-9-]+_[0-9-]+\.hoot.wpilog$", "E:\\event logs\\hoots\\converted"),
    ("^[A-Z]+_[A-Z]\d+_[a-zA-Z0-9]+_[0-9-]+_[0-9-]+\.hoot$", "E:\\event logs\\hoots\\raw"),
    ("^[a-zA-Z0-9]+_[0-9-]+_[0-9-]+\.hoot.wpilog$", "E:\\practice logs\\hoots\\converted"),
    ("^[a-zA-Z0-9]+_[0-9-]+_[0-9-]+\.hoot$", "E:\\practice logs\\hoots\\raw"),
]

for entry in os.listdir(sys.argv[1]):
    destination = DEFAULT_DESTINATION
    for rule in rules:
        if re.match(rule[0], entry):
            destination = rule[1]
            break

    print(f"moving {entry} to {destination}")

    shutil.move(f"{sys.argv[1]}\\{entry}", f"{destination}\\{entry}")
    
print("finished!")
