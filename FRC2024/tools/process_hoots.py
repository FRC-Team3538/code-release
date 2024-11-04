import sys
import os
import time
import subprocess
from typing import List
import glob

if len(sys.argv) != 2:
    print("Usage: process_hoots.py /path/to/hoots/directory")

dest_folder = set(os.listdir(f"{sys.argv[1]}\\converted"))

tasks: List[subprocess.Popen] = []

for entry in os.listdir(f"{sys.argv[1]}\\raw"):
    if f"{entry}.wpilog" in dest_folder:
        print(f"{entry} has already been processed; skipping")
        continue

    args = [
        "E:\\tools\\owlet.exe",
        f"{sys.argv[1]}\\raw\\{entry}",
        f"{sys.argv[1]}\\converted\\{entry}.wpilog",
        "-f",
        "wpilog",
    ]
    tasks.append(subprocess.Popen(args))

while True:
    if all(task.poll() is not None for task in tasks):
        print("finished!")
        break

    time.sleep(1)
