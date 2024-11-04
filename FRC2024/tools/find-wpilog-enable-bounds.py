from wpiutil.log import DataLog, DataLogReader
import sys

if len(sys.argv) != 2:
  print("Usage: find-wpilog-enable-bounds.py [wpilog]")
  sys.exit()

rdr = DataLogReader(sys.argv[1])

id = 0

for packet in rdr:
  if packet.isStart():
    start = packet.getStartData()
    if start.name in ('RobotEnable', '/DriverStation/Enabled'):
      id = start.entry
      break

print(f"Found RobotEnable with entry id {id}")

cval = False
edges = []

i = 0

for packet in rdr:
  if packet.getEntry() == id:
    if cval ^ packet.getBoolean():
      edges.append(packet.getTimestamp() / 1e6)

    cval = packet.getBoolean()

  if i % 100000 == 0:
    print(f"{i}, {packet.getTimestamp() / 1e6}")

  i += 1

for i in range(len(edges)):
  print(f"{i % 2 == 0}, {edges[i]}")


