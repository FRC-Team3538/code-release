from wpiutil.log import DataLog, DataLogReader
import sys

print(sys.argv)
if len(sys.argv) != 4:
  print("Usage: strip-wpilog.py [wpilog] [start timestamp] [end timestamp]")

low = float(sys.argv[2]) * 1_000_000
high = float(sys.argv[3]) * 1_000_000

folder = "\\".join(sys.argv[1].split("\\")[:-1])
file = sys.argv[1].split("\\")[-1]
destination_file = f"{file[:-7]}_{sys.argv[2]}-{sys.argv[3]}.wpilog"

rdr = DataLogReader(sys.argv[1])
wrtr = DataLog(folder, destination_file, 0.25, rdr.getExtraHeader())

for packet in rdr:
  if packet.isStart():
    start = packet.getStartData()
    wrtr.start(start.name, start.type, start.metadata, packet.getTimestamp())
  elif packet.isFinish():
    finish = packet.getFinishEntry()
    wrtr.finish(finish, packet.getTimestamp())
  elif packet.isSetMetadata():
    metadata = packet.getSetMetadataData()
    wrtr.setMetadata(metadata.entry, metadata.metadata, packet.getTimestamp())
  elif low < packet.getTimestamp() < high:
    wrtr.appendRaw(packet.getEntry(), packet.getRaw(), packet.getTimestamp())

wrtr.flush()
