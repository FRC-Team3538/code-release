import json
import csv
import sys


def main():
    with open(sys.argv[1]) as f:
        data = json.load(f)

    with open(sys.argv[1] + ".csv", "w", newline='') as f:
        wrtr = csv.DictWriter(f, fieldnames=data[0].keys())
        wrtr.writeheader()
        wrtr.writerows(data)


if __name__ == "__main__":
    main()
