import csv
import numpy as np
from pprint import pprint

filename = "test.csv"  # replace with the actual filename

# create empty lists to store the data
timestamps = []
radiations = []
estimations = []
points_list =[]

# read the CSV file
with open( ilename, "r") as f:
    reader = csv.reader(f, delimiter=",")
    next(reader)  # skip the header row
    for row in reader:
        # extract the data from each row
        timestamp = row[0]
        points = [[float(row[i]), float(row[i + 1]), float(row[i + 2])] for i in range(1, len(row) - 1, 3)]
        # append the data to the appropriate lists
        timestamps.append(timestamp)
        points_list.append((timestamp, points))
        estimations.append(points)

# convert the lists to numpy arrays
timestamps = np.array(timestamps)

print("time + points ")
pprint(points_list)
