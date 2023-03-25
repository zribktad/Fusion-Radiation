import csv

prefix = "/home/tadeas/my_workspace/src/fusion_radiation/data/"

def read_estimations(filename):
    # create empty lists to store the data
    timestamps = []
    estimations = []

    with open(prefix+filename, "r") as f:
        reader = csv.reader(f, delimiter=",")
        next(reader)  # skip the header row
        points =[]
        for row in reader:
            # extract the data from each row
            if(len(row)==0):
                break
            timestamp = row[0]
           # if len(row)>1:      #repeat if estimates are not available
            points = [
                    [float(row[i]), float(row[i + 1]), float(row[i + 2])]
                    for i in range(1, len(row) - 1, 3)
                ]

            # append the data to the appropriate lists
            timestamps.append(timestamp)
            estimations.append(points)
    timestamps = [str(float(timestamp) - float(min(timestamps))) for timestamp in timestamps]

    return timestamps, estimations


def read_radiations(filename):
    # create empty list to store the data
    radiations = []

    with open(prefix+filename, "r") as f:
        reader = csv.reader(f, delimiter=",")
        next(reader)  # skip the header row
        for row in reader:
            # extract the data from each row
            point = [float(row[0]), float(row[1]), float(row[2])]
            # append the data to the appropriate lists
            radiations.append(point)

    return radiations
