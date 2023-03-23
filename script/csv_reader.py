import csv
import numpy as np
from pprint import pprint
import matplotlib.pyplot as plt
from matplotlib import rc

filename = "test.csv"  # replace with the actual filename
filename_rad = "test_rad.csv"

# create empty lists to store the data
timestamps = []
radiations = []
estimations = []
points_list =[]

#* read the CSV file for estimation
with open( filename, "r") as f:
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

#print("time + points ")
#pprint(points_list)


#*read the CSV file for real sources
with open( filename, "r") as f:
    reader = csv.reader(f, delimiter=",")
    next(reader)  # skip the header row
    for row in reader:
        # extract the data from each row
        point = [float(row[0]), float(row[1]), float(row[2])] 
        # append the data to the appropriate lists
        radiations.append(point)

#******* graph **************

# unified plotting settings
dpi = 170
suptitle_size = 18
tick_size = 20
label_size = 24
line_width =3

# use latex text renderer
rc('font', **{'family': 'serif', 'serif': ['Palatino']})
plt.rc('text', usetex=True)


fig = plt.figure(figsize=(8, 6), dpi=dpi)
ax = fig.add_subplot(111)

for rad in radiations:
    min_distances = []
    for timestamp, points in points_list:
        distances = []
        if points:
            for point in points:
                distance = np.linalg.norm(np.array(point) - np.array(rad))
                distances.append(distance)
            min_dist = min(distances)
            print("Minimum distance for ", rad , "in time: ", timestamp," :", min_dist)
        else:
            min_dist = None
        min_distances.append(min_dist)
    ax.plot(timestamps, min_distances, linewidth=line_width,label=f"Radiation source {rad}")




#ax.set_xlim(min_time, max_time)
# x_ticks = np.arange(timestamps[0], timestamps[-1]+1, 1)
# plt.xticks(x_ticks)

# ax.set_xticks(np.linspace(0, len(timestamps), 6))
ax.set_xlabel(r'Time [s]', fontsize=label_size)
ax.set_ylabel(r'Minimum distance [m]', fontsize=label_size)
ax.tick_params(labelsize=tick_size)
plt.tick_params(labelsize=tick_size)
ax.grid(which='major')
#ax.set_aspect('equal')
#plt.legend(['Radiation 1', 'Radiation 2'], loc='upper right')
plt.tight_layout()
plt.show(block=True)


point_counts = [len(points) for timestamp, points in points_list]


# create a list of x-values and y-values for each edge
x_edges = []
y_edges = []
for i in range(len(timestamps)-1):
    x_edges.append(timestamps[i])
    x_edges.append(timestamps[i+1])
    y_edges.append(point_counts[i])
    y_edges.append(point_counts[i])
x_edges.append(timestamps[-1])
y_edges.append(point_counts[-1])

# plot the bar graph

fig = plt.figure(figsize=(8, 6), dpi=dpi)
ax = fig.add_subplot(111)
#ax.bar(point_count.keys(), point_count.values(), width=5,align='center',)
ax.plot(x_edges,y_edges,linewidth=line_width)
ax.fill_between(x_edges, y_edges, color='lightblue')
ax.set_xlabel("Time [s]", fontsize=label_size)
ax.set_ylabel("Number of points", fontsize=label_size)
ax.grid(which='major')
plt.tight_layout()

plt.show(block=True)
