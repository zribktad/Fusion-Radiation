import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc

dpi = 170
tick_size = 20
label_size = 24
line_width =3

def plot_distances_graph(timestamps, estimations, radiations):
    # unified plotting settings

    rc('font', **{'family': 'serif', 'serif': ['Palatino']})
    plt.rc('text', usetex=True) 
    fig = plt.figure(figsize=(8, 6), dpi=dpi)
    ax = fig.add_subplot(111)

    for idx, rad in enumerate(radiations):
        min_distances = []
        for points in estimations:
            distances = []
            if points:
                for point in points:
                    distance = np.linalg.norm(np.array(point) - np.array(rad))
                    distances.append(distance)
                min_dist = min(distances)
            else:
                min_dist = None
            min_distances.append(min_dist)
        ax.plot(
        timestamps,
        min_distances,
        linewidth=line_width,
        label=f"Radiation {idx+1}"  # create legend label
    )
    ax.legend( loc='best')
   

    ax.set_xlabel(r"Time [s]", fontsize=label_size)
    ax.set_ylabel(r"Minimum distance [m]", fontsize=label_size)
    ax.tick_params(labelsize=tick_size)
    plt.tick_params(labelsize=tick_size)
    ax.grid(which="major")
    plt.tight_layout()
    #plt.show(block=True)


def plot_points_graph(timestamps, estimations):

    # use latex text renderer
    # count the number of points for each timestamp
    point_counts = [len(points) for points in estimations]

    # create a list of x-values and y-values for each edge
    x_edges = [timestamps[0]]
    y_edges = [0]
    for i in range(len(timestamps)-1):
        x_edges.append(timestamps[i])
        x_edges.append(timestamps[i+1])
        y_edges.append(point_counts[i])
        y_edges.append(point_counts[i])
    x_edges.append(timestamps[-1])
    y_edges.append(point_counts[-1])
    x_edges.append(timestamps[-1])
    y_edges.append(0)


    # use latex text renderer
    rc('font', **{'family': 'serif', 'serif': ['Palatino']})
    plt.rc('text', usetex=True)

    fig = plt.figure(figsize=(8, 6), dpi=dpi)
    ax = fig.add_subplot(111)
    ax.plot(x_edges,y_edges,linewidth=line_width)
    ax.fill_between(x_edges, y_edges, color='lightblue')
    ax.set_xlabel("Time [s]", fontsize=label_size)
    ax.set_ylabel("Number of points", fontsize=label_size)
    ax.tick_params(labelsize=tick_size)
    plt.tick_params(labelsize=tick_size)
    ax.grid(which='major')
    plt.tight_layout()

    plt.show(block=True)