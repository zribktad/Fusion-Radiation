import numpy as np
from numpy import savetxt
from numpy import asarray
import matplotlib.pyplot as plt
import rosbag
from file_pair import find_file_pairs
from read_csv import *
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import os


label_size = 24
tick_size = 14


def plot_heatmap(x, y, sources, x_start=-5, x_end=20, y_start=-5, y_end=10, bins=50, cmap='jet', interpolation='gaussian', fig_size=(8, 6)):
    # Create a 2D histogram of the data within the specified range
    heatmap, xedges, yedges = np.histogram2d(
        x, y, bins=bins, range=[[x_start, x_end], [y_start, y_end]])

    # Set the figure size
    plt.figure(figsize=fig_size)

    # Plot the heatmap
    plt.imshow(heatmap.T, origin='lower', extent=[
               x_start, x_end, y_start, y_end], cmap=cmap, interpolation=interpolation)
    cbar = plt.colorbar()
    cbar.set_label("Estimation", fontsize=label_size)
    cbar.ax.tick_params(labelsize=tick_size)
    plt.xlabel('X', fontsize=label_size)
    plt.ylabel('Y', fontsize=label_size)

    # Add the sources to the plot
    for x, y in sources:
        plt.scatter(x, y, color='white', s=200, marker="x")

    plt.xticks(fontsize=tick_size)
    plt.yticks(fontsize=tick_size)

    plt.show()


# Generate some example data


# x = np.random.uniform(-5, 5, size=10000)
# y = np.random.uniform(-5, 5, size=10000)
# plt.scatter(0, 1, color='black', s=40)
# Plot the heatmap
# plot_heatmap(x, y, x_start=-6, x_end=6, y_start=-6, y_end=6, bins=20, cmap='plasma', interpolation='nearest')
csv = True

if csv is False:
    bagfilename = '/home/tadeas/my_workspace/bagfiles/experimenty_temesvar/29_2023_04_22_18_22_15/_2023-04-22-18-23-47.bag'
    # one working
    #  bagfilename = '/home/tadeas/my_workspace/bagfiles/experimenty_temesvar_nedele/35_2023_04_23_10_44_13_one_source_rtk/_2023-04-23-10-46-20.bag'
    if os.path.exists(bagfilename+".data.npy"):
         [x, y, z] = np.load(bagfilename+".data.npy")
    else:
        bag = rosbag.Bag(bagfilename)
        # bag = rosbag.Bag('/home/tadeas/my_workspace/bagfiles/experimenty_temesvar/29_2023_04_22_18_22_15/_2023-04-22-18-23-47.bag')
        # bag = rosbag.Bag('/home/tadeas/my_workspace/bagfiles/experimenty_temesvar/31_2023_04_22_18_43_25/_2023-04-22-18-43-56.bag')
            # Initialize empty arrays for x and y coordinates
        x = []
        y = []
        z = []

            # Iterate through each message in the specified topic
        for _, msg, _ in bag.read_messages(topics=['/uav35/fusion_radiation/estimation']):
                # Convert the PointCloud2 message to a numpy array

            if msg is not None:
                pointcloud = np.array(list(pc2.read_points(msg)))

                    # Extract the x and y coordinates
                if np.size(pointcloud) > 0:
                    x_coords = pointcloud[:, 0]
                    y_coords = pointcloud[:, 1]
                    z_coords = pointcloud[:, 2]

                    # Append the coordinates to the arrays
                    x.extend(x_coords)
                    y.extend(y_coords)
                    z.extend(z_coords)

        np.save(bagfilename+".data", [x, y, z])

    x_start, x_end = min(x), max(x)
    y_start, y_end = min(y), max(y)
        # Plot the heatmap
    sources = [[-0.7, 3.3], [15.6, 3.2]]
    # sources = [[8,14.3],[15.6,3.2],[-0.4,4]]
    # sources = [[3.14,10.9]] #nedela prvz
    # sources = [[3.14,10.9],[9.4,1.5],[-8.8,14.6]]
    plot_heatmap(x, y, sources, x_start=-5, x_end=20, y_start=-10,
                 y_end=15, bins=30, cmap='viridis', interpolation='hanning')
else:
    path = "/home/tadeas/my_workspace/src/fusion_radiation/data/"
    file_pairs = find_file_pairs(path)
    # For avg graph...
    avg_distances = []
    med_distances = []
    pair_labels = []
    pair_labels_row = []

    for pair in file_pairs:
        print("Process file: ", pair[0], " & ", pair[1])
        filename_rad = path + str(pair[0])
        filename = path + str(pair[1])
        timestamps, estimations, head = read_estimations(filename)
        print("Head of files", head)
        radiations = read_radiations(filename_rad)
        sources = []
        x = []
        y = []

        for sub_array in estimations:
            for sub_sub_array in sub_array:
                if sub_sub_array is None or len(sub_sub_array) < 2:
                    continue  # skip empty or None sub-arrays
                x.append(sub_sub_array[0])
                y.append(sub_sub_array[1])

        for sub_array in radiations:
            new_sub_array = sub_array[:2]  # extract first two elements
            sources.append(new_sub_array)
        souces_tmp = np.array(sources)

        x_coords = souces_tmp[:, 0]
        y_coords = souces_tmp[:, 1]

        # Find the maximum and minimum values of x and y
        x_max, x_min = np.max(x_coords), np.min(x_coords)
        y_max, y_min = np.max(y_coords), np.min(y_coords)
        mi_dist = max(x_max-x_min/2 ,y_max-y_min/2)
        x_mean = np.mean(x_coords)
        y_mean = np.mean(y_coords)

      

        
        # Plot the heatmap
        plot_heatmap(x, y, sources,x_start=x_mean-mi_dist, x_end=x_mean+mi_dist, y_start=y_mean-mi_dist, y_end=y_mean+mi_dist, bins=15, cmap='plasma', interpolation='hanning')

