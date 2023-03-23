import csv

from read_csv import read_estimations, read_radiations
from plot_graphs import plot_distances_graph, plot_points_graph



if __name__ == "__main__":
    filename = "/home/tadeas/my_workspace/src/fusion_radiation/script/test.csv"  # replace with the actual filename
    filename_rad = "/home/tadeas/my_workspace/src/fusion_radiation/script/test_rad.csv"
    print(__file__)
    # read the CSV file for estimation
    timestamps, estimations = read_estimations(filename)

    # read the CSV file for real sources
    radiations = read_radiations(filename_rad)
  
    # plot the distances graph
    plot_distances_graph(timestamps, estimations, radiations)

    # plot the points graph
    plot_points_graph(timestamps, estimations)
