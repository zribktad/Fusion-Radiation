
import re
import numpy as np
import matplotlib.pyplot as plt
import mplcursors

from read_csv import read_estimations, read_radiations
from plot_graphs import plot_distances_graph, plot_points_graph
from file_pair import find_file_pairs



def get_min_distances(estimations, radiations):
    list_min_distances = []
    for source in radiations:
        min_distances = []
        for points in estimations:
            distances = []
            if points:
                for point in points:
                    distance = np.linalg.norm(np.array(point) - np.array(source))
                    distances.append(distance)
                min_dist = (min(distances))#+abs(len(points)-len(radiations)))
            # print("Minimum distance for ", rad , "  : " , min_dist)
            else:
                min_dist = None
            min_distances.append(min_dist)
        list_min_distances.append(min_distances)
    return list_min_distances

def get_average_est_points(estimations):
    non_empty_lists = [points for points in estimations if len(points) != 0]
    if len(non_empty_lists) == 0:
        return 0
    sum = 0
    count = 0
    for points in non_empty_lists:
        sum += len(points) 
        count += 1
    return round((sum / count),1)


def make_bar_graph(avg_distances, med_distances, pair_labels, pair_labels_row):
    x_pos = np.arange(len(pair_labels))

        # Set the width and colors for the bars
    bar_width = 0.35
    colors = ['b','b','b','g','g','g','r','r','r','c','c','c','m','m','m','k','k','k','y','y','y' ]#['b', 'g', 'r', 'c', 'm', 'y', 'k']

        # fig, ax = plt.subplots()
        # Create the bar chart for the average distances
    bar1 = plt.bar(x_pos, avg_distances, width=bar_width, align='center', color=colors[:len(pair_labels)],
                    label='Average')

        # Create the bar chart for the median distances
    bar2 = plt.bar(x_pos + bar_width, med_distances, width=bar_width, align='center', color=colors[:len(pair_labels)],
                    label='Median', alpha=0.5)

        # Add the pair labels to the x-axis
    plt.xticks(x_pos + bar_width / 2, pair_labels_row, rotation=90)

        # Add axis labels and a title
    plt.xlabel('File Pair')
    plt.ylabel('Distance Value')
    plt.title('Head Chart')

        # Add a legend
    plt.legend()

        # Add a tooltip to each bar on hover
    tooltips = mplcursors.cursor([bar1, bar2]).connect('add', lambda sel: sel.annotation.set_text(
            pair_labels[sel.target.index]))

        # Display the chart
    plt.show()

def calc_bar_graph(get_average_number_estimations, avg_distances, med_distances, pair_labels, pair_labels_row, estimations, head, radiations, label, list_min_distances):
    if len(list_min_distances) == 0:
        print("list_min_distances is empty")
    else:
        print("array min ",list_min_distances)
        for idx,source in enumerate(radiations):
            list_min_distances[idx] = list_min_distances[idx][ 60:]
            list_min_distances[idx] = [x for x in list_min_distances[idx] if x is not None]
        one_d_distances=np.sum(list_min_distances, axis=0)/len(radiations)

            #one_d_distances = np.array(list_min_distances).flatten().tolist()
        one_d_distances = [x for x in one_d_distances if x is not None]
        print("array ",one_d_distances)
        avg = np.mean(one_d_distances)
        med = np.median(one_d_distances)
        avg_distances.append(avg)
        med_distances.append(med)

        avg_est=get_average_number_estimations(estimations)

        pair_labels_row.append("s:" + str(len(one_d_distances)) +" e:"+ str(avg_est))
        label_edit2 = re.sub('[^0-9a-zA-Z]+', ' ', label)

        head_edit = ''
        for i in range(1, len(head) - 1, 2):
            head_edit += "\n"+str(head[i+1]) + ": " + str(head[i + 2]) 
      
          #  tmp_label = '\n'.join(re.findall('.{1,20}', label_edit) + " \n " + head)
        tmp_label =  label_edit2+head_edit
           
        pair_labels.append(tmp_label)

if __name__ == "__main__":

    path = "/home/tadeas/my_workspace/src/fusion_radiation/data/Final/All/"
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
        print("Head of files",head)
        radiations = read_radiations(filename_rad)
        label = pair[0][8:]
        label_edit = re.sub('[^0-9a-zA-Z]+', ' ', label)
        list_min_distances = get_min_distances(estimations, radiations)

        calc_bar_graph(get_average_est_points, avg_distances, med_distances, pair_labels, pair_labels_row, estimations, head, radiations, label, list_min_distances)
    try:
        make_bar_graph(avg_distances, med_distances, pair_labels, pair_labels_row)
    except IndexError:
        print("Empty file sources")

    if False:
        plot_distances_graph(timestamps, list_min_distances, radiations, label_edit)

            # plot the points graph
        plot_points_graph(timestamps, estimations, label_edit)
