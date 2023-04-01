
import matplotlib.pyplot as plt
from matplotlib import rc
from matplotlib.ticker import ScalarFormatter

dpi = 100
tick_size = 20
label_size = 24
line_width = 3
title_size = 15


def plot_distances_graph(timestamps, min_distances, radiations, title=""):
    # unified plotting settings

    rc('font', **{'family': 'serif', 'serif': ['Palatino']})
    plt.rc('text', usetex=True)
    fig = plt.figure(figsize=(8, 6), dpi=dpi)
    ax = fig.add_subplot(111)

    timestamps = [float(timestamp) - float(min(timestamps))
                  for timestamp in timestamps]


    for idx, source in enumerate(radiations):
        label = f"Source {idx + 1} [{', '.join([f'{coord:.1f}' for coord in source])}]"
        ax.plot(timestamps, min_distances[idx], linewidth=line_width, label=label)

    ax.legend(loc='best', fontsize=tick_size)
    ax.set_xlabel(r"Time [s]", fontsize=label_size)
    ax.set_ylabel(r"Minimum distance [m]", fontsize=label_size)
    ax.tick_params(labelsize=tick_size)
    plt.tick_params(labelsize=tick_size)
    
    ax.grid(which="major")
 # Set y-axis scale to logarithmic
    ax.set_yscale('log')   # set y-axis scale to logarithmic
    ax.set_yticks([ 1.0, 10.0])
    ax.yaxis.set_major_formatter(ScalarFormatter())
    ax.get_yaxis().get_major_formatter().labelOnlyBase = False
   
  
    #ax.ticklabel_format(axis='y', style='sci', scilimits=(0, 0))
    #plt.title(title, fontsize=title_size)
    plt.tight_layout()
    plt.show(block=False)





# Commented out unused code
    # ax.yaxis.set_major_formatter(plt.ScalarFormatter(useMathText=True))
    # set x-axis ticks at every 11th timestamp
    # n_ticks = 11
    # tick_indices = np.linspace(0, len(timestamps) - 1, n_ticks).astype(int)
    # tick_labels = [f"{float(timestamps[i]):.0f}" for i in tick_indices]
    # ax.set_xticks([timestamps[i] for i in tick_indices])
   # ax.set_xticklabels(tick_labels, rotation=45)

   



def plot_points_graph(timestamps, estimations, title=""):

    # use latex text renderer
    # count the number of points for each timestamp
    point_counts = [len(points) for points in estimations]

    # create a list of x-values and y-values for each edge
    x_edges = [float(timestamps[0])]
    y_edges = [0]
    for i in range(len(timestamps)-1):
        x_edges.append(float(timestamps[i]))
        x_edges.append(float(timestamps[i+1]))
        y_edges.append(point_counts[i])
        y_edges.append(point_counts[i])
    x_edges.append(float(timestamps[-1]))
    y_edges.append(point_counts[-1])
    x_edges.append(float(timestamps[-1]))
    y_edges.append(0)

    # use latex text renderer
    rc('font', **{'family': 'serif', 'serif': ['Palatino']})
    plt.rc('text', usetex=True)

    fig = plt.figure(figsize=(8, 6), dpi=dpi)
    ax = fig.add_subplot(111)
    ax.plot(x_edges, y_edges, linewidth=line_width)
    ax.fill_between(x_edges, y_edges, color='lightblue')
    ax.set_xlabel("Time [s]", fontsize=label_size)
    ax.set_ylabel("Estimated number of sources", fontsize=label_size)
    ax.tick_params(labelsize=tick_size)
    plt.tick_params(labelsize=tick_size)
    ax.grid(which='major')
    plt.tight_layout()
    #plt.title(title, fontsize=title_size)
  
    # n_ticks = 11
    # tick_indices = np.linspace(0, len(timestamps) - 1, n_ticks).astype(int)
    # tick_labels = [f"{float(timestamps[i]):.1f}" for i in tick_indices]
    # ax.set_xticks([timestamps[i] for i in tick_indices])
    # ax.set_xticklabels(tick_labels, rotation=45)
    plt.show()
