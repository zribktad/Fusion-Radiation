import matplotlib.pyplot as plt
import numpy as np

uniform_rs_size = 110.876
random_rs_size = 275.654
lines_rs_size = 856.621

uniform_time = 107.526
random_time = 347.824
lines_time = 298.517

dpi = 100
tick_size = 20
label_size = 38
line_width = 3
title_size = 1

bar_width = 0.35
    #colors = ['b','b','b','g','g','g','r','r','r','c','c','c','m','m','m','k','k','k','y','y','y' ]#['b', 'g', 'r', 'c', 'm', 'y', 'k']
colors = ['b', 'g']

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Palatino']
plt.rcParams['font.size'] = 12

rs_sizes = [uniform_rs_size,  lines_rs_size]
times = [uniform_time, lines_time]

# create bar chart
labels = ['\nUniform',  '\nLine']
x = np.arange(len(labels))



fig = plt.figure(figsize=(5, 3))
ax = fig.add_subplot(111)

# plot RS size and times on the same bar chart
#ax.set_xlabel('Data Type',fontsize = label_size)
ax.set_ylabel('Num. of sampled points',fontsize = label_size)

bar_width = 0.35
opacity = 0.8

rs_bar = ax.bar(x, rs_sizes, bar_width, alpha=opacity, color='blue', label='num. of sampled points')

plt.xticks(x + bar_width / 2, labels, rotation=0)

#ax.set_xticks(x)
#ax.set_xticklabels(labels)

# add secondary y-axis for times
ax2 = ax.twinx()
ax2.set_ylabel('Time complexity ( μs )',fontsize = label_size)
ax2.tick_params(axis='y',  labelsize=tick_size)
ax.tick_params( labelsize=tick_size)
ax.tick_params(axis='x', labelsize=label_size)


time_bar = ax2.bar(x + bar_width, times, bar_width, alpha=opacity, color='orange')
ax2.set_ylim(0,500)
yticks = np.arange(0,451,50)
ax2.set_yticks(yticks)

ax.set_ylim(0,1200)
yticks2 = np.arange(0,1101,100)
ax.set_yticks(yticks2)
#plt.xlabel('Sampling algorithm', fontsize = label_size)

ax.legend((rs_bar, time_bar), ('Points', 'Comlexity'),fontsize=tick_size)


plt.show()



uniform_rs_size = 162.497
random_rs_size = 372.084
lines_rs_size = 1082.24

uniform_time = 132.892
random_time = 377.49
lines_time = 362.09

dpi = 100
tick_size = 20
label_size = 38
line_width = 3
title_size = 1

bar_width = 0.35
    #colors = ['b','b','b','g','g','g','r','r','r','c','c','c','m','m','m','k','k','k','y','y','y' ]#['b', 'g', 'r', 'c', 'm', 'y', 'k']
colors = ['b', 'g']

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Palatino']
plt.rcParams['font.size'] = 12

rs_sizes = [uniform_rs_size,  lines_rs_size]
times = [uniform_time, lines_time]

# create bar chart
labels = ['\nUniform',  '\nLine']
x = np.arange(len(labels))



fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111)


# plot RS size and times on the same bar chart
#ax.set_xlabel('Data Type',fontsize = label_size)
ax.set_ylabel('Num. of sampled points',fontsize = label_size)

bar_width = 0.35
opacity = 0.8

rs_bar = ax.bar(x, rs_sizes, bar_width, alpha=opacity, color='blue', label='num. of sampled points')

plt.xticks(x + bar_width / 2, labels, rotation=0)

#ax.set_xticks(x)
#ax.set_xticklabels(labels)

# add secondary y-axis for times
ax2 = ax.twinx()
ax2.set_ylabel('Time complexity ( μs )',fontsize = label_size)
ax2.tick_params(axis='y',  labelsize=tick_size)
ax.tick_params( labelsize=tick_size)
ax.tick_params(axis='x', labelsize=label_size)


time_bar = ax2.bar(x + bar_width, times, bar_width, alpha=opacity, color='orange')
ax2.set_ylim(0,500)
ax2.set_yticks(yticks)

ax.set_ylim(0,1200)

ax.set_yticks(yticks2)
#plt.xlabel('Sampling algorithm', fontsize = label_size)

ax.legend((rs_bar, time_bar), ('Points', 'Comlexity'),fontsize=tick_size)


plt.show()
