import os

import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import tkinter.filedialog as filedialog
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from pathlib import Path


def read_agents_fname(agents_fname):
    start_and_goal_can_be_at_same_loc = True
    delta = 0
    if start_and_goal_can_be_at_same_loc:
        delta = 0.25
    with open(agents_fname, 'r') as f:
        l = [[num for num in line.split(',')] for line in f]
    k = int(l.pop(0)[0])  # pop first row, this is the agents number
    agent_start_dict = {}  # {0: (Sx_0, Sy_0), 1: (Sx_1, Sy_1), ... }
    agent_goal_dict = {}   # {0: (Gx_0, Gy_0), 1: (Gx_1, Gy_1), ... }
    agent_start_dict_4plot = {}
    agent_goal_dict_4plot = {}
    for idx, line in enumerate(l):
        # PAY ATTENTION: indexing reversed (i, j) / (x, y) issue
        # print(idx, line)
        agent_start_dict[idx] = (int(line[1]), int(line[0]))
        agent_goal_dict[idx] = (int(line[3]), int(line[2]))
        agent_start_dict_4plot[idx] = (int(line[1])-delta, int(line[0])-delta)
        agent_goal_dict_4plot[idx] = (int(line[3])+delta, int(line[2])+delta)
        # data_file.write(line [0] +", "+ line[1] +", "+ line [2] +", "+ line[3] + ", ")

    # Test dictionary size:
    if (len(agent_goal_dict) != k or len(agent_start_dict) != k):
        raise ValueError('dictionary size doesnt equal to the number of agents')

    return k, agent_start_dict, agent_goal_dict, agent_start_dict_4plot, agent_goal_dict_4plot

"""
def priority_edges_from_file(priority_fname):
    l = []
    if isinstance(priority_fname, str):
        with open(priority_fname, 'r') as f:
            for line in f:
                chr_line=[]
                for char in list(line):
                    if char=='\n':
                        continue
                    chr_line.append(int(char=='@'))
                l.append(chr_line)
        l = np.array(l)
    else:
        l = priority_fname

    # P = nx.from_numpy_matrix(l, create_using=nx.MultiDiGraph())
    return l  # P
"""


def read_map(map_fname):
    map = []
    with open(map_fname, 'r') as f:
        for line in f:
            chr_line = []
            for char in list(line):
                if char == '\n' or char == '\r':
                    continue
                chr_line.append(int(char == '@' or char == '1' or char == 'T'))
            map.append(chr_line)
    map.pop(0)  # pop first row, this is the agents number
    map = np.array(map)
    return map


class QueryDisplayer:
    def __init__(self,  agents_fname, map_fname):

        self.agents_fname = agents_fname
        self.map_fname = map_fname
        self.priority_fname = agents_fname[:-6] + 'priorities'

    def display(self):
        self.map = read_map( self.map_fname)
        row = len(self.map) - 2     # minus padding
        col = len(self.map[0]) - 2  # minus padding

        k, self.agent_start_dict, self.agent_goal_dict, self.agent_start_dict_4plot, self.agent_goal_dict_4plot = \
            read_agents_fname( self.agents_fname)

        # plotting:
        self.plot_visualization()

    def plot_visualization(self):
        # Starts:
        self.S = nx.Graph()
        self.S.add_nodes_from(self.agent_start_dict.keys())

        # Goals:
        self.G = nx.Graph()
        self.G.add_nodes_from(self.agent_goal_dict.keys())

        self.labels = {i: i for i in self.S.nodes()}

        # plotting
        fig, ax = plt.subplots(figsize=(12, 12))
        S_nodes = nx.draw_networkx_nodes(self.S, pos=self.agent_start_dict_4plot, label=self.labels, node_color='lightblue', edgecolors='darkblue', node_size=210)
        nx.draw_networkx_labels(self.S, pos=self.agent_start_dict_4plot, labels=self.labels, font_size=8, verticalalignment='center', horizontalalignment='center')
        G_nodes = nx.draw_networkx_nodes(self.G, pos=self.agent_goal_dict_4plot, label=self.labels, node_color='lightgreen',edgecolors='darkgreen', node_size=200, node_shape='s')
        nx.draw_networkx_labels(self.G, pos=self.agent_goal_dict_4plot, labels=self.labels, font_size=8, verticalalignment='center', horizontalalignment='center')

        # Priorities:
        # PLOT_PROPRITY = False
        # if path.exists(self.folder_name + self.priority_fname) and PLOT_PROPRITY:
        #     self.PBS_priorities_matrix = priority_edges_from_file(priority_fname=self.folder_name + self.priority_fname)
        #     self.P = nx.from_numpy_matrix(self.PBS_priorities_matrix, create_using=nx.MultiDiGraph())
        #     S_edges = nx.draw_networkx_edges(self.P, pos=self.agent_start_dict_4plot, arrowstyle='->', node_size=350, edge_color='red')
        G_nodes.set_zorder(1)  # make edges above node
        S_nodes.set_zorder(1)  # make edges above node
        fig.tight_layout()

        self.plot_map(ax)
        plt.title(f'{self.agents_fname}')
        plt.show()

    def get_figure(self):

        # Starts:
        self.S = nx.Graph()
        self.S.add_nodes_from(self.agent_start_dict.keys())

        # Goals:
        self.G = nx.Graph()
        self.G.add_nodes_from(self.agent_goal_dict.keys())

        self.labels = {i: i for i in self.S.nodes()}

        # plotting
        fig, ax = plt.subplots(figsize=(12, 12))
        S_nodes = nx.draw_networkx_nodes(self.S, pos=self.agent_start_dict_4plot, label=self.labels, node_color='lightblue', edgecolors='darkblue', node_size=210)
        nx.draw_networkx_labels(self.S, pos=self.agent_start_dict_4plot, labels=self.labels, font_size=8, verticalalignment='center', horizontalalignment='center')
        G_nodes = nx.draw_networkx_nodes(self.G, pos=self.agent_goal_dict_4plot, label=self.labels, node_color='lightgreen',edgecolors='darkgreen', node_size=200, node_shape='s')
        nx.draw_networkx_labels(self.G, pos=self.agent_goal_dict_4plot, labels=self.labels, font_size=8, verticalalignment='center', horizontalalignment='center')

        # Priorities:
        # PLOT_PROPRITY = False
        # if path.exists(self.folder_name + self.priority_fname) and PLOT_PROPRITY:
        #     self.PBS_priorities_matrix = priority_edges_from_file(priority_fname=self.folder_name + self.priority_fname)
        #     self.P = nx.from_numpy_matrix(self.PBS_priorities_matrix, create_using=nx.MultiDiGraph())
        #     S_edges = nx.draw_networkx_edges(self.P, pos=self.agent_start_dict_4plot, arrowstyle='->', node_size=350, edge_color='red')
        G_nodes.set_zorder(1)  # make edges above node
        S_nodes.set_zorder(1)  # make edges above node
        fig.tight_layout()

        self.plot_map(ax)
        plt.title(f'{self.agents_fname}')
        return plt

    def plot_map(self, ax):
        row = len(self.map)
        col = len(self.map[0])
        ax.imshow(self.map, cmap='Greys', interpolation='None', origin='lower')
        ax.grid(True, which='major', axis='both', linestyle='-', color='grey')
        ax.set_xticks(np.arange(-0.5, col+0.5, 1))
        ax.set_yticks(np.arange(-0.5, row+0.5, 1))
        plt.setp(ax.get_xticklabels(), visible=False)
        plt.setp(ax.get_yticklabels(), visible=False)
        ax.tick_params(axis='both', which='both', length=0)


def input_map():
    map_path = tk.filedialog.askopenfilename(initialdir=Path.cwd(), filetypes=(("map files", "*.map"),))
    print(f'map file selected: {map_path}')
    map_entry.delete(1, tk.END)  # Remove current text in entry
    map_entry.insert(0, map_path)  # Insert the 'path'
    # return map_path


def input_agents():
    agents_path = tk.filedialog.askopenfilename(initialdir=Path.cwd(), filetypes=(("agents files", "*.agents"),))
    print(f'agents file selected: {agents_path}')
    agents_entry.delete(1, tk.END)  # Remove current text in entry
    agents_entry.insert(0, agents_path)  # Insert the 'path'
    # return agents_path


def plot():
    displsyer = QueryDisplayer(agents_fname=agents_entry.get(), map_fname=map_entry.get())
    displsyer.display()




master = tk.Tk()
master.title('Simple MAPF Query Displayer by Nitzan Madar')
# icon_fname = 'icon.ico'
# if os.path.isdir(icon_fname):
#     print('icon file exist')
#     master.iconbitmap(icon_fname)
# else:
#     print('icon file not exist')
# master.iconbitmap(icon_fname)

# canvas1 = tk.Canvas(master, width =500, height=500)
# canvas1.pack()
top_frame = tk.Frame(master)
mid_frame = tk.Frame(master)
bottom_frame = tk.Frame(master)
line = tk.Frame(master, height=1, width=400, bg="grey80", relief='groove')
line2 = tk.Frame(master, height=1, width=400, bg="grey80", relief='groove')

map_path = tk.Label(top_frame, text="Map File Path:")
map_entry = tk.Entry(top_frame, text="", width=50)
browse_map_file = tk.Button(top_frame, text="Browse", command=input_map)

agents_path = tk.Label(mid_frame, text="Agents File Path:")
agents_entry = tk.Entry(mid_frame, text="", width=50)
browse_agents_file = tk.Button(mid_frame, text="Browse", command=input_agents)

display_button = tk.Button(bottom_frame, text='Display!', command=plot)

top_frame.pack(side=tk.TOP)
line.pack(pady=10)
mid_frame.pack()
line2.pack(pady=10)
bottom_frame.pack()

map_path.pack(pady=5)
map_entry.pack(pady=5)
browse_map_file.pack(pady=5)

agents_path.pack(pady=5)
agents_entry.pack(pady=5)
browse_agents_file.pack(pady=5)

display_button.pack(pady=20, fill=tk.X)





master.mainloop()


# top_frame = tk.Frame(master)
# bottom_frame = tk.Frame(master)
# line = tk.Frame(master, height=1, width=400, bg="grey80", relief='groove')
#
# input_path = tk.Label(top_frame, text="Input File Path:")
# input_entry = tk.Entry(top_frame, text="", width=40)
# browse1 = tk.Button(top_frame, text="Browse", command=input)
#
# output_path = tk.Label(bottom_frame, text="Output File Path:")
# output_entry = tk.Entry(bottom_frame, text="", width=40)
# browse2 = tk.Button(bottom_frame, text="Browse", command=output)
#
# begin_button = tk.Button(bottom_frame, text='Begin!')
#
# top_frame.pack(side=tk.TOP)
# line.pack(pady=10)
# bottom_frame.pack(side=tk.BOTTOM)
#
# input_path.pack(pady=5)
# input_entry.pack(pady=5)
# browse1.pack(pady=5)
#
# output_path.pack(pady=5)
# output_entry.pack(pady=5)
# browse2.pack(pady=5)
#
# begin_button.pack(pady=20, fill=tk.X)
#
#
# master.mainloop()