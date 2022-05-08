import numpy as np
import matplotlib.pyplot as plt
import networkx as nx

def plot_priority_mat_as_directed_graph(priority_fname):
     l=[]
     if isinstance(priority_fname,str):
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

     # Z = [[0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0],
     #      [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     #      [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     #      [1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     #      [1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     #      [1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0],
     #      [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
     #      [0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0],
     #      [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
     #      [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1],
     #      [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1],
     #      [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0],
     #      [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0],
     #      [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0]]

     G = nx.from_numpy_matrix(l, create_using=nx.MultiDiGraph())
     pos = nx.circular_layout(G)
     nx.draw_circular(G, arrowstyle='->')
     labels = {i : i for i in G.nodes()}  # {0, 1, ..., k-1}
     nx.draw_networkx_labels(G, pos, labels, font_size=15)
     plt.show()

priority_fname = 'files/10obs-20x20map-50agents/10obs-20x20map-50agents-t30.priorities'
plot_priority_mat_as_directed_graph(priority_fname)