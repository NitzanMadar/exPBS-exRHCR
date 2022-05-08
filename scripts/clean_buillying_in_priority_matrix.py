import os
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx


def plot_priority_mat_as_directed_graph(priority_fname):
    l=[]
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
    nx.draw_circular(G,  arrowstyle='->', node_color='lightblue')
    labels = {i : i for i in G.nodes()}  # {0, 1, ..., k-1}
    nx.draw_networkx_labels(G, pos, labels, font_size=12, arrowstyle='->')
    # nx.draw_networkx_edges(G, pos)
    # plt.show()



def create_cleaned_priorities_file(priority_fname, to_plot=False):
    new_priority_fname = priority_fname[:-11] + '-cleaned' + priority_fname[-11:]  # add 'cleaned' before
    if to_plot:
        plt.rcParams['figure.figsize'] = (25, 10)
        plt.subplot(1, 2, 1)
        plot_priority_mat_as_directed_graph(priority_fname)
        plt.title('Before Cleaning')
    l=[]
    with open(priority_fname, 'r') as f:

        for line in f:
            chr_line=[]
            for char in list(line):
                if char=='\n':
                    continue
                chr_line.append(int(char=='@'))
            l.append(chr_line)

    priority_matrix = np.array(l)
    total_constraints_in_mat = np.sum(priority_matrix)
    precent = 0.2
    to_reduce = int(precent*total_constraints_in_mat)  # all the element that bigger or equal than the one in this location

    B = np.sum(priority_matrix, axis=1).reshape(-1, 1)  # [k, 1], Bullying measure
    N = np.sum(priority_matrix, axis=0).reshape(1, -1)  # [1, k], Nerds measure
    BN = np.dot(B, N)  # [k, k], bully-nerds relationship measure
    BN[priority_matrix==0] = 0  # zeros irrelevant elements (that zeros in priority_matrix)

    i_threshold, j_threshold = np.unravel_index(np.argsort(BN.ravel())[-to_reduce], BN.shape)

    # mask = np.ones_like(BN)  # elemets to keep is "1"
    # mask[BN >= BN[i_threshold, j_threshold]] = 0  # remove everyone that bigger than thee threshold
    priority_matrix[BN >= BN[i_threshold, j_threshold]] = 0  # remove everyone that bigger than thee threshold

    # output = np.multiply(mask, priority_matrix)
    # output = np.zeros
    # output = priority_matrix[mask==1]
    if to_plot:
        plt.subplot(1, 2, 2)
        plot_priority_mat_as_directed_graph(priority_matrix)
        plt.title('After Cleaning')
        plt.show()

    print(total_constraints_in_mat, "-", to_reduce, "~=", np.sum(priority_matrix))


    # write new file
    data_file = open(new_priority_fname, "w+")
    for i in priority_matrix:
        for j in i:
            if j==1:
                char = '@'
            elif j==0:
                char = '.'
            else:
                raise ValueError('Element in priority matrix is not 1 ot 0')
            data_file.write(char)
        data_file.write("\n")

    # print('Created file: {0:.2f}'.format(new_priority_fname))
    print('Created file: ' + new_priority_fname)

if __name__ == '__main__':
    # Example:
    # priority_fname = 'files/10obs-20x20map-100agents/10obs-20x20map-100agents-0.priorities'
    # priority_fname = 'files/10obs-20x20map-50agents/10obs-20x20map-50agents-t30.priorities'
    # create_cleaned_priorities_file(priority_fname, to_plot=True)

    folder_name = './files/10obs-20x20map-100agents/'
    counter = 0
    for filename in os.listdir(folder_name):
    # print(filename)
        if filename.endswith('priorities'):
            counter += 1
            print(filename)
            create_cleaned_priorities_file(folder_name + filename, to_plot=False)
    print('Total Created Files: '+ str(counter))