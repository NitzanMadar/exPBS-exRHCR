import os.path
from os import path
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import subprocess
from tqdm import tqdm

###################### Functions: ############################
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


def EuclideanDist(coordinate1, coordinate2):
    return ((coordinate1[0]-coordinate2[0])**2 + (coordinate1[1]-coordinate2[1])**2)**0.5


def ccw(A, B, C):
    Ax,Ay = A
    Bx,By = B
    Cx,Cy = C
    return (Cy-Ay)*(Bx-Ax) > (By-Ay)*(Cx-Ax)


def is_parallel(A, B, C, D):
    Ax,Ay = A
    Bx,By = B
    Cx,Cy = C
    Dx,Dy = D
    if (Ax-Bx)*(Cy-Dy) == (Ay-By)*(Cx-Dx):  # is parallel
        return True


def intersect(A, B, C, D):

    if is_parallel(A, B, C, D):
        return False  # no intersection for parallel lines

    # two segments intersection if... https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)


def two_line_segments_intersection(A, B, C, D):
    # Find intersection of 2 line segments.
    # Line 1 defined by (x1,y1) and (x2,y2)
    # Line 2 defined by (x3,y3) and (x4,y4)
    x1, y1 = A
    x2, y2 = B
    x3, y3 = C
    x4, y4 = D

    # Convert to float (needed for the intersection point calculation)
    x1, y1, x2, y2, x3, y3, x4, y4 = float(x1), float(y1), float(x2), float(y2), float(x3), float(y3), float(x4), float(y4)

    # Check if the two line segments has intersection (cover also parallel case)
    if not intersect((x1, y1), (x2, y2), (x3, y3), (x4, y4)):
        return None

    denominator = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
    Px = ((x1*y2-y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4)) / denominator
    Py = ((x1*y2-y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-y3*x4)) / denominator

    return Px, Py


def read_map(map_fname):
    map = []
    with open(map_fname, 'r') as f:
        for line in f:
            chr_line=[]
            for char in list(line):
                if char=='\n' or char=='\r':
                    continue
                chr_line.append(int(char=='@'))
            map.append(chr_line)
    map.pop(0)  # pop first row, this is the agents number
    map = np.array(map)
    return map


def read_agents_fname(agents_fname):
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
        agent_start_dict_4plot[idx] = (int(line[1])-0.25, int(line[0])-0.25)
        agent_goal_dict_4plot[idx] = (int(line[3])+0.25, int(line[2])+0.25)
        # data_file.write(line [0] +", "+ line[1] +", "+ line [2] +", "+ line[3] + ", ")

    # Test dictionary size:
    if (len(agent_goal_dict) != k or len(agent_start_dict) != k):
        raise ValueError('dictionary size doesnt equal to the number of agents')

    return k, agent_start_dict, agent_goal_dict, agent_start_dict_4plot, agent_goal_dict_4plot

######################################################################

############################ main ###################################
class ArtificialExperienceBuilder():
    def __init__(self, folder_name, agents_fname, map_fname):
        self.folder_name = folder_name
        self.agents_fname = agents_fname
        self.map_fname = map_fname
        self.priority_fname = agents_fname[:-6] + 'priorities'
        # selfpriority_fname = agents_fname[:-9] + '2236.priorities'  # 2236 is good NN of '100agent-t0'
        self.artificial_experience_fname = folder_name + agents_fname[:-7]+"-artificial.priorities"

    def Build(self, PLOT_S_G_P = False, PLOT_S_DISTANCES = False):
        self.map = read_map(self.folder_name + self.map_fname)
        row = len(self.map) - 2     # minus padding
        col = len(self.map[0]) - 2  # minus padding

        k, self.agent_start_dict, self.agent_goal_dict, self.agent_start_dict_4plot, self.agent_goal_dict_4plot =\
            read_agents_fname(self.folder_name + self.agents_fname)

        Gdistances = np.zeros((k, k))  # between agents, lower triangle only
        Sdistances = np.zeros((k, k))  # between agents, lower triangle only
        SGdistances = np.zeros((k))  # for each agent S to G distance, vector
        similar_SGdistances = np.zeros((k, k))
        intersections = np.zeros((k, k))
        dist_tolerance = 2

        similar_SGdistances_thresh = 10
        for i in range(len(self.agent_start_dict)):
            SGdistances[i] = EuclideanDist(self.agent_start_dict[i], self.agent_goal_dict[i])

        for i in range(len(self.agent_start_dict)):
            for j in range(len(self.agent_start_dict)):
                if i >= j:
                    continue

                P = two_line_segments_intersection(self.agent_start_dict[i], self.agent_goal_dict[i],
                                                   self.agent_start_dict[j], self.agent_goal_dict[j])
                ij_avg_dist_to_intersect = 0.0
                # if 2 line segments intersect and the intersection is at equal distance from both S_i and S_j than add edge
                if P is not None and \
                        abs(EuclideanDist(P, self.agent_start_dict[i]) - EuclideanDist(P, self.agent_start_dict[j])) < dist_tolerance:
                    # intersection founded and S to the intersection distance is small (<tolerance)
                    # print ('intersection:',P)
                    # print ('agent', i, agent_start_dict[i], agent_goal_dict[i])
                    # print ('agent', j, agent_start_dict[j], agent_goal_dict[j])
                    ij_avg_dist_to_intersect = 0.5*(EuclideanDist(P, self.agent_start_dict[i]) + EuclideanDist(P, self.agent_start_dict[j]))
                if SGdistances[j] >= SGdistances[i]:
                    intersections[i, j] = ij_avg_dist_to_intersect  # longer path may have more constraints later, contraints on shorter
                    Sdistances[i, j] = EuclideanDist(self.agent_start_dict[i], self.agent_start_dict[j])
                    Gdistances[i, j] = EuclideanDist(self.agent_goal_dict[i], self.agent_goal_dict[j])
                else:
                    intersections[j, i] = ij_avg_dist_to_intersect
                    Sdistances[j, i] = EuclideanDist(self.agent_start_dict[i], self.agent_start_dict[j])
                    Gdistances[j, i] = EuclideanDist(self.agent_goal_dict[i], self.agent_goal_dict[j])

                if abs(SGdistances[i] - SGdistances[j]) < similar_SGdistances_thresh:
                    similar_SGdistances[i, j] = 1

        # print ("artificial experience constraints: " + str(int(np.sum(intersections))))

        # clean distances:
        matrix_avg_dist_S_to_intersect = 0.5*(np.max(intersections)+ np.min(intersections[np.nonzero(intersections)]))
        upper_thresh = 22
        S_lower_thresh = 3
        G_lower_thresh = 3
        artifical_priorities_matrix = np.where((Sdistances < S_lower_thresh), 1, 0)
        self.artifical_priorities_matrix = np.where((Gdistances < G_lower_thresh) & (similar_SGdistances > 0), 1, 0)

        # === Intersection choose: ===
        # # option 1: intersect and equal S to intersection distance
        # self.artifical_priorities_matrix = np.where((intersections > 0), 1, 0)  # change to boolean

        # option 2: intersect and equal S to intersection distance and smaller the the average distance to intersection
        self.artifical_priorities_matrix = np.where((intersections > 0) & (intersections < 3), 1, 0)  # change to boolean





        # plotting: (if flags are True...)
        self.plot_visualization(PLOT_S_G_P, PLOT_S_DISTANCES)

        # Save priorities file:

        data_file = open(self.artificial_experience_fname, "w+")
        for row in self.artifical_priorities_matrix:
            for char in row:
                if char == 1:
                    data_file.write("@")
                else:
                    data_file.write(".")
            data_file.write("\n")

        print("Generated file " + str(self.artificial_experience_fname))

        data_file.close()

    def plot_visualization(self, PLOT_S_G_P, PLOT_S_DISTANCES):
        # Starts:
        self.S = nx.Graph()
        self.S.add_nodes_from(self.agent_start_dict.keys())

        # Goals:
        self.G = nx.Graph()
        self.G.add_nodes_from(self.agent_goal_dict.keys())

        # Priorities:
        if (path.exists(self.folder_name + self.priority_fname)):
            self.PBS_priorities_matrix = priority_edges_from_file(priority_fname=self.folder_name + self.priority_fname)
            self.P = nx.from_numpy_matrix(self.PBS_priorities_matrix, create_using=nx.MultiDiGraph())
        self.labels = {i: i for i in self.S.nodes()}

        if PLOT_S_G_P:
            # plotting
            fig, ax = plt.subplots(figsize=(12, 12))
            S_nodes = nx.draw_networkx_nodes(self.S, pos=self.agent_start_dict_4plot, label=self.labels, node_color='lightblue', node_size=350)
            nx.draw_networkx_labels(self.S, pos=self.agent_start_dict_4plot, labels=self.labels, font_size=12)
            G_nodes = nx.draw_networkx_nodes(self.G, pos=self.agent_goal_dict_4plot, label=self.labels, node_color='lightgreen', node_size=350, node_shape='s')
            nx.draw_networkx_labels(self.G, pos=self.agent_goal_dict_4plot, labels=self.labels, font_size=12)
            S_edges = nx.draw_networkx_edges(self.P, pos=self.agent_start_dict_4plot, arrowstyle='->', node_size=350, edge_color='red')
            G_nodes.set_zorder(1)  # make edges above node
            S_nodes.set_zorder(1)  # make edges above node
            fig.tight_layout()

            self.plot_map(ax)
            plt.title('Start, Goal and Priorities Relation (Number of Constrints = ' + str(int(np.sum(self.PBS_priorities_matrix))))

        # plot S_i and distances
        if PLOT_S_DISTANCES:
            # plt.figure(2)
            fig, ax = plt.subplots(figsize=(12, 12))
            S_nodes = nx.draw_networkx_nodes(self.S, pos=self.agent_start_dict, label=self.labels, node_color='lightblue', node_size=350)
            nx.draw_networkx_labels(self.S, pos=self.agent_start_dict, labels=self.labels, font_size=12)
            Sdist = nx.from_numpy_matrix(self.artifical_priorities_matrix, create_using=nx.MultiDiGraph())
            nx.draw_networkx_edges(Sdist, self.agent_start_dict, arrowstyle='->', node_size=350, edge_color='red')
            S_nodes.set_zorder(1)
            fig.tight_layout()
            self.plot_map(ax)
            plt.title('Starts + Artificial Priorities (Number of Constraints: ' + str(int(np.sum(self.artifical_priorities_matrix))) + ')')

        plt.show()


    def plot_map(self, ax):
        row = len(self.map)
        col = len(self.map[0])
        ax.imshow(self.map, cmap='Greys', interpolation='None', origin='lower')
        ax.grid(True, which='major', axis='both', linestyle='-', color='grey')
        ax.set_xticks(np.arange(-0.5, row+0.5, 1))
        ax.set_yticks(np.arange(-0.5, col+0.5, 1))
        plt.setp(ax.get_xticklabels(), visible=False)
        plt.setp(ax.get_yticklabels(), visible=False)
        ax.tick_params(axis='both', which='both', length=0)



# Main:
if __name__ == '__main__':
    # Visualize only
    VISUALIZE = False
    if VISUALIZE:
        number_of_agents = '50'
        folder_name = 'files/10obs-20x20map-'+number_of_agents+'agents/'
        map_fname = '10obs-20x20.map'
        agents_fname = "10obs-20x20map-"+number_of_agents+"agents-t" +str(0) + ".agents"
        builder = ArtificialExperienceBuilder(folder_name=folder_name, agents_fname=agents_fname, map_fname=map_fname)
        builder.Build(PLOT_S_G_P=True, PLOT_S_DISTANCES=True)


    RUN_TEST = True
    # create and run
    if RUN_TEST:
        number_of_agents = '100'
        folder_name = 'files/10obs-20x20map-'+number_of_agents+'agents/'
        map_fname = '10obs-20x20.map'
        number_of_queries = 1
        output_fname = 'Output_artificial.csv'

        waitbar = tqdm(total = number_of_queries, position = 0 , leave = True)

        for a in range(0,number_of_queries,1): #agents file number
            print ('\n =====' + str(a) + '/' + str(number_of_queries-1) + '=====')
            agents_fname = "10obs-20x20map-"+number_of_agents+"agents-t" +str(a) + ".agents"

            # Build Artificial experience
            builder = ArtificialExperienceBuilder(folder_name=folder_name, agents_fname=agents_fname, map_fname=map_fname)
            builder.Build()

            # Create CLI strings
            # -m [ --map ] arg                  input file for map
            # -a [ --agents ] arg               input file for agents
            # -o [ --output ] arg               output file for schedule
            # -s [ --solver ] arg               solvers (EPEA, ECBS, ICBS, CBSH, N-ECBS,
            #                                            N-ICBS, N-CBSH, CBSH-CR, CBSH-R, CBSH-RM
            # -k [ --agentNum] arg (default=0): number of agents
            # -p [ --priority ] arg (=2)        priority branching (0 = lazy generating, 1= aggressive generating,
            #                                   2 = aggressive greedy, 3 = fixed priority).
            #                                   This is original PBS fiels, use "2" or remove this field (default = 2)
            # -q [ --priorityMatrix ] arg(=-1)  Choose specific priority matrix, -1 means no choose,
            #                                                                   -2 means use bully-nerd cleaning,
            #                                                                   -3 artificial experience
            # -e [ --experience ] arg (=0)      using experience (=database size, check _.database exist files)
            #                                   or not (=0, for original PBS and artificial)

        # priorityMatrix enforce specific experience matrix or cleaned (use '-2') or artificial (use '-3')

            original_pbs = "./driver --map " + folder_name+map_fname + " --agents " + folder_name+agents_fname+ \
                           " --agentNum " + number_of_agents + " --output " + output_fname
            pbs_with_experience = "./driver --map " + folder_name+map_fname + " --agents " + folder_name+agents_fname+ \
                                  " --agentNum " + number_of_agents + " --output " + output_fname + \
                                  " --experience 5000 --priorityMatrix -1"
            pbs_with_artificial_experience = "./driver --map " + folder_name+map_fname + " --agents " + folder_name+agents_fname+ \
                                             " --agentNum " + number_of_agents + " --output " + output_fname + \
                                             " --priorityMatrix -3"


            status = subprocess.call(original_pbs, shell=True)
            print ('\n ===== Running PBS + Experience ======')
            status = subprocess.call(pbs_with_experience, shell=True)
            print ('\n ===== Running PBS + Artificial Experience =====')
            status = subprocess.call(pbs_with_artificial_experience, shell=True)
            print ('\n\n\n')

            waitbar.update(1)

        waitbar.close()
