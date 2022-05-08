import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors


def SquaredEuclideanDist(coordinate1, coordinate2):
    return (coordinate1[0] - coordinate2[0]) ** 2 + (coordinate1[1] - coordinate2[1])**2


def EuclideanDist(coordinate1, coordinate2):
    return ((coordinate1[0] - coordinate2[0]) ** 2 + (coordinate1[1] - coordinate2[1]) ** 2) ** 0.5


def read_agents_fname(agents_fname):
    with open(agents_fname, 'r') as f:
        l = [[num for num in line.split(',')] for line in f]
    number_of_agents = int(l.pop(0)[0])  # pop first row, this is the agents number
    agent_start_dict = {}  # {0: (Sx_0, Sy_0), 1: (Sx_1, Sy_1), ... }
    agent_goal_dict = {}  # {0: (Gx_0, Gy_0), 1: (Gx_1, Gy_1), ... }

    for idx, line in enumerate(l):
        # PAY ATTENTION: indexing reversed (i, j) / (x, y) issue
        # print(idx, line)
        agent_start_dict[idx] = (int(line[1]), int(line[0]))
        agent_goal_dict[idx] = (int(line[3]), int(line[2]))

    # Test dictionary size:
    if len(agent_goal_dict) != number_of_agents or len(agent_start_dict) != number_of_agents:
        raise ValueError('dictionary size doesnt equal to the number of agents')

    return number_of_agents, agent_start_dict, agent_goal_dict


def costumized_histogram(data, colored_hist=False, title='', base_color='blue'):
    plt.figure()
    mean = data.mean()
    bins = 'auto'
    bins = 20
    N, bins, patches = plt.hist(data, bins=bins, density=True, label='NN query distances histogram', edgecolor='black',
                                color=base_color, alpha=0.4)
    if colored_hist:
        # Setting color
        fracs = ((N ** (1 / 5)) / N.max())
        norm = colors.Normalize(fracs.min(), fracs.max())
        for thisfrac, thispatch in zip(fracs, patches):
            color = plt.cm.viridis(norm(thisfrac))
            thispatch.set_facecolor(color)

    # add mean vertical line
    plt.axvline(mean, color='dark'+base_color, linestyle='dashed', linewidth=2, label='Mean')

    # add a 'best fit' line
    from scipy.stats import norm
    (mean, sigma) = norm.fit(data)
    y = norm.pdf(bins, mean, sigma)
    plt.plot(bins, y, base_color[0]+'--', linewidth=2)

    plt.xlabel('Corresponded Agents Distance')
    plt.ylabel('Normalized Frequency')
    plt.legend()
    plt.title(title + r'$\mathrm{\ Histogram\ }\ \mu=%.3f,\ \sigma=%.3f$ ' % (mean, sigma))


# ===== main: =====
agents_num_str = '100'
folder_name = 'files/10obs-20x20map-' + agents_num_str + 'agents/'
map_fname = folder_name + '10obs-20x20.map'

# insert here: #######
query_num = 16       #
exp_query_num = 63   #
base_color = 'red'   #
######################

agents_fname = "10obs-20x20map-" + agents_num_str + "agents-" + "t" + str(query_num) + ".agents"
exp_agents_fname = "10obs-20x20map-" + agents_num_str + "agents-" + str(exp_query_num) + ".agents"

number_of_agents, agent_start_dict, agent_goal_dict = read_agents_fname(folder_name + agents_fname)
_, exp_agent_start_dict, exp_agent_goal_dict = read_agents_fname(folder_name + exp_agents_fname)
Sdistances = np.empty(shape=number_of_agents)

for i in range(number_of_agents):
    Sdistances[i] = SquaredEuclideanDist(agent_start_dict[i], exp_agent_start_dict[i])
costumized_histogram(data=Sdistances, title='Query: t'+str(query_num)+' NN: '+str(exp_query_num), base_color=base_color)

for i in range(number_of_agents):
    Sdistances[i] = EuclideanDist(agent_start_dict[i], exp_agent_start_dict[i])
costumized_histogram(data=Sdistances, title='Query: t'+str(query_num)+' NN: '+str(exp_query_num), base_color=base_color)

# print(np.sum(Sdistances))

plt.show()
