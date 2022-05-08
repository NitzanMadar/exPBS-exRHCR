import networkx as nx
# from networkx.drawing.nx_graph import write_dot, graphiz_layout
import matplotlib.pyplot as plt
import numpy as np


# adapted from: https://stackoverflow.com/questions/29586520/can-one-get-hierarchical-graphs-from-networkx-with-python-3
def hierarchy_pos(G, root, levels=None, width=1., height=1.):
    '''If there is a cycle that is reachable from root, then this will see infinite recursion.
       G: the graph
       root: the root node
       levels: a dictionary
               key: level number (starting from 0)
               value: number of nodes in this level
       width: horizontal space allocated for drawing
       height: vertical space allocated for drawing'''
    TOTAL = "total"
    CURRENT = "current"

    def make_levels(levels, node=root, currentLevel=0, parent=None):
        """Compute the number of nodes for each level
        """
        if not currentLevel in levels:
            levels[currentLevel] = {TOTAL: 0, CURRENT: 0}
        levels[currentLevel][TOTAL] += 1
        neighbors = G.neighbors(node)
        for neighbor in neighbors:
            if not neighbor == parent:
                levels = make_levels(levels, neighbor, currentLevel + 1, node)
        return levels

    def make_pos(pos, node=root, currentLevel=0, parent=None, vert_loc=0):
        dx = 1 / levels[currentLevel][TOTAL]
        left = dx / 2
        pos[node] = ((left + dx * levels[currentLevel][CURRENT]) * width, vert_loc)
        levels[currentLevel][CURRENT] += 1
        neighbors = G.neighbors(node)
        for neighbor in neighbors:
            if not neighbor == parent:
                pos = make_pos(pos, neighbor, currentLevel + 1, node, vert_loc - vert_gap)
        return pos

    if levels is None:
        levels = make_levels({})
    else:
        levels = {l: {TOTAL: levels[l], CURRENT: 0} for l in levels}
    vert_gap = height / (max([l for l in levels]) + 1)
    return make_pos({})

folder_name = "visualization/HL_DFS/"

case_name = "for_depth_visualization_bad_long_dead_end"
case_name = "for_depth_visualization_good"
# case_name = "for_depth_visualization_straight"
fname = folder_name + case_name
# fname = "for_depth_visualization" # run from file create by './driver ....'
with open(fname, 'r') as f:
    print('using file:' + fname)
    l = [[num for num in line.rstrip('\n').split(', ')] for line in f]

depth_dict = {}
parent_dict = {}
list_node = []
edges_list = []
'''
# # # plot tail only... # # #
# if it not work try another number (propagate to unknown parent).
tail_start = {'t2': -100, 't37':-100}
q = 't2'
l = l[tail_start[q]::]
l[0][0] = "ROOT"
# # # # # # # # # # # # # # #
'''

# print(l)
max_depth = 0
min_depth = 0
for parent, child, depth in l:
    child = int(child)
    if child not in list_node:
        list_node.append(child)
        depth_dict[child] = int(depth)
    if parent == "ROOT":
        parent_dict[child] = None
        root = child
        continue
    parent_dict[child] = int(parent)

    edges_list.append((int(parent), int(child)))

    if int(depth) > max_depth:
        max_depth = int(depth)
    # if int(depth) < max_depth:
    #     min_depth = int(depth)
# print(depth_dict)
min_depth = min(depth_dict.items(), key=lambda x: x[1])[1]
# print(min_depth)
# print(list_node)
# print(edges_list)
# print(depth_dict)
# print(parent_dict)


G = nx.Graph()
# G.add_nodes_from(np.unique(depth_list).tolist())
G.add_nodes_from(list_node)
G.add_edges_from(edges_list)

pos = nx.spring_layout(G)
# print(pos)
used_depths = {}
parent_with_left_child = []

'''
for node, node_pos in pos.items():
    # print(node)
    pos_ = pos[node]
    # print(parent_with_left_child)
    # d = 2, 5, 11, 23
    d = depth_dict[node] - min_depth
    if d < 2:
        d = d ** 2 + d
    else:
        d = 3 * 2 ** d - 1
    # print(depth_dict[node], min_depth)
    pos_[1] = -(depth_dict[node] - min_depth)  # /max_depth

    if parent_dict[node] is None or depth_dict[node] == min_depth:
        pos_[0] = 0
        parent_with_left_child.append(parent_dict[node])
        continue
    elif parent_dict[node] not in parent_with_left_child:
        pos_[0] = pos[parent_dict[node]][0] - 1 / d
        parent_with_left_child.append(parent_dict[node])
    else:
        pos_[0] = pos[parent_dict[node]][0] + 1 / d

    pos[node] = pos_
'''

pos = hierarchy_pos(G, root=root)

# print(list_node)
# nx.draw(G, node_size=1, arrows=True, width=1, arrowsize=10, pos=pos, edge_color='grey')
# if list_node[-1] > 400:
#     node_color = "red"
#     node_size = 1
# else:
#     node_color = "green"
#     node_size = 10
node_color = "green" if fname[-4:]=="good" else "red"
name = "good" if fname[-4:]=="good" else "bad"

fig, ax = plt.subplots(figsize=(4, 22))
nx.draw_networkx_nodes(G, pos=pos, node_size=1, node_color=node_color, alpha=0.5)
nx.draw_networkx_edges(G, pos=pos, width=1, edge_color='grey', alpha=0.8)
# plt.gca().set_aspect('equal')
plt.show()
figname = "visualization/HL_DFS/HL_DFS_"+case_name
fig.savefig(figname+".pdf",bbox_inches="tight")
fig.savefig(figname+".png",bbox_inches="tight")
fig.savefig(figname+".eps",bbox_inches="tight")
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.spines['bottom'].set_visible(False)
ax.spines['left'].set_visible(False)
print(f"save figure: {figname}")