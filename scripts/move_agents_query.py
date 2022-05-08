import time
import copy
import os
import sys
import resource
import subprocess
import shutil
import zipfile
import smtplib
from tqdm import tqdm
import random
import matplotlib.pyplot as plt
from os import path
import fnmatch
import re
import click

"""
    This script generate random agents file given some parameters.
    Run this code using the line: `python scripts/move_agents_query.py`
"""


def move_agent_manhattan(si, gi, map_si, map_gi, dist):
    # This function move each agent by 1 or 2 steps
    global agent_num
    # global map
    # init_row = init_loc[0] # start in 1 not in 0
    # init_col = init_loc[1] # start in 1 not in 0
    # succ = False
    visited = []

    for step_num in range(dist):
        i = random.randint(0, agent_num - 1)  # random agent
        succ = False
        counter = 0
        visited = []
        while not succ:
            counter = counter + 1
            if counter > 50:  # surrounded agent - can't move
                i = random.randint(0, agent_num - 1)  # random agent
            my_randi = random.randint(-1, 1)  # -1 \ 0 \ 1
            Z = [-1, 1]
            plusminus = Z[random.randint(0, 1)]
            s_or_g = random.randint(0, 1)  # move in s or g
            s_or_g = 1  # g
            s_or_g = 0  # s
            if s_or_g == 0:  # , si
                new_row = si[i][0] + my_randi
                new_col = si[i][1] + plusminus * (1 - abs(my_randi))
                if map_si[new_row][new_col] == 0:  # no obstacle or other agent
                    # switch positions
                    map_si[new_row][new_row] == 1
                    map_si[si[i][0]][si[i][1]] == 0
                    succ = True
                    # visited.append([si[i][0]-1,si[i][1]])
                    # visited.remove([si[i][0],si[i][1]])

                    si[i][0] = new_row
                    si[i][1] = new_col
                    print(new_row, new_col, 's')

            else:  # , gi
                new_row = gi[i][0] + my_randi
                new_col = gi[i][1] + plusminus * (1 - abs(my_randi))

                if map_gi[new_row][new_col] == 0:  # no obstacle or other agent
                    # switch positions
                    map_gi[new_row][new_row] == 1
                    map_gi[gi[i][0]][gi[i][1]] == 0
                    succ = True
                    # visited.append([si[i][0]-1,si[i][1]])
                    # visited.remove([si[i][0],si[i][1]])
                    gi[i][0] = new_row
                    gi[i][1] = new_col
                    print(new_row, new_col, 'g')

        # visited

    # # move every agent "dist" steps
    # for i in range(len(si)):#
    #
    #     # init_row = si[i][0] # start in 1 not in 0
    #     # init_col = si[i][1] # start in 1 not in 0
    #     visited = [[si[i][0],si[i][1]]]
    #     counter=0
    #     succ = False
    #     while not succ:
    #         counter = counter+1
    #         if counter>50:
    #             # print("surrounded agent, error")
    #             break
    #         my_randi = random.randint(-dist,dist)
    #         Z = [-1,1]
    #         plusminus = Z[random.randint(0,1)]
    #         new_row = si[i][0] + my_randi
    #         new_col = plusminus*(si[i][1] + dist - abs(my_randi))
    #
    #         if new_row<len(map) and new_row>0 and new_col<len(map[0]) and new_col>0: #inside map
    #             if map_si[new_row][new_col] == 0: #no obstacle or other agent
    #                 # switch positions
    #                 map_si[new_row][new_row] == 1
    #                 map_si[si[i][0]][si[i][1]] == 0
    #                 succ = True
    #                 # visited.append([si[i][0]-1,si[i][1]])
    #                 # visited.remove([si[i][0],si[i][1]])
    #                 si[i][0] = new_row
    #                 si[i][1] = new_col
    # print(succ)

    return si, gi

def atoi(text):
    return int(text) if text.isdigit() else text

def natural_keys(text):
    '''
    alist.sort(key=natural_keys) sorts in human order
    http://nedbatchelder.com/blog/200712/human_sorting.html
    (See Toothy's implementation in the comments)
    '''
    return [ atoi(c) for c in re.split(r'(\d+)', text) ]


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

    for idx, line in enumerate(l):
        # PAY ATTENTION: indexing reversed (i, j) / (x, y) issue
        # print(idx, line)
        agent_start_dict[idx] = (int(line[1]), int(line[0]))
        agent_goal_dict[idx] = (int(line[3]), int(line[2]))


    # Test dictionary size:
    if (len(agent_goal_dict) != k or len(agent_start_dict) != k):
        raise ValueError('dictionary size doesnt equal to the number of agents')

    return k, agent_start_dict, agent_goal_dict
"""#._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._.#"""

destdir = "/home/Dropbox/MAPF/PBS_Experience"  # my working dir

# ////////// Inputs: //////////#
map_fname = '10obs-20x20'
folder_name = 'files/'+map_fname+'/'
map_fname = folder_name + map_fname + '.map'
# agent_num = 120
agent_num = int(input("Insert Number of agents: "))




# agent_fname_prefix = (map_fname[:-4] + "map-" + str(agent_num) + "agents-t")  # todo add t for test set
agent_fname_prefix = (folder_name + str(agent_num) + "agents/" + map_fname[len(folder_name):-4] + "map-" + str(
    agent_num) + "agents-")  # todo EDB
output_fname = "Output.csv"

agents_fname = agent_fname_prefix+ '01.agents'

# //////////////////////////// #

map = []
# Read Map file into binary list of lists (once). The map wrapped map by obstacles
# Indices starts at 1 not in 0
with open(map_fname, 'r') as f:
    for line in f:
        a = []
        for bit in line:
            if bit == '.' or bit == '0':
                a.append(0)
            if bit == '@' or bit == '1' or bit == 'T':
                a.append(1)
            else:
                continue
        map.append(a)

map.pop(0)  # first element indices the map size
rows = len(map) - 2  # wrapped map by obstacles so minus 2
cols = len(map[0]) - 2  # wrapped map by obstacles, so minus 2



# illegal position maps
illegal_map_si = [x[:] for x in map]  # map for starts
illegal_map_gi = [x[:] for x in map]  # map for goals

# illegal_row = []
# illegal_col = []
# for i in range(rows + 1):
#     for j in range(cols + 1):
#         if i in illegal_row or j in illegal_col or 7 <= j <= 40:
#             illegal_map_si[i][j] = 1
#         if i in illegal_row or j in illegal_col or j <= 7 or j >= 40:
#             illegal_map_gi[i][j] = 1



# # --- Test - print map: ---
for s in illegal_map_si:
    print(s)

k, agent_start_dict, agent_goal_dict = read_agents_fname(agents_fname=agents_fname)
# print(k, agent_start_dict, agent_goal_dict)
print(agent_start_dict)

# print(illegal_map_gi == illegal_map_si) ==> True
new_starts = {}
new_goals = {}
for i in range(k):
    # start location
    success = False
    while not success:
        Z = [-1, 1]
        plusminus = Z[random.randint(0, 1)]
        idx_to_move = random.randint(0, 1)  # row / col
        original_s = agent_start_dict[i]
        new_s = [original_s[0], original_s[1]]
        new_s[idx_to_move] = new_s[idx_to_move] + plusminus
        if illegal_map_si[new_s[1]][new_s[0]] == 0:
            success = True
            illegal_map_si[new_s[1]][new_s[0]] = 1
            new_starts[i] = new_s

    # goals
    success = False
    while not success:
        Z = [-1, 1]
        plusminus = Z[random.randint(0, 1)]
        idx_to_move = random.randint(0, 1)  # row / col
        original_g = agent_goal_dict[i]
        new_g = [original_g[0], original_g[1]]
        new_g[idx_to_move] = new_g[idx_to_move] + plusminus
        if illegal_map_gi[new_g[1]][new_g[0]] == 0:
            success = True
            illegal_map_gi[new_g[1]][new_g[0]] = 1
            new_goals[i] = new_g

print('new_starts\n',new_starts)
print('new_goals\n',new_goals)

with open(agents_fname, 'w') as f:
    f.write(str(k) + '\n')
    for i in range(k):
        f.write(str(new_starts[i][1]) + ',' + str(new_starts[i][0]) + ',' +
                str(new_goals[i][1]) + ',' + str(new_goals[i][0]) + ',\n')
print("Generate file: " + agents_fname)

