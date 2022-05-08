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
    This script generate agents files which the goals are differ from the start by given steps (parameter).
    Those files generated in loop until runtime limit and in each round it solved with some algorithm.
    The output written into csv file, and need a post process to understand which algorithm solve more in an accumulated
    runtime limit. It also possible to estimate the throughput assuming the average distance per task.
    
    Run this code using the line: `python3 scripts/pseudo_lifelong_simulator.py`
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

def read_map_file(map_fname):
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
    return map, rows, cols

def generate_goal_with_given_manhattan_distance(start_c, start_r, dist, max_col, max_row):
    Z = [-1, 1]
    success = False
    while not success:
        plusminus_r = Z[random.randint(0, 1)]
        delta_r = plusminus_r * random.randint(0, dist)  # new index
        r = start_r + delta_r
        plusminus_c = Z[random.randint(0, 1)]
        delta_c = plusminus_c * (dist - abs(delta_r))  # new index
        c = start_c + delta_c
        if c<1 or c>max_col or r<1 or r>max_row:
            success = False
        else:
            success = True

    return r,c
"""#._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._.#"""


destdir = "/home/Dropbox/MAPF/PBS_Experience"

# ////////// Inputs: //////////#
folder_name = 'files/sorting_37x77/'
folder_name = 'files/kiva_33x46/'
map_fname = folder_name + 'sorting_37x77.map'
map_fname = folder_name + 'kiva_33x46.map'
agent_num = 180   # TODO: update! ============================================================
s_to_g_distance = 5  # planning horizon in Li et al. 2020 Lifelong Multi-Agent Path Finding in Large-Scale Warehouses

agent_fname_prefix = (folder_name + str(agent_num) + "agents/windowed_queries/"+map_fname[len(folder_name):-4]+"map-"+str(agent_num)+"agents-")
output_fname = "Output_pseudo_lifelong.csv"

# //////////////////////////// #

illegal_row = [1, 5, 9, 13, 17, 21, 25, 29, 33]
illegal_col = [1, 4, 7, 18, 29, 40, 43, 46]

map, rows, cols = read_map_file(map_fname=map_fname)

# illegal position maps
illegal_map_si = [x[:] for x in map]  # map for starts
illegal_map_gi = [x[:] for x in map]  # map for goals

# # --- Test - print map: ---
# for s in map_gi:
# print(s)
1


time_accumulated = 0  # [sec]
total_runtime = 60*10  # [sec]
waitbar = tqdm(total=total_runtime, position=0, leave=True)
stop = False
# agent_file_object = open(agent_fname, 'w')
q_num = 0
while time_accumulated < total_runtime and not stop:
    # stop used for check generate file and skip algorithms run and stop the while (next line)
    # stop = True
    # reset start and goal maps
    map_si = copy.deepcopy(illegal_map_si)
    map_gi = copy.deepcopy(illegal_map_gi)
    agent_fname = agent_fname_prefix + "0"+str(q_num)+".agents"
    q_num +=1

    si = []  # list of lists, starts locations
    gi = []  # list of lists, starts locations
    # print(agent_fname)

    for agent in range(agent_num):  # for each agent
        ### starts ###
        r = random.randint(1, rows)  # random index
        c = random.randint(1, cols)  # random index
        # c = side_cols[random.randint(0, len(side_cols)-1)] # coordinate for kiva domain
        while map_si[r][c] == 1:  # run until finding "open" location
            r = random.randint(1, rows)  # change index
            c = random.randint(1, cols)  # change index
            # c = side_cols[random.randint(0, len(side_cols)-1)] # coordinate for kiva domain
        si.append([r, c])  # add the location to stats list
        map_si[r][c] = 1  # mark this location as closed
        start_r = r
        start_c = c

        ### goals ###
        delta = s_to_g_distance  # how many steps to jump (Manhattan distance)
        if random.randint(0, 1) == 0:  # sometime, change the number of steps to consider some waits actions
            delta = random.randint(s_to_g_distance-2, s_to_g_distance)
        r, c = generate_goal_with_given_manhattan_distance(start_c=start_c, start_r=start_r, dist=delta, max_col=cols, max_row=rows)
        # Z = [-1, 1]
        # plusminus_r = Z[random.randint(0, 1)]
        # delta_r = plusminus_r * random.randint(0, delta)  # new index
        # r = start_r + delta_r
        # plusminus_c = Z[random.randint(0, 1)]
        # delta_c = plusminus_c * (delta - abs(delta_r))  # new index
        # c = start_c + delta_c
        # c = max(0, c)
        # c = min(c, cols)
        # r = max(0, r)
        # r = min(r, rows)
        while map_gi[r][c] == 1:  # run until finding "open" location that is not the same as the start point
            # change index

            r, c = generate_goal_with_given_manhattan_distance(start_c=start_c, start_r=start_r, dist=delta, max_col=cols, max_row=rows)

        gi.append([r, c])  # add the location to goal list
        map_gi[r][c] = 1  # mark this location as closed

    # generate file:  #todo - remove comment to write...
    with open(agent_fname, 'w') as f:
        f.write(str(agent_num) + '\n')
        for ai in range(agent_num):
            f.write(str(si[ai][0]) + ',' + str(si[ai][1]) + ',' +
                    str(gi[ai][0]) + ',' + str(gi[ai][1]) + ',\n')
    f.close()
    print("\nGenerate file: " + agent_fname)
    if stop:
        continue
    t_i = time.time()
    # run PBS #
    original_pbs = "./driver --map " + map_fname + " --agents " + agent_fname + \
                   " --agentNum " + str(agent_num) + " --output " + output_fname
    print(original_pbs)
    status = subprocess.call(original_pbs, shell=True)

    # run exPBS versions:
    widths = [-1, 10]
    fallbacks = [0, 0.75, 1, -0.75, -1, 2, 3]  # 0 / (0,1] / 2 / 3
    fallback_dict = {0: "None", 0.75: "3/4 tail P_d", 1: "end tail P_d",
                     -0.75: "3/4 tail P_d-P_exp", -1: "end tail P_d-P_exp",
                     2: "P_exp1 -> Original PBS", 3: "P_exp1 -> P_exp2 -> Original PBS"}

    width = widths[1]
    fallback = fallbacks[2]
    pbs_with_experience = "./driver --map " + map_fname + " --agents " + agent_fname + \
                          " --agentNum " +  str(agent_num)  + " --output " + output_fname + \
                          " --experience " + "1" + " --fallback " + str(fallback)+ \
                          " --width_limit_hl " + str(width) + " --cleaning_threshold -1"

    print(pbs_with_experience)
    status = subprocess.call(pbs_with_experience, shell=True)

    # ====

    width = widths[1]
    fallback = fallbacks[4]
    pbs_with_experience = "./driver --map " + map_fname + " --agents " + agent_fname + \
                          " --agentNum " +  str(agent_num)  + " --output " + output_fname + \
                          " --experience " + "1" + " --fallback " + str(fallback)+ \
                          " --width_limit_hl " + str(width) + " --cleaning_threshold -1"

    print(pbs_with_experience)
    status = subprocess.call(pbs_with_experience, shell=True)

    #
    # width = widths[1]
    # fallback = fallbacks[-1]
    #
    # pbs_with_experience_with_clean = "./driver --map " + map_fname + " --agents " + agent_fname + \
    #                                  " --agentNum " +  str(agent_num)  + " --output " + \
    #                                  output_fname + " --experience " + "2" + " --fallback " + str(fallback) + \
    #                                  " --width_limit_hl " + str(width) + " --cleaning_threshold -2"  # --cleaning_threshold = -2 means clean by 1*average distance, cleaning_threshold = -1 means do not clean
    # print(pbs_with_experience_with_clean)
    # status = subprocess.call(pbs_with_experience_with_clean, shell=True)

    delta_T = time.time() - t_i
    time_accumulated += delta_T

    # if time_accumulated > total_runtime:
    waitbar.update(delta_T)
    print(f'solved {q_num} queries, results reported in {output_fname}')