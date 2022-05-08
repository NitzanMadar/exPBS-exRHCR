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
import time


"""
    Run this code using the line: `python scripts/kiva33x46_agents_generator.py`
"""

"""#._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._.#"""


def is_possible_goal_loc_sort(r, c, map_g, map_obstacles):
    if r in [2, 36] or c in [1,77] or map_obstacles[r][c] == 1 or map_g[r][c] == 1:
        return False

    elif r in [1, 37]:
        if c%3==0:# in [3,6,9,12,15,18,21,24,27,30,33,36,39,42,45,48,51,54,57,60,63,66,69,72,75]:
            return True
        else:# in [3,6,9,12,15,18,21,24,27,30,33,36,39,42,45,48,51,54,57,60,63,66,69,72,75]:
            return False
    elif (map_obstacles[r][c+1] == 1 or map_obstacles[r][c-1] == 1 or map_obstacles[r+1][c] == 1 or map_obstacles[r-1][c] == 1 ) and map_g[r][c] == 0:
        return True


destdir = "/home/Dropbox/MAPF/PBS_Experience"

# ////////// Inputs: //////////#
folder_name = 'files/sorting_37x77/'
map_fname = folder_name + 'sorting_37x77.map'
agent_num = 600   # TODO: update! ============================================================
num_queries_to_gen = 10

agent_fname_prefix = (folder_name + str(agent_num) + "agents/"+map_fname[len(folder_name):-4]+"map-"+str(agent_num)+"agents-")
output_fname = "Output.csv"

create_for_EDB = True  # TODO: update! ============================================================
# if not create_for_EDB:
#     agent_fname_prefix = agent_fname_prefix + "t"
#     num_queries_to_gen = 100
# //////////////////////////// #

illegal_row = [1, 5, 9, 13, 17, 21, 25, 29, 33]
illegal_col = [1, 4, 7, 18, 29, 40, 43, 46]

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
# for i in range(rows + 1):
#     for j in range(cols + 1):
#         if i in illegal_row or j in illegal_col or 7 <= j <= 40:
#             illegal_map_si[i][j] = 1
#         if i in illegal_row or j in illegal_col or j <= 7 or j >= 40:
#             illegal_map_gi[i][j] = 1

# # --- Test - print map: ---
# for s in map_gi:
# print(s)

si = []  # list of lists, starts locations
gi = []  # list of lists, starts locations
i = 0
total_runs = 0
start_time = time.time()
while i != num_queries_to_gen:  # to regenerate queries that fails (TIMEOUT or can't be solved)
    # if i not in [6]:
    #     i+=1
    #     continue
    total_runs += 1

    # if there is priorities file and trying to build EDB
    if path.isfile("./" + agent_fname_prefix + str(i) + ".priorities") and create_for_EDB:
        print("i++")
        i += 1
        continue
    # if there is already agents file and not for EDB
    if path.isfile("./" + agent_fname_prefix + str(i) + ".agents") and not create_for_EDB:
        print("i++")
        i += 1
        continue

    # reset start and goal maps
    map_si = copy.deepcopy(illegal_map_si)
    map_gi = copy.deepcopy(illegal_map_gi)

    agent_fname = agent_fname_prefix + str(i) + ".agents"
    # print(agent_fname)

    for agent in range(agent_num):  # for each agent
        r = random.randint(1, rows)  # random index
        c = random.randint(1, cols)  # random index
        # c = side_cols[random.randint(0, len(side_cols)-1)] # coordinate for kiva domain
        while map_si[r][c] == 1:  # run until finding "open" location
            r = random.randint(1, rows)  # change index
            c = random.randint(1, cols)  # change index
            # c = side_cols[random.randint(0, len(side_cols)-1)] # coordinate for kiva domain
        si.append([r, c])  # add the location to stats list
        map_si[r][c] = 1  # mark this location as closed

        r = random.randint(1, rows)  # new index
        c = random.randint(1, cols)  # new index
        # c = side_cols[random.randint(0, len(side_cols)-1)] # coordinate for kiva domain
        while not is_possible_goal_loc_sort(r,c,map_gi, map) and si[0] != [r,c]:  # run until finding "open" location that is not the same as the start point
            r = random.randint(1, rows)  # change index
            c = random.randint(1, cols)  # change index
            # c = side_cols[random.randint(0, len(side_cols)-1)] # coordinate for kiva domain
        gi.append([r, c])  # add the location to goal list
        map_gi[r][c] = 1  # mark this location as closed

    # generate file:  #todo - remove comment to write...
    with open(agent_fname, 'w') as f:
        f.write(str(agent_num) + '\n')
        for ai in range(agent_num):
            f.write(str(si[ai][0]) + ',' + str(si[ai][1]) + ',' +
                    str(gi[ai][0]) + ',' + str(gi[ai][1]) + ',\n')
    print("Generate file: " + agent_fname)

    # solve with original PBS (if need to create priorities file change in driver "bool to_save_P_matrix = true;"
    if create_for_EDB:
        original_pbs = "./driver --map " + map_fname + " --agents " + agent_fname + \
                       " --agentNum " + str(agent_num) + " --output " + output_fname + " --to_save_P_matrix 1"
        print(original_pbs)
        # status = subprocess.call(original_pbs, shell=True)

    si = []  # clear, starts locations
    gi = []  # clear, starts locations
    if not create_for_EDB:
        i += 1

if create_for_EDB:
    # this is true if we starts with no agents file at the beginning...
    print(
        f'total runs: {total_runs}, query solved: {num_queries_to_gen} --> success rate: {num_queries_to_gen / (total_runs-1) * 100}%')

hours, rem = divmod(time.time() - start_time, 3600)
minutes, seconds = divmod(rem, 60)
print("\n\n ===== Total time elapsed: {:0>2}:{:0>2}:{:05.2f} =====\n".format(int(hours), int(minutes), seconds))
