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
    Run this code using the line: `python scripts/agents_file_generator.py`
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
    # for i in range(len(si)):
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


# plot function
def plot_map(A):
    # plots:
    fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(20, 20))

    row = len(A)
    col = len(A[0])
    ax[0].imshow(A, cmap='Greys')  # ,  interpolation='nearest')
    ax[0].grid(True, which='major', axis='both', linestyle='-', color='gray')
    ax[0].set_xticks(np.arange(-.5, col, 1))
    ax[0].set_yticks(np.arange(-.5, row, 1))
    plt.setp(ax[0].get_xticklabels(), visible=False)
    plt.setp(ax[0].get_yticklabels(), visible=False)
    ax[0].tick_params(axis='both', which='both', length=0)
    ax[0].set(title='Map')

def atoi(text):
    return int(text) if text.isdigit() else text

def natural_keys(text):
    '''
    alist.sort(key=natural_keys) sorts in human order
    http://nedbatchelder.com/blog/200712/human_sorting.html
    (See Toothy's implementation in the comments)
    '''
    return [ atoi(c) for c in re.split(r'(\d+)', text) ]
"""#._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._.#"""

destdir = "/home/Dropbox/MAPF/PBS_Experience"  # my working dir

# ////////// Inputs: //////////#
map_fname = '10obs-20x20'
folder_name = 'files/'+map_fname+'/'
map_fname = folder_name + map_fname + '.map'
agent_num = int(input("Insert Number of agents: "))



# agent_fname_prefix = (map_fname[:-4] + "map-" + str(agent_num) + "agents-t")  # todo add t for test set
agent_fname_prefix = (folder_name + str(agent_num) + "agents/" + map_fname[len(folder_name):-4] + "map-" + str(
    agent_num) + "agents-")  # todo EDB
output_fname = "Output.csv"

create_for_EDB = False
num_queries_to_gen = 100  # change to 5000 if create_for_EDB will change to True

import click

if click.confirm('Do you want to create EDB (agents + priorities) or not (test set only, without priorities files)? '
                 'Press Ctrl+C to cancel', default=False):
    create_for_EDB = True
    num_queries_to_gen = 5000

if not create_for_EDB:
    agent_fname_prefix = agent_fname_prefix + "t"


if not click.confirm(f'Inputs:\n'
                     f'    - map = {folder_name + map_fname}\n'
                     f'    - number of agents = {agent_num}\n'
                     f'    - number of queries = {num_queries_to_gen}\n'
                     f'    - agents file name: {agent_fname_prefix  + "<i>.agents"}\n'
                     f'Do you want to continue?', default=False):
    raise ValueError


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
# for s in map_gi:
# print(s)

si = []  # list of lists, starts locations
gi = []  # list of lists, starts locations
i = 0
total_runs = 0
start_time = time.time()
while i != num_queries_to_gen:  # to regenerate queries that fails (TIMEOUT or can't be solved)
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
        while map_gi[r][c] == 1 and si[0] != [r,
                                              c]:  # run until finding "open" location that is not the same as the start point
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
        status = subprocess.call(original_pbs, shell=True)

    si = []  # clear, starts locations
    gi = []  # clear, starts locations
    if not create_for_EDB:
        i += 1

if create_for_EDB:
    # this is true if we starts with no agents file at the beginning...
    print(
        f'total runs: {total_runs}, query solved: {num_queries_to_gen} --> success rate: {num_queries_to_gen / total_runs * 100}%')

    # create _.database file:
    agents_EDB_fname = map_fname[len(folder_name):-4] + "map-" + str(agent_num) + "agents-*.agents"
    agents_test_fname = map_fname[len(folder_name):-4] + "map-" + str(agent_num) + "agents-t*.agents"
    agent_folder_name = folder_name + str(agent_num) + "agents/"
    priorities_file_names = []

    for filename in os.listdir('./'+agent_folder_name):
    # print(filename)

        if fnmatch.fnmatch(filename, agents_EDB_fname) and not fnmatch.fnmatch(filename, agents_test_fname):
            # and "t" not in (filename[-9], filename[-10],filename[-11]): # change to "not in"  to avoid test queries
            # print(filename[18:20])
            # print(filename)
            priorities_file_names.append(filename)

    priorities_file_names.sort(key=natural_keys)
    # print(priorities_file_names)
    # # Test:
    print("first 10: ")
    print(priorities_file_names[:10])
    # print("...")
    # print("last 10: ")
    # print (priorities_file_names[-10:])



    # TODO: choose name for the database (note the size at the end - "_._._-<size>.database")
    # database_name = agent_folder_name + agents_fname[:-9]+"testset.database"
    database_name = agent_folder_name + agents_EDB_fname[:-9]+".database"
    # print(database_name)
    data_file = open('./'+database_name, "w+")

    for fname in priorities_file_names:  # [0:1000] to build smaller EDB
    # print(agent_folder_name + fname[:-7] + ".priorities"+", ")
        data_file.write(agent_folder_name + fname[:-7] + ".priorities"+", ")
    with open('./'+agent_folder_name + fname, 'r') as f:
        l = [[num for num in line.split(',')] for line in f]
    l.pop(0)

    for line in l:
    # print(line [0], line[1], line [2], line[3])
        data_file.write(line [0] +", "+ line[1] +", "+ line [2] +", "+ line[3] + ", ")  # TODO - fix the last char in line " ", every line is "x,x, ..., x," need to delete the last ","
    # data_file.
    data_file.write("\n")
    #
    data_file.close()

    print ("file generated:\n" + database_name)


hours, rem = divmod(time.time() - start_time, 3600)
minutes, seconds = divmod(rem, 60)
print("\n\n ===== Total time elapsed: {:0>2}:{:0>2}:{:05.2f} =====\n".format(int(hours), int(minutes), seconds))



#
#
# tot_runs = 0
# i = 0
# # side_cols = [1, 2, 3, 4, 5, 53, 52, 51, 50, 49]  # for kiva domain
# start_time = time.time()
#
# while i != num_queries_to_gen:  # to regenerate queries that fails (TIMEOUT or can't be solved)
#     tot_runs += 1
#     agent_fname = agent_fname_prefix + str(i) + ".agents"
#     print(agent_fname)
#
#     # if there is priorities file and trying to build EDB
#     if path.isfile("./" + agent_fname_prefix + str(i) + ".priorities") and create_for_EDB:
#         print("i++")
#         i += 1
#         continue
#     # if there is already agents file and not for EDB
#     if path.isfile("./" + agent_fname_prefix + str(i) + ".agents") and not create_for_EDB:
#         print("i++")
#         i += 1
#         continue
#
#
#
#
#     # print(agent_fname)
#
#     original_pbs = "./driver --map " + map_fname + " --agents " + agent_fname + \
#                    " --agentNum " + str(agent_num) + " --output " + output_fname
#     print(original_pbs)
#     status = subprocess.call(original_pbs, shell=True)
#
# print(f'===== total runs = {tot_runs}, total solved: {i}===')
#
# """
# map = []
# # Read Map file into binary list of lists (once). The map wrapped map by obstacles
# # Indices starts at 1 not in 0
# with open(map_fname, 'r') as f:
#     for line in f:
#         a = []
#         for bit in line:
#             if bit == '.' or bit == '0':
#                 a.append(0)
#             if bit == '@' or bit == '1' or bit == 'T':
#                 a.append(1)
#             else:
#                 continue
#         map.append(a)
#
# map.pop(0)  # first element indices the map size
# rows = len(map) - 2  # wrapped map by obstacles so minus 2
# cols = len(map[0]) - 2  # wrapped map by obstacles, so minus 2
#
# # # --- Test - print map: ---
# # for s in map:
# #     print(s)
#
# map_si = [x[:] for x in map]  # map for starts
# map_gi = [x[:] for x in map]  # map for goals
#
# si = []  # list of lists, starts locations
# gi = []  # list of lists, starts locations
# """
# queries_to_regenerate_failed_to_solve = []
# for a in range(start_generated_query_index, last_generated_query_index, 1):  # agents file number
#     # map_fname = 'files/kiva/kiva_0.map'
#     # agent_fname_prefix = (map_fname[:-4]+"map-"+str(agent_num)+"agents-") # for database
#
#     # when building a database, sometime we failed to solve - check which queries failed and regenarate new ones
#     # if not path.isfile("files/10obs-20x20map-100agents/10obs-20x20map-100agents-" + str(a) + ".priorities"):
#     if not path.isfile(agent_fname_prefix + str(a) + ".priorities"):
#         # print(a)
#         queries_to_regenerate_failed_to_solve.append(a)
# # print(queries_to_regenerate_failed_to_solve)
#
# print("Failure Percent: " + str(
#     100 * len(queries_to_regenerate_failed_to_solve) / (last_generated_query_index - start_generated_query_index)) +
#       '% (total number: ' + str(len(queries_to_regenerate_failed_to_solve)) + 'queries')
#
# # *** which queries to generate? ***
# # # failed to solve?
# # queries_to_generate = queries_to_regenerate_failed_to_solve
# # # or manually choose number
# queries_to_generate = range(start_generated_query_index, last_generated_query_index)
# """
# tot_runs = 0
# i=0
# # side_cols = [1, 2, 3, 4, 5, 53, 52, 51, 50, 49]  # for kiva domain
#
# while i != num_queries_to_gen:  # to regenerate queries that fails (TIMEOUT or can't be solved)
#     tot_runs+=1
#     print("./"+agent_fname_prefix + str(i) + ".priorities")
#     if path.isfile("./"+agent_fname_prefix + str(i) + ".priorities"):
#         i += 1
#         print('i++')
#         continue
#     # reset start and goal maps
#     map_si = [x[:] for x in map]  # map for starts
#     map_gi = [x[:] for x in map]  # map for goals
#     agent_fname = agent_fname_prefix + str(i) + ".agents"
#     # print(agent_fname)
#
#     for agent in range(agent_num):  # for each agent
#         r = random.randint(1, rows)  # random index
#         c = random.randint(1, cols)  # random index
#         # c = side_cols[random.randint(0, len(side_cols)-1)] # coordinate for kiva domain
#         while map_si[r][c] == 1:  # run until finding "open" location
#             r = random.randint(1, rows)  # change index
#             c = random.randint(1, cols)  # change index
#             # c = side_cols[random.randint(0, len(side_cols)-1)] # coordinate for kiva domain
#         si.append([r, c])  # add the location to stats list
#         map_si[r][c] = 1  # mark this location as closed
#
#         r = random.randint(1, rows)  # new index
#         c = random.randint(1, cols)  # new index
#         # c = side_cols[random.randint(0, len(side_cols)-1)] # coordinate for kiva domain
#         while map_gi[r][c] == 1 and si[0] != [r,
#                                               c]:  # run until finding "open" location that is not the same as the start point
#             r = random.randint(1, rows)  # change index
#             c = random.randint(1, cols)  # change index
#             # c = side_cols[random.randint(0, len(side_cols)-1)] # coordinate for kiva domain
#         gi.append([r, c])  # add the location to goal list
#         map_gi[r][c] = 1  # mark this location as closed
#
#     # generate file:  #todo - remove comment to write...
#     with open(agent_fname, 'w') as f:
#         f.write(str(agent_num)+'\n')
#         for ai in range(agent_num):
#             f.write(str(si[ai][0])+','+str(si[ai][1])+','+
#                     str(gi[ai][0])+','+str(gi[ai][1])+',\n')
#     print("Generate file: " + agent_fname)
#
#     original_pbs = "./driver --map " + map_fname + " --agents " + agent_fname + \
#                    " --agentNum " + str(agent_num) + " --output " + output_fname
#     print(original_pbs)
#     status = subprocess.call(original_pbs, shell=True)
#
#
#     si = []  # clear, starts locations
#     gi = []  # clear, starts locations
#
#
# print (f'===== total runs = {tot_runs}, total solved: {i}===')
#
# """
