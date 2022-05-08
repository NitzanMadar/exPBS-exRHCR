import pickle
import numpy as np
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
from tqdm import tqdm
import time
import click
import argparse

"""
    run this code using the line:
    python scripts/run_pbs_test.py
"""

# functions:
def read_agents_fname(agents_fname):
    start_and_goal_can_be_at_same_loc = True
    delta = 0
    if start_and_goal_can_be_at_same_loc:
        delta = 0.25
    with open(agents_fname, 'r') as f:
        l = [[num for num in line.split(',')] for line in f]
    k = int(l.pop(0)[0])  # pop first row, this is the agents number
    agent_start_dict = {}  # {0: (Sx_0, Sy_0), 1: (Sx_1, Sy_1), ... }
    agent_goal_dict = {}  # {0: (Gx_0, Gy_0), 1: (Gx_1, Gy_1), ... }

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

def create_load_mission():
    possible_R = [1,37]
    possible_C = [3,6,9,12,15,18,21,24,27,30,33,36,39,42,45,48,51,54,57,60,63,66,69,72,75]
    rr = possible_R[random.randint(0, len(possible_R)-1)]  # new index
    cc = possible_C[random.randint(0, len(possible_C)-1)]  # new index
    return rr,cc


def is_in_working_station(r,c):
    return r in [1,37] and c%3==0

def is_possible_goal_loc_sorting_center(r, c, map_g, map_obstacles):
    if r in [2, 36] or c in [1, 77] or map_obstacles[r][c] == 1 or map_g[r][c] == 1:
        return False

    elif r in [1, 37]:
        if c % 3 == 0:  # in [3,6,9,12,15,18,21,24,27,30,33,36,39,42,45,48,51,54,57,60,63,66,69,72,75]:
            return True
        else:  # in [3,6,9,12,15,18,21,24,27,30,33,36,39,42,45,48,51,54,57,60,63,66,69,72,75]:
            return False
    elif (map_obstacles[r][c + 1] == 1 or map_obstacles[r][c - 1] == 1 or map_obstacles[r + 1][c] == 1 or
          map_obstacles[r - 1][c] == 1) and map_g[r][c] == 0:
        return True


def generate_agents_file(agent_fname, illegal__si, illegal__gi, number_of_agents):
    map_si = copy.deepcopy(illegal__si)
    map_gi = copy.deepcopy(illegal__gi)
    map, rows, cols = read_map_file(map_fname=map_fname)

    si = []  # list of lists, starts locations
    gi = []  # list of lists, starts locations
    for agent in range(number_of_agents):  # for each agent
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
        while not is_possible_goal_loc_sorting_center(r, c, map_gi, map) or si[agent] == [r, c] or map_gi[r][c] == 1:  # run until finding "open" location that is not the same as the start point
            r = random.randint(1, rows)  # change index
            c = random.randint(1, cols)  # change index
            # c = side_cols[random.randint(0, len(side_cols)-1)] # coordinate for kiva domain
        gi.append([r, c])  # add the location to goal list
        map_gi[r][c] = 1  # mark this location as closed

    # generate file:  #todo - remove comment to write...
    with open(agent_fname, 'w') as f:
        f.write(str(number_of_agents) + '\n')
        for ai in range(number_of_agents):
            f.write(str(si[ai][0]) + ',' + str(si[ai][1]) + ',' +
                    str(gi[ai][0]) + ',' + str(gi[ai][1]) + ',\n')
    print("Generate file: " + agent_fname)


def euclidean_distance(A,B):
    return ((A[0]-B[0])**2+(A[1]-B[1])**2)**.5


def assign_new_missions(agent_fname_, mew_missions):
    # map_si = copy.deepcopy(illegal_map_si)
    # map_gi = copy.deepcopy(illegal_map_gi)

    k, agent_start_dict, agent_goal_dict = read_agents_fname(agents_fname=agent_fname_)
    agents_to_update_g = []
    for i in range(k):
        if agent_start_dict[i] == agent_goal_dict[i] : #euclidean_distance(agent_start_dict[i], agent_goal_dict[i])<= 3:#agent_start_dict[i] == agent_goal_dict[i] : #TODO

            # print(agent_start_dict[i], agent_goal_dict[i])
            agents_to_update_g.append(i)  # agent_i will locate in line i+1 in _.agents files
    assigned = []
    missions_finished = len(agents_to_update_g)
    a_file = open(agent_fname_, "r")
    list_of_lines = a_file.readlines()
    for i in agents_to_update_g:
        counter = 0  # if didnt found a position after 10 times, probably all working station are assigned
        prev_c, prev_r = agent_start_dict[i]
        # mission_type = 0 if is_in_working_station(prev_r, prev_c) else 1
        for m in range(len(mew_missions)):
            r, c = mew_missions[m]
            if counter < 100 and is_in_working_station(prev_r, prev_c) == is_in_working_station(r,c):  # new position is also start or also goal
                counter += 1
                continue

            if (c, r) not in agent_goal_dict.values() and [r, c] not in assigned:

                mew_missions.pop(m)
                # print(f'r,c = {r}, {c}')
                # print(f'agent_goal_dict = {agent_goal_dict}')
                # print(f"assign from {agent_start_dict[i]} to {c, r}" )
                # input('is new mission correct?')
                assigned.append([r,c])
                break
        newline = str(agent_start_dict[i][1]) + "," + str(agent_start_dict[i][0]) + "," + str(r) + "," + str(c) + ",\n"
        list_of_lines[i + 1] = newline  # agent_i will locate in line i+1 in _.agents files
        # print(f'assign new location for agent {i}: {newline}')

    #     # if agent holding end location and other agents (with lower priority or none priority) occupy this vertex, a collision occurs
    # # fix it here
    # for ai in range(len(agent_start_dict)):
    #     for aj in range(ai, len(agent_start_dict)):
    #         if agent_start_dict[ai] == agent_start_dict[aj]:
    #             list_of_lines[aj+1]

    print(f'assign new location for: {agents_to_update_g}')
    # input('before assign')
    a_file = open(agent_fname_, "w")
    a_file.writelines(list_of_lines)
    a_file.close()
    print(f'len(mew_missions)={len(mew_missions)}')


    # input(f'check file: {agent_fname_}')
    return missions_finished


def create_new_mission_list(agents_fname_, illegal_gi):
    to_visualize_goal_locs_on_map= False
    map_gi_zeros = copy.deepcopy(illegal_gi)
    for xx in range(len(map_gi_zeros)):
        for yy in range(len(map_gi_zeros[0])):
            map_gi_zeros[xx][yy]=0
    map, rows, cols = read_map_file(map_fname=map_fname)
    # print(f'map[19,30] = {map[19][30]} or {map[30][19]}')
    k, agent_start_dict, agent_goal_dict = read_agents_fname(agents_fname=agents_fname_)
    new_missions = []  # [drop mission], [load mission]
    for _ in range(int(10000/2)):
        r = random.randint(1, rows)  # new index
        c = random.randint(1, cols)  # new index
        while not is_possible_goal_loc_sorting_center(r, c, map_gi_zeros, map) and not is_in_working_station(r, c):  # run until finding "open" legal goal location
            r = random.randint(1, rows)  # change index
            c = random.randint(1, cols)  # change index
        new_missions.append([r,c])

        r,c = create_load_mission()
        while not is_possible_goal_loc_sorting_center(r, c, map_gi_zeros, map):  # run until finding "open" working station goal location
            r, c = create_load_mission()
        new_missions.append([r,c])  # 1 - load mission, perimeter

    if to_visualize_goal_locs_on_map:
        map1 = copy.deepcopy(map)
        for elem in new_missions:
            map1[elem[0]][elem[1]] = 0.5
        # map1[2][2]=0.5
        fig, ax = plt.subplots()
        map1 = np.array(map1)
        im = ax.imshow(map1, interpolation='none', #cmap='Greys',
                       )

        plt.show()

        input('press Enter to continue...')
    return new_missions

def check_collisions(agents_fname_):
    _, agent_start_dict, agent_goal_dict = read_agents_fname(agents_fname=agents_fname_)
    rev_dict={}
    for key, value in agent_start_dict.items():
            rev_dict.setdefault(value, set()).add(key)

    result = [values for key, values in rev_dict.items()
              if len(values) > 1]

    if len (result) > 0:
        print(f'collision in ==Start== locations has detected!!!!! agents: {result}')
        # print(agent_start_dict)
        input("Press Enter to Continue...")
    rev_dict={}
    for key, value in agent_goal_dict.items():
            rev_dict.setdefault(value, set()).add(key)

    result = [values for key, values in rev_dict.items()
              if len(values) > 1]

    if len (result) > 0:
        print(f'collision in **GOAL** locations has detected!!!!! ')#agents: {result}')
        # print(agent_goal_dict)
        # input("Press Enter to Continue...")


def build_dummy_edb(agents_w_fname):
    priorities_fname = agents_w_fname[:-7]+".priorities"
    # print(priorities_fname)
    database_name = agents_w_fname[:-10]+".database"
    # print(database_name)
    data_file = open('./'+database_name, "w+")
    data_file.write(priorities_fname+", ")
    with open('./'+agents_w_fname, 'r') as f:
        l = [[num for num in line.split(',')] for line in f]
    l.pop(0)

    for line in l:
        # print(line [0], line[1], line [2], line[3])
        data_file.write(line [0] +", "+ line[1] +", "+ line [2] +", "+ line[3] + ", ")  # TODO - fix the last char in line " ", every line is "x,x, ..., x," need to delete the last ","
    data_file.write("\n")

    data_file.close()

##########################################################################################

# inputs:
prefix_dir = '../'
destdir = "/home/Dropbox/MAPF/PBS_Experience"
folder_name = prefix_dir + 'files/sorting_37x77/'
map_fname = folder_name + 'sorting_37x77.map'
parser = argparse.ArgumentParser(description='Test Arguments')
parser.add_argument("-a", "--num_of_agent", help="choose number of agents")
parser.add_argument("-d", "--delta", help="choose number of agents")
parser.add_argument("-t", "--test_num", help="saved test number")
parser.add_argument("-c", "--create_and_save", help="create and save test")
args = parser.parse_args()
number_of_agents = int(args.num_of_agent)
J = int(args.delta) # PBS --> J*exPBS
test_num = args.test_num
create_and_save = int(args.create_and_save)

window_size = 10
h = 5

# number_of_agents = 400
output_fname = folder_name + str(number_of_agents) + "agents/lifelong/Output_lifelong_sorting_"+str(number_of_agents)+"_PBS-"+str(J)+"exPBS+BLDFS_w="+str(window_size)+"-h="+str(h)+"_batch"+".csv"
throughput_PBS = 0
throughput_exPBS = 0

# agents file name will be used and overwrite iteratively. Be careful!
agent_fname = (folder_name + str(number_of_agents) + "agents/lifelong/" + map_fname[len(folder_name):-4] + "map-" + str(
    number_of_agents) + "agents-w.agents")
agent_fname_expbs = (folder_name + str(number_of_agents) + "agents/lifelong/" + map_fname[len(folder_name):-4] + "map-" + str(
    number_of_agents) + "agents-_w.agents")

agent_save_test_file = (folder_name + str(number_of_agents) + "agents/lifelong/" + map_fname[len(folder_name):-4] + "map-" + str(
    number_of_agents) + "agents-test"+test_num+".agents")
tasks_saved_test = (folder_name + str(number_of_agents) + "agents/lifelong/" + "tasks-test"+test_num+".pkl")
print(f'map_fname: {map_fname}\nagent_fname: {agent_fname}')

# read map:
map, rows, cols = read_map_file(map_fname=map_fname)
# illegal position maps
illegal_map_si = [x[:] for x in map]  # map for starts
illegal_map_gi = [x[:] for x in map]  # map for goals


if create_and_save:
    generate_new = True
    save_generated = True
    # input('create and save test query?')
else:
    generate_new = False
    save_generated = False

if generate_new:

    # generate agents files for sorting center: (other scenario needs other generate_agents_file function)
    generate_agents_file(agent_fname=agent_fname, illegal__si=illegal_map_si, illegal__gi=illegal_map_gi, number_of_agents=number_of_agents)
    check_collisions(agent_fname)
    if save_generated:
        if os.path.isfile(agent_save_test_file):
            pass
            # input(f'are you sure you want to override {agent_save_test_file}?')
            #     print('xx')
        shutil.copy(agent_fname, agent_save_test_file)
else:
    print(f'load agents file - {agent_save_test_file}')
    # copy saved test file to running agents file
    shutil.copy(agent_save_test_file, agent_fname)

# copy agents file to be used for exPBS:
shutil.copy(agent_fname, agent_fname_expbs)
# shutil.copy(agent_fname, agent_fname_expbs2)

print(f'Copied {agent_fname}\n   to: {agent_fname_expbs}')

# create dummy EDB with size of one, we need it to point on PBS priority for the hybrid solver
create_edb=True
for fname in os.listdir('.'):
    if fname.endswith('.database'):
        create_edb=False
if create_edb:
    build_dummy_edb(agent_fname_expbs)
    # build_dummy_edb(agent_fname_expbs2)
# input('check EDB')

if generate_new:

    new_mission_list = create_new_mission_list(agents_fname_=agent_fname,illegal_gi=illegal_map_gi)
    if save_generated:
        with open(tasks_saved_test, 'wb+') as f:
            pickle.dump(new_mission_list, f)
        # input('test pickle')
else:  # read
    with open(tasks_saved_test, 'rb') as f:
        new_mission_list =pickle.load(f)

new_mission_list_expbs = copy.deepcopy(new_mission_list)
# new_mission_list_expbs2 = copy.deepcopy(new_mission_list)


total_step_limit = h*100
steps_done = 0
# accumulated = 0
# output CSV title:
if J == 0:
    print('#######################################\n'
          '################# PBS #################\n'
          '#######################################')
    with open(output_fname, 'a') as f:
        f.write(f'SearchRuntime,BuildDatabaseRuntime,FindNNRuntime,Agentsfile,Experience,NearestNeighborIndex,'
                f'NearestNeighborDistance,ConstraintsMeasure,HLexpanded,HLgenerated,LLexpanded,LLgenerated,Cost,SolDepth,'
                f'WidthLimit,Fallback,max_breadth\n')
        f.write(f'PBS, {number_of_agents} agents, w=h={window_size}, total steps = {total_step_limit}\n')
    f.close()

    while steps_done < total_step_limit:
        # # read agents start and goal locations:
        # _, agent_start_dict, agent_goal_dict = read_agents_fname(agents_fname=agent_fname)
        print(f'\nrun number = {int(steps_done/h+1)} / {int(total_step_limit/h)}\n')
        # update goals for agents which reached the their goals

        missions_finished = assign_new_missions(agent_fname_=agent_fname, mew_missions=new_mission_list)
        throughput_PBS += missions_finished
        # input("before run...")
        original_pbs = prefix_dir + 'driver --map '+map_fname+' --agents ' + agent_fname + \
                       ' --agentNum ' + str(number_of_agents) +\
                       ' --output '+output_fname+' --to_save_P_matrix 0 --windowed_mapf ' + str(window_size)+ ' --frequency_mapd ' + str(h)
        # original_pbs_bldfs = './driver --map files/sorting_37x77/sorting_37x77.map --agents ' + agent_fname + \
        #                      ' --agentNum ' + str(number_of_agents) + ' --width_limit_hl 10 --fallback 1' + \
        #                      ' --output '+output_fname+' --to_save_P_matrix 0 --windowed_mapf ' + str(window_size)
        print(original_pbs)

        check_collisions(agents_fname_=agent_fname)
        status = subprocess.call(original_pbs, shell=True)


        steps_done += h
    print(f'throughput PBS = {throughput_PBS}')

    # input('stop?')


else:  # exRHCR, J!=0
    print('#######################################\n'
          '################ exPBS ################\n'
          '#######################################')
    accumulated = 0
    steps_done = 0
    w_PBS = 10
    w_exPBS = 10
    h_PBS = h
    h_exPBS = h

    if w_PBS + J*w_exPBS != (J+1)*window_size:
        print(f'w_PBS + J*w_exPBS != (J+1)*window_size --> not equivalent for one run cycle. J={J}, w_pbs={w_PBS}, w_expbs={w_exPBS}')
        input('  OK?')

    with open(output_fname, 'a') as f:
        f.write(f'\n\nexPBS, {number_of_agents} agents, PBS:w={w_PBS} h={h_PBS},->J={J},exPBS:w={w_exPBS} h={h_exPBS} J={J}, total steps = {total_step_limit}\n')
        # f.write(f'SearchRuntime,BuildDatabaseRuntime,FindNNRuntime,Agentsfile,Experience,NearestNeighborIndex,'
        #         f'NearestNeighborDistance,ConstraintsMeasure,HLexpanded,HLgenerated,LLexpanded,LLgenerated,Cost,SolDepth,'
        #         f'WidthLimit,Fallback\n')
    f.close()



    while steps_done < total_step_limit:
        # # read agents start and goal locations:
        # _, agent_start_dict, agent_goal_dict = read_agents_fname(agents_fname=agent_fname_expbs)
        print(f'\n\trun number = {int(steps_done)} / {int(total_step_limit)},\t\t now running PBS')
        # update goals for agents which reached the their goals
        missions_finished = assign_new_missions(agent_fname_=agent_fname_expbs,  mew_missions=new_mission_list_expbs)
        throughput_exPBS += missions_finished
        print(f'\nthroughput exPBS = {throughput_exPBS}')

        original_pbs = prefix_dir + 'driver --map '+map_fname+' --agents ' + agent_fname_expbs + \
                       ' --agentNum ' + str(number_of_agents) + \
                       ' --output '+output_fname+' --to_save_P_matrix 1 --windowed_mapf ' + str(w_PBS) + ' --frequency_mapd ' + str(h_PBS)
        # print(original_pbs)
        # start = time.time()
        check_collisions(agents_fname_=agent_fname_expbs)
        status = subprocess.call(original_pbs, shell=True)
        # accumulated += (time.time()-start)
        steps_done += h_PBS  #
        # after PBS we need to save priorities into EDB (_.database)
        # we solve it by inserting only PBS agents file which will be chosen because its the only one
        # input(f'pbs... check {agent_fname_expbs}')
        for _ in range(J):  # run 3 exPBS after 1 PBS
            if steps_done >= total_step_limit:
                break
            widths = [-1, 10]
            fallbacks = [0, 0.75, 1, -0.75, -1, 2, 3]  # 0 / (0,1] / 2 / 3
            fallback_dict = {0: "None", 0.75: "3/4 tail P_d", 1: "end tail P_d",
                             -0.75: "3/4 tail P_d-P_exp", -1: "end tail P_d-P_exp",
                             2: "P_exp1 -> Original PBS", 3: "P_exp1 -> P_exp2 -> Original PBS"}

            # # read agents start and goal locations:
            # _, agent_start_dict, agent_goal_dict = read_agents_fname(agents_fname=agent_fname_expbs)
            print(f'\n\trun number = {int(steps_done)} / {int(total_step_limit)},\t\t now running exPBS')
            # update goals for agents which reached the their goals
            # input(f'check if missions fininsed in {agent_fname_expbs}')
            missions_finished = assign_new_missions(agent_fname_=agent_fname_expbs,  mew_missions=new_mission_list_expbs)
            throughput_exPBS += missions_finished
            print(f'\nthroughput exPBS = {throughput_exPBS}')
            pbs_with_experience = prefix_dir + "driver --map "+map_fname+ \
                                " --agents " + agent_fname_expbs + \
                              " --agentNum " + str(number_of_agents) + " --width_limit_hl 10 --fallback 2 --output " + output_fname + \
                              " --experience 1  --cleaning_threshold -1 --to_save_P_matrix 0 --windowed_mapf " + str(w_exPBS) + ' --frequency_mapd ' + str(h_exPBS)
            # print(pbs_with_experience)
            # start = time.time()
            check_collisions(agents_fname_=agent_fname_expbs)
            status = subprocess.call(pbs_with_experience, shell=True)
            # accumulated += (time.time()-start)
            steps_done += h_exPBS



# os.system(f'play -nq -t alsa synth {1/2} sine {1000}')


with open(output_fname, 'a') as f:
    percent_improve = "+inf"
    if throughput_PBS != 0:
        percent_improve = 100*(throughput_exPBS-throughput_PBS)/throughput_PBS
    f.write(f'\n\nthroughput PBS =,{throughput_PBS}\n'
            f'throughput exPBS =,{throughput_exPBS}, improve =,{percent_improve}\n\n'
            )

print(f'throughput PBS = {throughput_PBS}')
print(f'throughput exPBS = {throughput_exPBS}')
print(f'difference = {percent_improve}')


f.close()
os.system(f'play -nq -t alsa synth {1/10} sine {1000}')
