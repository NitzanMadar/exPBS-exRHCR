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


# ./driver --map files/5obs-20x20.map --agents files/5obs-20x20map-40agents-5gal.agents --agentNum 40 --solver ICBS --priority 2 --priorityMatrix files/5obs-20x20map-40agents-5.priorities --output Output.csv

"""
    run this code using the line:
    python scripts/similar_agents_file_generator.py
"""


def move_agent_manhattan(si, gi, map_si, map_gi, dist):
    global agent_num
    # global map
    # init_row = init_loc[0] # start in 1 not in 0
    # init_col = init_loc[1] # start in 1 not in 0
    # succ = False
    visited = []

    for step_num in range(dist):
        i = random.randint(0,agent_num-1) # random agent
        succ = False
        counter = 0
        visited=[]
        while not succ:
            counter = counter + 1
            if counter > 50: # surrounded agent - can't move
                i = random.randint(0,agent_num-1) # random agent
            my_randi = random.randint(-1,1) # -1 \ 0 \ 1
            Z = [-1,1]
            plusminus = Z[random.randint(0,1)]
            s_or_g = random.randint(0,1) # move in s or g
            s_or_g =1 #g
            s_or_g =0 #s
            if s_or_g == 0: # , si
                new_row = si[i][0] + my_randi
                new_col = si[i][1] + plusminus*( 1 - abs(my_randi))
                if map_si[new_row][new_col] == 0: #no obstacle or other agent
                    # switch positions
                    map_si[new_row][new_row] == 1
                    map_si[si[i][0]][si[i][1]] == 0
                    succ = True
                    # visited.append([si[i][0]-1,si[i][1]])
                    # visited.remove([si[i][0],si[i][1]])

                    si[i][0] = new_row
                    si[i][1] = new_col
                    print ( new_row,new_col, 's')

            else: # , gi
                    new_row = gi[i][0] + my_randi
                    new_col = gi[i][1] + plusminus*( 1 - abs(my_randi))

                    if map_gi[new_row][new_col] == 0: #no obstacle or other agent
                            # switch positions
                            map_gi[new_row][new_row] == 1
                            map_gi[gi[i][0]][gi[i][1]] == 0
                            succ = True
                            # visited.append([si[i][0]-1,si[i][1]])
                            # visited.remove([si[i][0],si[i][1]])
                            gi[i][0] = new_row
                            gi[i][1] = new_col
                            print ( new_row,new_col, 'g')

        # visited

    # # move every agent "dist" steps
    # for i in range(len(si)):# todo - add option when move k manhattan step is possible to move k-1
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

    return si,gi


# plot function
def plot_map(A):
    # plots:
    fig, ax = plt.subplots(nrows=1, ncols=1,figsize=(20,20))

    row = len(A)
    col = len(A[0])
    ax[0].imshow(A, cmap='Greys')#,  interpolation='nearest')
    ax[0].grid(True,which='major', axis='both', linestyle='-',color='gray')
    ax[0].set_xticks(np.arange(-.5,col,1))
    ax[0].set_yticks(np.arange(-.5,row,1))
    plt.setp(ax[0].get_xticklabels(), visible=False)
    plt.setp(ax[0].get_yticklabels(), visible=False)
    ax[0].tick_params(axis='both', which='both', length=0)
    ax[0].set(title='Map')



"""#._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._.#"""


#////////// Inputs: //////////#
steps = int(raw_input("steps input please: ")) # steps to move for each agent (distance 2*Steps*agentsNum(*2 if moving both s and g))
map_fname = 'files/10obs-20x20.map'
agents_fname = 'files/10obs-20x20map-40agents-10.agents'
# dest_fname = 'files/5obs-20x20map-40agents-0-'+str(steps)+'.agents'
dest_fname = 'files/5obs-20x20map-40agents-1000.agents'
# //////////////////////////// #

map=[]
# Read Map file into binar list of lists. the map wrapped map by obstacles
with open(map_fname, 'r') as f:
    for line in f:
        a=[]
        for bit in line:
            if bit == '.' or bit=='0':
                a.append(0)
            if bit == '@' or bit == '1':
                a.append(1)
            else:
                continue
        map.append(a)

map.pop(0) # first element indices the map size
rows = len(map)-2 # wrapped map by obstacles so minus 2
cols = len(map[0])-2 # wrapped map by obstacles, so minus 2


map_si = [x[:] for x in map]
map_gi = [x[:] for x in map]

# Read agents file into
with open(agents_fname, 'r') as f:
    l = [[num for num in line.split(',')] for line in f]
l.pop(0)
si=[]
gi=[]
for line in l:
    si.append([int(line [0]), int(line[1])])
    gi.append([int(line [2]), int(line[3])])
    map_si[int(line [0])][int(line[1])] = 1
    map_gi[int(line [2])][int(line[3])] = 1

agent_num = len(si)
copy_si = [x[:] for x in si]
copy_gi = [x[:] for x in gi]
# print(agent_num)
# print(map)
# print(map_si)
# print(map_gi)

si_tag,gi_tag = move_agent_manhattan(si, gi, map_si, map_gi, steps) #last parameter is how many steps it takes (Manhattan distance)
# gi_tag = move_agent_manhattan(gi,map_gi,Steps) #last parameter is how many steps it takes (Manhattan distance)
# print(copy_si, '->', si_tag)
# print(copy_gi, '->', gi_tag)
# print (si)

# # todo - save the si_tag and gi_tag into new text file
with open(dest_fname, 'w') as f:
    f.write(str(agent_num)+'\n')
    for ai in range(agent_num):
        f.write(str(si_tag[ai][0])+','+str(si_tag[ai][1])+','+
        str(gi_tag[ai][0])+','+str(gi_tag[ai][1])+',\n')
# # todo - add option when move k manhattan step is possible to move k-1

print("Generate file:" + dest_fname)

sys.stdout.flush()


# run tests:
# for m in range(0,50,1): # case
#     for a in range(50,110,10): #agents
#         s1 = "./driver --map files/10obs-20x20.map --agents files/10obs-20x20map-"+str(a)+"agents-"+str(m) + ".agents" + \
#              " --agentNum "+ str(a)+" --solver CBSH --priority 2 --priorityMatrix files/empty-"+str(a)+"agents.priorities --output Output.csv"
#
#         # s2 = "./driver --map files/5obs-20x20.map --agents files/5obs-20x20map-40agents-0-"+str(m) + ".agents" + " --agentNum 40 --solver ICBS --priority 2 --priorityMatrix files/5obs-20x20map-40agents-0.priorities --output Output.csv"
#         # print(m)
#         print(s1)
#         status = subprocess.call(s1, shell=True)
#         # print(s2)
#         # status = subprocess.call(s2, shell=True)
