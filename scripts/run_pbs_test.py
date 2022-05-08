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


# run tests:
# number_of_agents = input("Insert Number of agents: ")  # inputs are str by default

parser = argparse.ArgumentParser(description='Test Arguments')
parser.add_argument("-a", "--num_of_agent", help="choose number of agents")
args = parser.parse_args()
number_of_agents = args.num_of_agent

number_of_queries = 100
map_fname = 'sorting_37x77'  # '10obs-20x20'
folder_name = './files/' + map_fname + '/'
output_fname = 'Output_'+map_fname+'_'+number_of_agents+'agents.csv'
map_fname = map_fname + '.map'

widths = [-1, 10]
fallbacks = [0, 0.75, 1, -0.75, -1, 2, 3]  # 0 / (0,1] / 2 / 3
fallback_dict = {0: "None", 0.75: "3/4 tail P_d", 1: "end tail P_d",
                 -0.75: "3/4 tail P_d-P_exp", -1: "end tail P_d-P_exp",
                 2: "P_exp1 -> Original PBS", 3: "P_exp1 -> P_exp2 -> Original PBS"}
solvers = ["PBS", "PBS+Experience", "PBS+Experience+Clean"]

# if not click.confirm(f'Inputs:\n'
#                      f'    map = {folder_name + map_fname}\n'
#                      f'    number of agents = {number_of_agents}\n'
#                      f'    number of queries = {number_of_queries}\n'
#                      f'    agents file name = {number_of_agents+"agents/"+map_fname[:-4]+"map-"+number_of_agents+"agents-t<i>.agents"}\n\n'
#                      f'Do you want to continue?', default=False):
#     raise ValueError

waitbar = tqdm(total=len(widths)*len(fallbacks)*len(solvers)*number_of_queries, position=0, leave=True)

# output CSV title:
with open(output_fname, 'a') as f:
    f.write(f'SearchRuntime,BuildDatabaseRuntime,FindNNRuntime,Agentsfile,Experience,NearestNeighborIndex,'
            f'NearestNeighborDistance,ConstraintsMeasure,HLexpanded,HLgenerated,LLexpanded,LLgenerated,Cost,SolDepth,'
            f'WidthLimit,Fallback')
f.close()

total_start_time = time.time()
for width in widths:
    for fallback in fallbacks:
        for solver in solvers:
            print(solver, fallback, width)
            if (solver == "PBS" and (fallback < 0 or fallback > 1)) or (width <= 1 and fallback != 0) or (width > 0 and fallback == 0):
                # irrelevant combination - original PBS and "other experience" fallback
                # (P_exp1 -> original / P_exp1 -> P_exp2 -> original)
                print("skipping irrelevant combination")
                waitbar.update(number_of_queries)
                continue

            with open(output_fname, 'a') as f:
                f.write(f'\n{solver}, w = {width}, fb = {fallback_dict[fallback]}\n')
            f.close()
            start_time = time.time()

            for a in range(0, number_of_queries, 1):  # number of numbered agents file to solve
                agents_fname = number_of_agents+'agents/'+map_fname[:-4]+'map-'+number_of_agents+'agents-t' + str(a) + '.agents'

                if solver == "PBS":
                    original_pbs = "./driver --map " + folder_name+map_fname + " --agents " + folder_name+agents_fname + \
                                   " --agentNum " + number_of_agents + " --output " + output_fname + " --fallback " + \
                                   str(fallback)+" --width_limit_hl " + str(width)

                    print(original_pbs)
                    status = subprocess.call(original_pbs, shell=True)

                if solver == "PBS+Experience":
                    pbs_with_experience = "./driver --map " + folder_name+map_fname + " --agents " + folder_name+agents_fname + \
                                          " --agentNum " + number_of_agents + " --output " + output_fname + \
                                          " --experience " + "1" + " --fallback " + str(fallback)+ \
                                          " --width_limit_hl " + str(width) + " --cleaning_threshold -1"

                    print(pbs_with_experience)
                    status = subprocess.call(pbs_with_experience, shell=True)

                if solver == "PBS+Experience+Clean":
                    pbs_with_experience_with_clean = "./driver --map " + folder_name+map_fname + " --agents " + \
                                                     folder_name+agents_fname + " --agentNum " + number_of_agents + " --output " + \
                                                     output_fname + " --experience " + "2" + " --fallback " + str(fallback) + \
                                                     " --width_limit_hl " + str(width) + " --cleaning_threshold -2"  # --cleaning_threshold = -2 means clean by 1*average distance, cleaning_threshold = -1 means do not clean

                    print(pbs_with_experience_with_clean)
                    status = subprocess.call(pbs_with_experience_with_clean, shell=True)

                waitbar.update(1)

            hours, rem = divmod(time.time()-start_time, 3600)
            minutes, seconds = divmod(rem, 60)
            print("\n\n ===== Time elapsed during this batch: {:0>2}:{:0>2}:{:05.2f} =====\n".format(int(hours), int(minutes), seconds))

hours, rem = divmod(time.time()-total_start_time, 3600)
minutes, seconds = divmod(rem, 60)
print("\n\n\n =========== Total time elapsed: {:0>2}:{:0>2}:{:05.2f} ===========\n".format(int(hours), int(minutes), seconds))


# rerun original PBS
with open(output_fname, 'a') as f:
    f.write(f'\n\n\nOriginal PBS - rerun\n')
f.close()
start_time = time.time()

for a in range(0, number_of_queries, 1):  # number of numbered agents file to solve
    agents_fname = number_of_agents+'agents/'+map_fname[:-4]+'map-'+number_of_agents+'agents-t' + str(a) + '.agents'

    original_pbs = "./driver --map " + folder_name+map_fname + " --agents " + folder_name+agents_fname + \
                   " --agentNum " + number_of_agents + " --output " + output_fname
    print("Rerun original PBS")
    print(original_pbs)
    status = subprocess.call(original_pbs, shell=True)

hours, rem = divmod(time.time()-total_start_time, 3600)
minutes, seconds = divmod(rem, 60)
print("\n\n ===== Total time elapsed: {:0>2}:{:0>2}:{:05.2f} =====\n".format(int(hours), int(minutes), seconds))











"""
# run tests:
number_of_queries = 100
# experience = '1'
number_of_agents = '100'
waitbar = tqdm(total=number_of_queries, position=0, leave=True)
# agents_prefix = "files/kiva/50agent_random/kiva_0map-50agents-"
# agents_prefix = "files/kiva/50agent_random/kiva_0map-50agents-t"
folder_name = './files/10obs-20x20map-100agents/'
map_fname = '10obs-20x20.map'
output_fname = 'PBS_Exp_fallback_Output.csv'
width = 10
fallback = 0.75  # 0 / (0,1] / 1 / 2
# '''
run_again = [7,9,14,17,18,21,30,31,34,36,46,48,50,70,74,84,86,89,99]
# print(len(run_again))
start_time = time.time()
for a in range(0, number_of_queries, 1):  # number of numbered agents file to solve
    # if a not in run_again:
    #     waitbar.update(1)
    #     continue

    # random choose:
    # a = random.randint(0,1000)

    agents_fname = '10obs-20x20map-100agents-t' + str(a) + '.agents'
    pbs_with_experience = "./driver --map " + folder_name+map_fname + " --agents " + folder_name+agents_fname + \
                          " --agentNum " + number_of_agents + " --output " + output_fname + \
                          " --experience " + "1" + " --fallback "+ str(fallback)+\
                          " --width_limit_hl " + str(width) + " --cleaning_threshold -1"  # --cleaning_threshold = -2 means clean by 3*average distance, cleaning_threshold = -1 means do not clean
    pbs_with_experience_with_clean = "./driver --map " + folder_name+map_fname + " --agents " + \
                                     folder_name+agents_fname + " --agentNum " + number_of_agents + " --output " + \
                                     output_fname + " --experience " + "2" +" --fallback "+ str(fallback)+ \
                                     " --width_limit_hl " + str(width) +" --cleaning_threshold -2"  # --cleaning_threshold = -2 means clean by 1*average distance, cleaning_threshold = -1 means do not clean
    original_pbs = "./driver --map " + folder_name+map_fname + " --agents " + folder_name+agents_fname + \
                   " --agentNum " + number_of_agents + " --output " + output_fname


    # print(original_pbs)
    # status = subprocess.call(original_pbs, shell=True)
    # print(pbs_with_experience)
    # status = subprocess.call(pbs_with_experience, shell=True)
    print(pbs_with_experience_with_clean)
    status = subprocess.call(pbs_with_experience_with_clean, shell=True)

    # # if run one query and depth visualization in ON, display the HL expanded tree
    # status = subprocess.call(' python3 ./scripts/draw_HL_graph_by_depth.py', shell=True)

    waitbar.update(1)

    # for exp in range(0, experience,1):
    #     s1 = "./driver --map files/10obs-20x20map-50agents/10obs-20x20.map " \
    #          "--agents files/10obs-20x20map-50agents/10obs-20x20map-50agents-t" + str(a) + ".agents" + \
    #          " --agentNum 50 --solver CBSH --priority 2 --output Output.csv --experience 5000"
    #
    #     s2 = "./driver --map files/10obs-20x20map-20agents/10obs-20x20.map " \
    #          "--agents files/10obs-20x20map-100agents/10obs-20x20map-100agents-" + str(a) + ".agents" + \
    #          " --agentNum 20 --solver CBSH --priority 2 --output Output.csv --experience 0"
    #
    #     s3 = "./driver --map files/10obs-20x20map-20agents/10obs-20x20.map " \
    #          "--agents files/10obs-20x20map-20agents/10obs-20x20map-20agents-" + str(a) + ".agents" + \
    #          " --agentNum 20 --solver CBSH --priority 2 --output Output.csv --experience 0"
    #
    #     # # Kiva: #TODO add t before +str(a) for test set
    #     # s1 = "./driver --map files/kiva/0agent_random/kiva_0.map " \
    #     #      "--agents files/kiva/50agent_random/kiva_0map-50agents-t" + str(a) + ".agents" + \
    #     #      " --agentNum 50 --solver CBSH --priority 2 --output Output.csv --experience 0"
    #     # s2 = "./driver --map files/kiva/50agent_random/kiva_0.map " \
    #     #      "--agents files/kiva/50agent_random/kiva_0map-50agents-t" + str(a) + ".agents" + \
    #     #      " --agentNum 50 --solver CBSH --priority 2 --output Output.csv --experience 10000"
    #     s3 = "./driver --map files/10obs-20x20map-100agents/10obs-20x20.map " \
    #          "--agents files/10obs-20x20map-100agents/10obs-20x20map-100agents-t"+str(a)+".agents" \
    #          " --agentNum 100 --solver CBSH --priority 2 --output Output.csv --experience 0 --priorityMatrix -1"
    #
    #     s4 = "./driver --map files/10obs-20x20map-100agents/10obs-20x20.map " \
    #          "--agents files/10obs-20x20map-100agents/10obs-20x20map-100agents-t"+str(a)+".agents" \
    #          " --agentNum 100 --solver CBSH --priority 2 --output Output_for_metric_learning.csv" \
    #          " --experience 5000 --priorityMatrix "+str(exp)
    #
    #     # if not path.isfile(agents_prefix + str(a) + ".priorities"):
    #     #     #no priorities - didn't solved without experience already
    #     #     print(s1)
    #     #     status = subprocess.call(s1, shell=True)
    #     # print(s1)
    #     # status = subprocess.call(s1, shell=True)
    #     # print(s2)
    #     # status = subprocess.call(s2, shell=True)
    #     # print(s3)
    #     # status = subprocess.call(s3, shell=True)
    #     # print(s4)
    #     # status = subprocess.call(s4, shell=True)
    #
    #     waitbar.update(1)


    # waitbar.update(1)
waitbar.close()
hours, rem = divmod(time.time()-start_time, 3600)
minutes, seconds = divmod(rem, 60)
print("\n\n ===== Time elapsed during this batch: {:0>2}:{:0>2}:{:05.2f} =====\n".format(int(hours),int(minutes),seconds))

"""