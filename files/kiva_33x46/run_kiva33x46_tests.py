import copy
import csv
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



"""
    run this code using the line:
    python3 ./files/kiva_33x46/run_kiva33x46_tests.py
"""


# run tests:
number_of_queries = 100
number_of_agents = '180'
folder_name = './files/kiva_33x46/'
map_fname = 'kiva_33x46.map'
output_fname = 'Output_kiva33x46_'+number_of_agents+'agents.csv'

widths = [-1, 10]
fallbacks = [0, 0.75, 1, -0.75, -1, 2, 3]  # 0 / (0,1] / 2 / 3
fallback_dict = {0: "None", 0.75: "3/4 tail P_d", 1: "end tail P_d",
                 -0.75: "3/4 tail P_d-P_exp", -1: "end tail P_d-P_exp",
                 2: "P_exp1 -> Original PBS", 3: "P_exp1 -> P_exp2 -> Original PBS"}
solvers = ["PBS", "PBS+Experience", "PBS+Experience+Clean"]

import click

if not click.confirm(f'Inputs:\n'
                     f'    map = {folder_name + map_fname}\n'
                     f'    number of agents = {number_of_agents}\n'
                     f'    number of queries = {number_of_queries}\n\n'
                     f'Do you want to continue?', default=False):
    raise ValueError

waitbar = tqdm(total=len(widths)*len(fallbacks)*len(solvers)*number_of_queries, position=0, leave=True)

# output CSV title:
with open(output_fname, 'a') as f:
    f.write(f'SearchRuntime,BuildDatabaseRuntime,FindNNRuntime,Agentsfile,Experience,NearestNeighborIndex,'
            f'NearestNeighborDistance,ConstraintsMeasure,HLexpanded,HLgenerated,LLexpanded,LLgenerated,Cost,SolDepth,'
            f'WidthLimit,Fallback\n')
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
                agents_fname = number_of_agents+'agents/kiva_33x46map-'+number_of_agents+'agents-t' + str(a) + '.agents'

                if solver == "PBS":
                    original_pbs = "./driver --map " + folder_name+map_fname + " --agents " + folder_name+agents_fname + \
                                   " --agentNum " + number_of_agents + " --output " + output_fname+ " --fallback "+ \
                                   str(fallback)+" --width_limit_hl " + str(width)

                    print(original_pbs)
                    status = subprocess.call(original_pbs, shell=True)

                if solver == "PBS+Experience":
                    pbs_with_experience = "./driver --map " + folder_name+map_fname + " --agents " + folder_name+agents_fname + \
                                          " --agentNum " + number_of_agents + " --output " + output_fname + \
                                          " --experience " + "1" + " --fallback "+ str(fallback)+ \
                                          " --width_limit_hl " + str(width) + " --cleaning_threshold -1"

                    print(pbs_with_experience)
                    status = subprocess.call(pbs_with_experience, shell=True)

                if solver == "PBS+Experience+Clean":
                    pbs_with_experience_with_clean = "./driver --map " + folder_name+map_fname + " --agents " + \
                                                     folder_name+agents_fname + " --agentNum " + number_of_agents + " --output " + \
                                                     output_fname + " --experience " + "2" +" --fallback "+ str(fallback)+ \
                                                     " --width_limit_hl " + str(width) +" --cleaning_threshold -2"  # --cleaning_threshold = -2 means clean by 1*average distance, cleaning_threshold = -1 means do not clean

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
    agents_fname = number_of_agents+'agents/kiva_33x46map-'+number_of_agents+'agents-t' + str(a) + '.agents'

    original_pbs = "./driver --map " + folder_name+map_fname + " --agents " + folder_name+agents_fname + \
                   " --agentNum " + number_of_agents + " --output " + output_fname
    print("Rerun original PBS")
    print(original_pbs)
    status = subprocess.call(original_pbs, shell=True)

hours, rem = divmod(time.time()-total_start_time, 3600)
minutes, seconds = divmod(rem, 60)
print("\n\n ===== Total time elapsed: {:0>2}:{:0>2}:{:05.2f} =====\n".format(int(hours), int(minutes), seconds))



"""


start_time = time.time()
for a in range(0, number_of_queries, 1):  # number of numbered agents file to solve

    agents_fname = '100agents/kiva_33x46map-100agents-t' + str(a) + '.agents'
    original_pbs = "./driver --map " + folder_name+map_fname + " --agents " + folder_name+agents_fname + \
                   " --agentNum " + number_of_agents + " --output " + output_fname+ " --fallback "+ \
                   str(fallback)+" --width_limit_hl " + str(width)

    pbs_with_experience = "./driver --map " + folder_name+map_fname + " --agents " + folder_name+agents_fname + \
                          " --agentNum " + number_of_agents + " --output " + output_fname + \
                          " --experience " + "1" + " --fallback "+ str(fallback)+ \
                          " --width_limit_hl " + str(width) + " --cleaning_threshold -1"  # --cleaning_threshold = -2 means clean by 3*average distance, cleaning_threshold = -1 means do not clean
    pbs_with_experience_with_clean = "./driver --map " + folder_name+map_fname + " --agents " + \
                                     folder_name+agents_fname + " --agentNum " + number_of_agents + " --output " + \
                                     output_fname + " --experience " + "2" +" --fallback "+ str(fallback)+ \
                                     " --width_limit_hl " + str(width) +" --cleaning_threshold -2"  # --cleaning_threshold = -2 means clean by 1*average distance, cleaning_threshold = -1 means do not clean

    print(original_pbs)
    status = subprocess.call(original_pbs, shell=True)
    # print(pbs_with_experience)
    # status = subprocess.call(pbs_with_experience, shell=True)
    # print(pbs_with_experience_with_clean)
    # status = subprocess.call(pbs_with_experience_with_clean, shell=True)

    waitbar.update(1)


waitbar.close()
hours, rem = divmod(time.time()-start_time, 3600)
minutes, seconds = divmod(rem, 60)
print("\n\n ===== Time elapsed during this batch: {:0>2}:{:0>2}:{:05.2f} =====\n".format(int(hours),int(minutes),seconds))

with open(output_fname, 'a') as f:
    f.write('\n\n')
    # writer.writerow([])
"""