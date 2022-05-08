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
# EDB_size = 5000
number_of_queries = 100
map_fname = '10obs-20x20'#'kiva_33x46' #'sorting_37x77'#'kiva_33x46'  #
folder_name = './files/' + map_fname + '/'
output_fname = 'Output_'+map_fname+'_'+number_of_agents+'agents_BLDFS.csv'
map_fname = map_fname + '.map'


if not click.confirm(f'Inputs:\n'
                     f'    map = {folder_name + map_fname}\n'
                     f'    number of agents = {number_of_agents}\n'
                     f'    number of queries = {number_of_queries}\n'
                     f'    agents file name = {number_of_agents+"agents/"+map_fname[:-4]+"map-"+number_of_agents+"agents-t<i>.agents"}\n'
                     f'     \n\n'
                     f'Do you want to continue?', default=False):
    raise ValueError


# output CSV title:
with open(output_fname, 'a') as f:
    f.write(f'SearchRuntime,BuildDatabaseRuntime,FindNNRuntime,Agentsfile,Experience,NearestNeighborIndex,'
            f'NearestNeighborDistance,ConstraintsMeasure,HLexpanded,HLgenerated,LLexpanded,LLgenerated,Cost,SolDepth,'
            f'WidthLimit,Fallback')
f.close()

total_start_time = time.time()


start_time = time.time()

for a in range(0, number_of_queries, 1):  # number of numbered agents file to solve
    agents_fname = number_of_agents+'agents/'+map_fname[:-4]+'map-'+number_of_agents+'agents-t' + str(a) + '.agents'

    original_pbs = "./driver --map " + folder_name+map_fname + " --agents " + folder_name+agents_fname + \
                   " --agentNum " + number_of_agents + " --output " + output_fname

    print(original_pbs)
    status = subprocess.call(original_pbs, shell=True)

with open(output_fname, 'a') as f:
    f.write(f'\n\nPBS+BLDFS\n')
f.close()

for a in range(0, number_of_queries, 1):  # number of numbered agents file to solve
    # if a not in [18,20,21,23,27,29,41,64,77]:
    #     continue
    agents_fname = number_of_agents+'agents/'+map_fname[:-4]+'map-'+number_of_agents+'agents-t' + str(a) + '.agents'

    pbs_with_experience = "./driver --map "+folder_name+map_fname+ \
                          " --agents " + folder_name+agents_fname + \
                          " --agentNum " + str(number_of_agents) + " --width_limit_hl 5 --fallback 1 --output " + output_fname + \
                          " --experience 0  --cleaning_threshold -1 --to_save_P_matrix 0"
    print(pbs_with_experience)
    status = subprocess.call(pbs_with_experience, shell=True)


hours, rem = divmod(time.time()-start_time, 3600)
minutes, seconds = divmod(rem, 60)
print("\n\n ===== Time elapsed during this batch: {:0>2}:{:0>2}:{:05.2f} =====\n".format(int(hours), int(minutes), seconds))


