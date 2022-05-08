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
import fnmatch
import re



# ./driver --map files/5obs-20x20.map --agents files/5obs-20x20map-40agents-5gal.agents --agentNum 40 --solver ICBS --priority 2 --priorityMatrix files/5obs-20x20map-40agents-5.priorities --output Output.csv
# ./driver --map files/kiva/kiva_0.map --agents files/kiva/kiva_0map-50agents-0.agents --agentNum 50 --solver CBSH --priority 2 --output Output.csv --experience 0

"""
    run this code using the line:
    python scripts/build_experience_database.py
"""
def atoi(text):
    return int(text) if text.isdigit() else text

def natural_keys(text):
    '''
    alist.sort(key=natural_keys) sorts in human order
    http://nedbatchelder.com/blog/200712/human_sorting.html
    (See Toothy's implementation in the comments)
    '''
    return [ atoi(c) for c in re.split(r'(\d+)', text) ]

destdir = "/home/Dropbox/MAPF/PBS_Experience"    # my working dir
#////////// Inputs: //////////#
# TODO update the correct folder name and agent file name template
agents_num = 600
# folder_name = 'files/10obs-20x20map-100agents/'
folder_name = 'files/10obs-20x20/'+str(agents_num)+'agents/'
folder_name = 'files/kiva_33x46/'+str(agents_num)+'agents/'
folder_name = 'files/sorting_37x77/'+str(agents_num)+'agents/'
agents_test_fname = '10obs-20x20map-'+str(agents_num)+'agents-t*.agents'  # square map 20x20, 100 agents, 10% obstacles
agents_test_fname = 'kiva_33x46map-'+str(agents_num)+'agents-t*.agents'
agents_test_fname = 'sorting_37x77map-'+str(agents_num)+'agents-t*.agents'
agents_test_fname = 'sorting_37x77map-'+str(agents_num)+'agents-*_*.agents'
agents_fname = '10obs-20x20map-'+str(agents_num)+'agents-*.agents'
agents_fname = 'kiva_33x46map-'+str(agents_num)+'agents-*.agents'     # kiva-like domain, 50 agents, spread near to the packing stations (right and left)
agents_fname = 'sorting_37x77map-'+str(agents_num)+'agents-*.agents'
# //////////////////////////// #

priorities_file_names = []
# print(os.listdir('./files'))

for filename in os.listdir('./'+folder_name):
    # print(filename)
    #todo update the t position (test queries) in the next if, [25] in 10obs-20x20... and [18] in kiva_0map-50agents-t
    if fnmatch.fnmatch(filename, agents_fname) and not fnmatch.fnmatch(filename, agents_test_fname):
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
# database_name = folder_name + agents_fname[:-9]+"testset.database"
database_name = folder_name + agents_fname[:-9]+".database"
# print(database_name)
data_file = open('./'+database_name, "w+")

for fname in priorities_file_names:  # [0:1000] to build smaller EDB
    # print(folder_name + fname[:-7] + ".priorities"+", ")
    data_file.write(folder_name + fname[:-7] + ".priorities"+", ")
    with open('./'+folder_name + fname, 'r') as f:
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
# Each row contain [prioritirs_file_name, S_x1, S_y1, G_x1, G_y1, ..., S_xn, S_yn, G_xn, G_yn] of agents file (query)

# print("PAY ATTETION: \n "
#       "this script used all of the files that look like \' " + agents_fname + "\' from: \'"+folder_name+
#       "\' to build the database. Please separate from test agents files.")