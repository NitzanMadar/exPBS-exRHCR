import joblib
# import time
# from tqdm import tqdm
# import csv
import numpy as np
import pandas as pd
# import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler
# from sklearn.svm import LinearSVC
# from sklearn.svm import SVC
# from sklearn.ensemble import BaggingClassifier, RandomForestClassifier
# from sklearn.multiclass import OneVsRestClassifier
# import unittest
# import os.path
# from os import path
# from sklearn.metrics import plot_confusion_matrix
# from sklearn.metrics import precision_score
import subprocess

# ============= functions =============
def read_agents_fname(agents_fname):
    l = []
    with open(agents_fname, 'r') as f:
        next(f)  # skip first line, which contains the number of agents
        for line in f:
            for num in line.split(','):
                if num != '\n':
                    l.append(int(num))  # Convert to int and append

    agents_query_numpy = np.array(l)
    # print(agents_query_numpy)

    return agents_query_numpy

# ============= end functions =============

# ============= main =============
number_of_agents = '100'
folder_name = 'files/10obs-20x20map-'+number_of_agents+'agents/'
map_fname = folder_name + '10obs-20x20.map'
map_row = 20
map_col = 20
map_avg_size = 0.5*(map_row+map_col)
classifier_fname = 'classifier/random_forest_clf_manual_standartization_wrt_original.pkl'
classifier_fname = 'classifier/random_forest_clf_manual_standartization_wrt_top_20.pkl'
EDB_df = 'classifier/1000_EDB_coords_constraints.csv'
number_of_queries = 10
output_fname = 'Output_classifier.csv'

# Load classifier:
random_forest_clf = joblib.load(folder_name + classifier_fname)  # TODO - handle the warning Trying to unpickle estimator RandomForestClassifier from version 0.22.2.post1 when using version 0.24.1. This might lead to breaking code or invalid results. Use at your own risk.
# print (random_forest_clf)
random_forest_clf.verbose = False

# Load EDB (experience database):
df = pd.read_csv(folder_name + EDB_df).to_numpy()

# coords_standrard = df[:, :-1].copy()
scaler = StandardScaler(with_mean=True, with_std=False)
scaler.fit_transform(df)
# print(scaler_coords.scale_)


EDB_size = len(df)  # =1000

for a in range(number_of_queries):
    # Read .agents file:
    agents_fname = folder_name + "10obs-20x20map-"+number_of_agents+"agents-t" +str(a+1000) + ".agents"

    agents_q = read_agents_fname(agents_fname=agents_fname)
    agents_q = agents_q - map_avg_size
    agents_q_duplicate = np.array([agents_q]*EDB_size)

    # Create X matrix for classifier:
    # [ [q_tilde, q_EDB_1],
    #      ...,
    #   [q_tilde, q_EDB_N] ]   - size: EDB_size x 2*4*number_of_agent + 1 (2 queries, 4 coordinates for each query, and constraint meseaure (+1))
    X = np.array(np.concatenate((agents_q_duplicate, df), axis=1))
    # Standartization (This won't affect the q_tilde because it the same for each row - no distribution)
    # scaler = StandardScaler(with_mean=False, with_std=True)
    # X = scaler_coords.fit_transform(X)
    # Predict - with probability
    y_hat = random_forest_clf.predict_proba(X)
    # Choose the max probability
    choosed_experience = np.argmax(y_hat[:,0])

    pbs_with_experience = "./driver --map " + map_fname + " --agents " + agents_fname + \
                          " --agentNum " + number_of_agents + " --output " + output_fname + \
                          " --experience 1000 --priorityMatrix "+str(choosed_experience)
    original_pbs = "./driver --map " + map_fname + " --agents " + agents_fname+ \
                   " --agentNum " + number_of_agents + " --output " + output_fname
    print (f'\n ===== {str(a)} / {str(number_of_queries-1)} ({100*(a+1) / number_of_queries}%)===== ')
    print ('===== Running Original PBS ======')
    status = subprocess.call(original_pbs, shell=True)
    print ('\n ===== Running PBS + Experience With Random Forest Classifier ======')
    print(f'Experience chosen: #{choosed_experience}; With probability: {round(y_hat[choosed_experience][0]*100, 2)}%')
    status = subprocess.call(pbs_with_experience, shell=True)

# # print X - visual check
# print(pd.DataFrame(X).head(3))




# ============= end main =============