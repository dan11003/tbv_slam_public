from cProfile import label
import pandas as pd
import os
import sklearn.metrics as metrics
import matplotlib.pyplot as plt
import argparse
import numpy as np
from sklearn.metrics import confusion_matrix
from sklearn.metrics import accuracy_score
from sklearn.linear_model import LogisticRegression
from sklearn import tree

from sklearn.metrics import precision_score
from sklearn.metrics import recall_score
from sklearn.metrics import f1_score
from vis_loops import *
from LoopClassifier import *

pd.options.mode.chained_assignment = None 

if __name__ == '__main__':
    # Parse input arguments
    parser = argparse.ArgumentParser(description='Get script arguments.')
    parser.add_argument('--csv_file', type=str, required=True, help='csv file input')
    parser.add_argument('--disable-output', type=str, default='', help='csv file input')
    parser.add_argument('--p-threshold', type=float, default=0.8, help='th for prediction')
    parser.add_argument('--max_distance', type=float, default=6, help='max distance for loop')
    args = parser.parse_args()
    
    if(not os.path.isfile(args.csv_file)):
        print("No such csv file!")
        exit()

    disable_output = args.disable_output!=''
    pthreshold = args.p_threshold

    if not disable_output:
        print("Loading file: " + args.csv_file)
    # Read experiment csv file
    df_full = pd.read_csv(args.csv_file, sep=r',', skipinitialspace=True)
    datapoints = len(df_full.index)

    if not disable_output:
        print("Loaded: " + str(datapoints))

    df_full['max_distance'] = df_full.apply (lambda row: args.max_distance, axis=1)
    df_full['is loop'] = df_full.apply (lambda row: int(row["closest_loop_distance"]<row["max_distance"]), axis=1)
    df_full['candidate close'] = df_full.apply (lambda row: row["candidate_loop_distance"]<=row["max_distance"], axis=1)
    df_full=df_full[df_full["is loop"] == 1]

    guess_nr = df_full['guess_nr'].unique().tolist()
    guess_nr = sorted(guess_nr)

    nr_correct_candidates_data = []
    nr_loops_data = []
    for guess in guess_nr:
        df=df_full.loc[(df_full['guess_nr']==guess) ]
        nr_correct_candidates = df['candidate close'].sum()
        nr_correct_candidates_data.append(nr_correct_candidates)
        nr_loops = df['is loop'].sum()
        nr_loops_data.append(nr_loops)

    plt.bar(range(len(nr_correct_candidates_data)), nr_loops_data, bottom=nr_loops_data, label="Actual loops")
    plt.bar(range(len(nr_correct_candidates_data)), nr_correct_candidates_data, bottom=nr_loops_data, label="Found loops")
    plt.ylabel('correct candidates', labelpad=0)
    plt.xlabel('guess_nr', labelpad=0)
    plt.legend()
    plt.show()