import pandas as pd
import os
import matplotlib.pyplot as plt
import argparse
import numpy as np

if __name__ == '__main__':
    # Parse input arguments
    parser = argparse.ArgumentParser(description='Get script arguments.')
    parser.add_argument('--csv_file', type=str, default='/media/mattias/2TB/radardata/TBV_Eval/candidates_mulran_eval_2022-10-04_10-57/job_0/loop/loop.csv', help='csv file input')
    parser.add_argument('--max_distance', type=float, default=6, help='max distance for loop')
    args = parser.parse_args()

    if(not os.path.isfile(args.csv_file)):
        print("No such csv file!")
        exit()

    print("Loading file: " + args.csv_file)
    df_full = pd.read_csv(args.csv_file, sep=r',', skipinitialspace=True)
    datapoints = len(df_full.index)
    print("Loaded: " + str(datapoints))

    df_full['max_distance'] = df_full.apply (lambda row: args.max_distance, axis=1)
    df_full['is loop'] = df_full.apply (lambda row: int(row["closest_loop_distance"]<row["max_distance"]), axis=1)
    df_full['candidate close'] = df_full.apply (lambda row: row["candidate_loop_distance"]<=row["max_distance"], axis=1)
    df_full=df_full[df_full["is loop"] == 1]

    candidates = len(df_full['guess_nr'].unique().tolist())

    id_from = df_full['id_from'].unique().tolist()

    df_id = pd.DataFrame({"id_from":id_from})
    df_id.set_index(("id_from"), inplace=True)
    no_gt_id = []
    for id in id_from:
        df_temp = df_full[(df_full["candidate close"] == 1) & (df_full['id_from'] == id)]

        # If gt existed for all guess_nr
        if (df_temp['candidate_loop_distance'] == -1).values.sum() == 0:
            guess_nr = df_temp["guess_nr"].values

            min_candidates = candidates
            if len(guess_nr) != 0:
                min_candidates = min(guess_nr)
            df_id.at[id, "N"] = min_candidates
        else:
            no_gt_id.append(id)

    df_id.drop(no_gt_id, inplace=True)

    candidate_data = np.zeros(candidates)
    for candidate in range(candidates):
        candidate_data[candidate] = (df_id["N"] <= candidate).values.sum()

    print("Tot loops: ", df_id.size)
    print("Correct loops: ", candidate_data)
    x = np.arange(0,candidates)
    y = candidate_data / df_id.size
    plt.plot(x,y,linewidth=5,marker='.',markersize=20)
    plt.ylabel('Accuracy %', labelpad=0)
    plt.xlabel('N', labelpad=0)
    plt.ylim([np.amin(y) - np.amin(y)*0.02, 1])
    plt.xticks(x)
    plt.grid(axis='y')
    plt.show()