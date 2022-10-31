from matplotlib.pyplot import title
from sklearn.metrics import precision_recall_curve
import pandas as pd
import os
import math
import argparse
import numpy as np
from itertools import product
import pathlib
from LoopClassifier import *
from re import match

def LoadData(base_dir):
    dfs = []
    print("opening the following files:")
    for job in os.listdir(base_dir):
        if match(r"^job_[0-9]+", job):
            print("Loading data: ", job)
            df = pd.read_csv(base_dir+'/'+job+"/loop/loop.csv", index_col=0, header=0, skipinitialspace=True)
            dfs.append(df)
    return pd.concat(dfs, axis=0, ignore_index=True)

def GetNameFromSettings(feature_cols, guess0=True, radar_raw=1, augment=0, odometry_coupled=1, desc_function="sum", no_point=0, sequence="", desc_divider=1):
    if feature_cols == ['sc-sim'] and guess0 and odometry_coupled == 0 and radar_raw == 1 and augment == 0:
        return "1) Radar Scan Context"

    elif feature_cols == ['sc-sim'] and guess0 and odometry_coupled == 0 and radar_raw == 0 and augment == 0:
        return "2) Aggregated point cloud map"

    elif feature_cols == ['sc-sim'] and guess0 and odometry_coupled == 0 and radar_raw == 0 and augment == 1:
        return "3) Origin augmentation"

    elif feature_cols == ['sc-sim', 'alignment_quality'] and guess0 and odometry_coupled == 0 and radar_raw == 0 and augment == 1:
        return "4) Alignment loop verification"

    elif feature_cols == ['odom-bounds', 'sc-sim', 'alignment_quality'] and guess0 and odometry_coupled == 0 and radar_raw == 0 and augment == 1:
        return "5) Odometry decoupled"

    elif feature_cols == ['odom-bounds', 'sc-sim', 'alignment_quality'] and guess0 and odometry_coupled == 1 and radar_raw == 0 and augment == 1:
        return "6) Odometry coupled"

    elif feature_cols == ['odom-bounds', 'sc-sim', 'alignment_quality'] and not guess0 and odometry_coupled == 1 and radar_raw == 0 and augment == 1:
        return "7) Multiple candidate selection"

    else:
        return ''

if __name__ == '__main__':
    # Parse input arguments
    parser = argparse.ArgumentParser(description='Get script arguments.')
    parser.add_argument('--dir', type=str, required=True, help='experiment dir')
    parser.add_argument('--output', type=str, default='', help='output folder')
    parser.add_argument('--p-threshold', type=float, default=0.9, help='th for prediction')
    parser.add_argument('--max_distance', type=float, default=6, help='max distance for loop')
    parser.add_argument('--max_registration_distance', type=float, default=4, help='max registration distance for loop')
    parser.add_argument('--max_registration_rotation', type=float, default=2.5, help='max registration rotation for loop')
    parser.add_argument('--full_path', type=str, default="False", choices=["True", "False"],help="If dir is full path, or in TBV_Eval")
    parser.add_argument('--sequences', type=str, default="False", choices=["True", "False"],help="If dir is full path, or in TBV_Eval")
    args = parser.parse_args()

    base_dir = args.dir if args.full_path == 'True' else os.environ["BAG_LOCATION"] + "/TBV_Eval/" + args.dir
    out_dir = args.output + "/output/loop_closure_eval/" if args.output != '' else base_dir + "/output/loop_closure_eval/"
    dataset = pd.read_csv(base_dir+"/job_0/pars.txt", index_col=0, header=0, skipinitialspace=True).T["dataset"].values[0]
    plot_sequences = True if args.sequences == 'True' else False

    pathlib.Path(out_dir).mkdir(parents=True, exist_ok=True)
    print("Output dir:", out_dir)

    if (not pathlib.Path(base_dir)) or (not pathlib.Path(out_dir)):
        print(base_dir, "or", out_dir, "doesnt exist!")
        exit()

    pthreshold = args.p_threshold

    # Read experiment csv file
    df_full = LoadData(base_dir)

    stats_header = ['Name', 'Settings', 'Training accuracy[%]', 'Training precision [%]', 'Training recall [%]', 'Testing accurac [%]', 'Testing precision [%]', 'Testing recall [%]', 'nr correct candidates', 'nr loops', 'correct_loop_ratio [%]', 'Coef', 'Intercept', 'Threshold', 'max_distance', 'max_registration_distance', 'max_registration_rotation']
    df_stats = pd.DataFrame(columns=stats_header)

    dataset = pd.read_csv(base_dir+"/job_0/pars.txt", index_col=0, header=0, skipinitialspace=True).T["dataset"].values[0]

    datapoints = len(df_full.index)
    print("Loaded: " + str(datapoints))

    df_full['max_distance'] = df_full.apply (lambda row: args.max_distance, axis=1)
    df_full['is loop'] = df_full.apply (lambda row: int(row["closest_loop_distance"]<row["max_distance"]), axis=1)
    df_full['candidate close'] = df_full.apply (lambda row:( math.sqrt(row["diff.x"]*row["diff.x"] + row["diff.y"]*row["diff.y"]) < args.max_registration_distance) & (math.fabs(row["diff.z"]) < args.max_registration_rotation*math.pi/180.0), axis=1)
    df_full['candidate transl_error'] = df_full.apply (lambda row: math.sqrt(row["diff.x"]*row["diff.x"] + row["diff.y"]*row["diff.y"]), axis=1)
    df_full['candidate rot_error'] = df_full.apply (lambda row: 180.0/math.pi*math.fabs(row["diff.z"]), axis=1)
    df_full['prediction pos ok'] = df_full.apply (lambda row: int(not row["is loop"] or row["candidate close"]), axis=1) #Please verify this logic. only false if is loop while candidate is far away
    df_full = df_full[(df_full['guess_nr'] >= 0 )]

    if dataset == "oxford":
        sequences = ["2019-01-10-12-32-52-radar-oxford-10k","2019-01-16-13-09-37-radar-oxford-10k","2019-01-17-13-26-39-radar-oxford-10k","2019-01-18-14-14-42-radar-oxford-10k","2019-01-18-15-20-12-radar-oxford-10k","2019-01-16-11-53-11-radar-oxford-10k","2019-01-10-11-46-21-radar-oxford-10k","2019-01-18-14-46-59-radar-oxford-10k"]
    else: 
        sequences = ["DCC01","DCC02","DCC03","KAIST01","KAIST02","KAIST03","Riverside01","Riverside02","Riverside03"]

    # Combinaiton parameters
    guess_nr = [True,False]
    odometry_coupled = df_full["SC - odometry_coupled_closure"].unique()
    raw_scan_context = df_full["Scan Context - raw_scan_context"].unique()
    augmentations = df_full["SC - augment_sc"].unique()
    no_point = df_full["SC - no_point"].unique()
    desc_function = df_full["SC - desc_function"].unique()
    desc_divider = df_full["SC - desc_divider"].unique()
    feature_cols_all = [['sc-sim'], ['odom-bounds', 'sc-sim'], ['sc-sim', 'alignment_quality'], ['odom-bounds', 'sc-sim', 'alignment_quality']]

    all_setings = [guess_nr, raw_scan_context, odometry_coupled, augmentations, feature_cols_all, no_point, desc_function, desc_divider]
    if plot_sequences:
        all_setings.append(sequences)
        
    settings_combinations = product(*all_setings)

    # Loop over alla selected combinations
    for settings in settings_combinations:
        if plot_sequences:
            guess0, radar_raw, odometry_coupled, augmentations, feature_cols, no_point, desc_function, desc_divider, sequence = settings
        else:
            guess0, radar_raw, odometry_coupled, augmentations, feature_cols, no_point, desc_function, desc_divider = settings
        # See if current parameter combination is of interest
        name = GetNameFromSettings(feature_cols, guess0=guess0, radar_raw=radar_raw, odometry_coupled=odometry_coupled, augment=augmentations, no_point=no_point, desc_function=desc_function, desc_divider=desc_divider)
        if name != '':
            if plot_sequences:
                data_dir = "//media/daniel/m2_ssd/BAG_LOCATION/TBV_Eval/" + dataset + "/" + sequence + "/radar"
                df = df_full[(df_full["SC - odometry_coupled_closure"] == odometry_coupled) & (df_full["Scan Context - raw_scan_context"] == radar_raw) & (df_full["SC - augment_sc"] == augmentations) & (df_full["SC - no_point"] == no_point) & (df_full["SC - desc_function"] == desc_function) & (df_full["SC - desc_divider"] == desc_divider) & (df_full["data_dir"] == data_dir)]
            else:
                df = df_full[(df_full["SC - odometry_coupled_closure"] == odometry_coupled) & (df_full["Scan Context - raw_scan_context"] == radar_raw) & (df_full["SC - augment_sc"] == augmentations) & (df_full["SC - no_point"] == no_point) & (df_full["SC - desc_function"] == desc_function) & (df_full["SC - desc_divider"] == desc_divider)]
            df = df.reset_index()
            df_filtered=df[ (df['guess_nr'] == 0 ) ]

            # Train classifier
            df_train = df_filtered[(df_filtered['prediction pos ok']==True) & (df_filtered['id_from']!=df_filtered['id_to']) ]
            X_train = df_train[feature_cols].to_numpy()
            y_train = df_train['is loop'].to_numpy()
            y_train_pos_ok = df_train['candidate close'].to_numpy()
            logreg, y_train_pred = TrainClassifier(X_train,y_train) #Train classifier on subset of data to avoid position errors
            train_acc, train_precision, train_recall, tn, fp, fn, tp, cnf_matrix_train, _ = ComputeClassifierStatistics(y_train, y_train_pred, y_train_pos_ok) #extract statistics

            # Update df_filtered using best probabilites
            if not guess0:
                nr_guess = df['guess_nr'].to_numpy().max() + 1
                X_test_all = df[feature_cols].to_numpy()
                y_test_prob_all = Predict_proba(logreg, X_test_all) #Predict on all datapoints
                df["y_test_prob"] = y_test_prob_all
                M_pred = y_test_prob_all.reshape(-1, nr_guess) #reshape into matrix
                argmax = np.argmax(M_pred, axis=1, keepdims=False) #highest probability
                max_idx = np.array(range(0, df.shape[0], nr_guess)) + argmax #index for df for highest probability
                df_filtered=df.loc[max_idx.tolist()] #create new df for highest probability candidates
            df_filtered.reset_index()

            nr_correct_candidates = df_filtered['candidate close'].sum()
            nr_loops = df_filtered['is loop'].sum()
            correct_loop_ratio = nr_correct_candidates/nr_loops

            X_test = df_filtered[feature_cols].to_numpy()
            y_test = df_filtered['is loop'].to_numpy()
            y_test_pos_ok = df_filtered['candidate close'].to_numpy()
            y_test_pred = Predict(logreg, X_test, pthreshold) #Predict on all datapoints

            acc_test, test_precision, test_recall, tn, fp_test, fn, tp, cnf_matrix_test, y_test = ComputeClassifierStatistics(y_test, y_test_pred, y_test_pos_ok) #extract statistics and fix "y_test"

            df_stats.loc[len(df_stats)] = [name[0:2], name, train_acc*100, train_precision*100, train_recall*100, acc_test*100, test_precision*100, test_recall*100, nr_correct_candidates, nr_loops, correct_loop_ratio*100, logreg.coef_, logreg.intercept_, pthreshold, args.max_distance, args.max_registration_distance, args.max_registration_rotation]

            y = df_filtered['is loop'].to_numpy()
            y_prob = logreg.predict_proba(X_test)[:,1] * df_filtered['prediction pos ok'].values
            fpr, tpr, _ = metrics.roc_curve(y, y_prob)
            tpr[-1] = tpr[-2]
            roc_auc = metrics.auc(fpr, tpr)

            if plot_sequences:
                plt.figure(sequence)
                plt.plot(fpr, tpr, label = name, linewidth=1)
            else:
                plt.figure(0)
                plt.plot(fpr, tpr, label = name, linewidth=0.5)

    if plot_sequences:
        for sequence in sequences:
            # ROC CURVE
            plt.figure(sequence)
            plt.xlim([-0.01, 1])
            plt.ylim([0, 1])
            plt.ylabel('True Positive Rate')
            plt.xlabel('False Positive Rate')
            plt.title(sequence)
            plt.plot([0, 1], [0, 1],'r--', zorder=-1)
            handles, labels = plt.gca().get_legend_handles_labels()
            labels, handles = zip(*sorted(zip(labels, handles), key=lambda t: t[0]))
            plt.legend(handles, labels, loc = 'lower right', fontsize="x-small")
            plt.savefig(out_dir + "/" + sequence + "_ROC.pdf", bbox_inches='tight', format='pdf')
    else:
        # ROC CURVE
        plt.figure(0)
        plt.xlim([-0.01, 1])
        plt.ylim([0, 1])
        plt.ylabel('True Positive Rate')
        plt.xlabel('False Positive Rate')
        plt.title(dataset)
        plt.plot([0, 1], [0, 1],'r--', zorder=-1)
        handles, labels = plt.gca().get_legend_handles_labels()
        labels, handles = zip(*sorted(zip(labels, handles), key=lambda t: t[0]))
        plt.legend(handles, labels, loc = 'lower right', fontsize="x-small")
        plt.savefig(out_dir + "/ROC.pdf", bbox_inches='tight', format='pdf')