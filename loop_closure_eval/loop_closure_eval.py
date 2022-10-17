from matplotlib.pyplot import title
import pandas as pd
import os
import math
import argparse
import numpy as np
from itertools import product
import seaborn as sns
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

def GetNameFromSettings(feature_cols, guess0=True, radar_raw=1, augment=0, odometry_coupled=1):
    if feature_cols == ['sc-sim'] and guess0 and odometry_coupled == 0 and radar_raw == 1 and augment == 0:
        return "1) SC raw radar"

    elif feature_cols == ['sc-sim'] and guess0 and odometry_coupled == 0 and radar_raw == 0 and augment == 0:
        return "2) SC local map"

    elif feature_cols == ['sc-sim'] and guess0 and odometry_coupled == 0 and radar_raw == 0 and augment == 1:
        return "3) SC local map, augment"

    elif feature_cols == ['sc-sim', 'alignment_quality'] and guess0 and odometry_coupled == 0 and radar_raw == 0 and augment == 1:
        return "4) SC local map, augment, CorAl"        

    elif feature_cols == ['odom-bounds', 'sc-sim', 'alignment_quality'] and guess0 and odometry_coupled == 0 and radar_raw == 0 and augment == 1:
        return "5) SC local map, augment, CorAl, odometry decoupled"

    elif feature_cols == ['odom-bounds', 'sc-sim', 'alignment_quality'] and guess0 and odometry_coupled == 1 and radar_raw == 0 and augment == 1:
        return "6) SC local map, augment, CorAl, odometry coupled"

    elif feature_cols == ['odom-bounds', 'sc-sim', 'alignment_quality'] and not guess0 and odometry_coupled == 1 and radar_raw == 0 and augment == 1:
        return "7) SC local map, augment, CorAl, odometry coupled, guess(0..N)"

    else:
        return ''

if __name__ == '__main__':
    # Parse input arguments
    parser = argparse.ArgumentParser(description='Get script arguments.')
    parser.add_argument('--dir', type=str, required=True, help='experiment dir')
    parser.add_argument('--output', type=str, default='', help='output folder')
    parser.add_argument('--p-threshold', type=float, default=0.8, help='th for prediction')
    parser.add_argument('--max_distance', type=float, default=6, help='max distance for loop')
    parser.add_argument('--max_registration_distance', type=float, default=4, help='max registration distance for loop')
    parser.add_argument('--max_registration_rotation', type=float, default=2.5, help='max registration rotation for loop')
    parser.add_argument('--use_guess0', action='store_true', help='Use only guess 0, otherwise use best probability guess')
    args = parser.parse_args()

    base_dir = os.environ["BAG_LOCATION"] + "/TBV_Eval/" + args.dir
    out_dir = args.output

    if(args.output == ""):
        out_dir = base_dir + "/output/loop_closure_eval/"

    if(not pathlib.Path(base_dir)):
        print(base_dir, "doesnt exist!")
        exit()

    pathlib.Path(out_dir).mkdir(parents=True, exist_ok=True)
    print("created output dir at", out_dir)  

    pthreshold = args.p_threshold

    # Read experiment csv file
    df_full = LoadData(base_dir)

    stats_header = ['Name', 'Settings', 'Training accuracy[%]', 'Training precision [%]', 'Training recall [%]', 'Testing accurac [%]', 'Testing precision [%]', 'Testing recall [%]', 'nr correct candidates', 'nr loops', 'correct_loop_ratio [%]', 'Coef', 'Intercept', 'Threshold', 'max_distance', 'max_registration_distance', 'max_registration_rotation']
    df_stats = pd.DataFrame(columns=stats_header)

    datapoints = len(df_full.index)
    print("Loaded: " + str(datapoints))

    df_full['max_distance'] = df_full.apply (lambda row: args.max_distance, axis=1)
    df_full['is loop'] = df_full.apply (lambda row: int(row["closest_loop_distance"]<row["max_distance"]), axis=1)
    df_full['candidate close'] = df_full.apply (lambda row:( math.sqrt(row["diff.x"]*row["diff.x"] + row["diff.y"]*row["diff.y"]) < args.max_registration_distance) & (math.fabs(row["diff.z"]) < args.max_registration_rotation*math.pi/180.0), axis=1)
    df_full['candidate transl_error'] = df_full.apply (lambda row: math.sqrt(row["diff.x"]*row["diff.x"] + row["diff.y"]*row["diff.y"]), axis=1)
    df_full['candidate rot_error'] = df_full.apply (lambda row: 180.0/math.pi*math.fabs(row["diff.z"]), axis=1)
    df_full['prediction pos ok'] = df_full.apply (lambda row: int(not row["is loop"] or row["candidate close"]), axis=1) #Please verify this logic. only false if is loop while candidate is far away
    df_full = df_full[(df_full['guess_nr'] >= 0 )]

    guess_nr = [True, False]
    odometry_coupled = df_full["SC - odometry_coupled_closure"].unique()
    raw_scan_context = df_full["Scan Context - raw_scan_context"].unique()
    augmentations = df_full["SC - augment_sc"].unique()
    feature_cols_all = [['sc-sim'], ['odom-bounds', 'sc-sim'], ['sc-sim', 'alignment_quality'], ['odom-bounds', 'sc-sim', 'alignment_quality']]

    all_setings = [guess_nr, raw_scan_context, odometry_coupled, augmentations, feature_cols_all]
    settings_combinations = product(*all_setings)

    for settings in settings_combinations:
        guess0, radar_raw, odometry_coupled, augmentations, feature_cols = settings
        name = GetNameFromSettings(feature_cols, guess0=guess0, radar_raw=radar_raw, odometry_coupled=odometry_coupled, augment=augmentations)
        if name != '':
            df = df_full[(df_full["SC - odometry_coupled_closure"] == odometry_coupled) & (df_full["Scan Context - raw_scan_context"] == radar_raw) & (df_full["SC - augment_sc"] == augmentations)]
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
            plt.plot(fpr, tpr, label = name + ' (AUC: = %0.2f)' % roc_auc, linewidth=3)

    plt.xlim([-0.01, 1])
    plt.ylim([0, 1])
    plt.ylabel('True Positive Rate')
    plt.xlabel('False Positive Rate')
    plt.title("ROC")
    plt.plot([0, 1], [0, 1],'r--', zorder=-1)
    handles, labels = plt.gca().get_legend_handles_labels()
    labels, handles = zip(*sorted(zip(labels, handles), key=lambda t: t[0]))
    plt.legend(handles, labels, loc = 'lower right', fontsize="x-small")
    plt.savefig(out_dir + "/ROC.pdf", bbox_inches='tight', format='pdf')

    result_path = os.path.join(out_dir, "result.csv")
    df_stats.sort_values(by=['Name'], inplace=True)
    df_stats.reset_index(drop=True, inplace=True)
    df_stats.to_csv(result_path, index=False)

    plot_parameters = stats_header[2:11]
    sns.set_theme(style="ticks", color_codes=True)
    sns.set(style="ticks")
    for plot_par in plot_parameters:
        g = sns.catplot(x="Name", y=plot_par,
                            data=df_stats, saturation=.5,
                            kind="bar", ci=None, aspect=1)
        g.set_xticklabels(rotation=90)
        file_name = ''.join(e for e in plot_par if e.isalnum())
        plt.savefig(out_dir + "/" + file_name + ".pdf", bbox_inches='tight', format='pdf')