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

plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['ps.fonttype'] = 42

def LoadData(base_dir):
    dfs = []
    print("opening the following files:")
    for job in os.listdir(base_dir):
        if match(r"^job_[0-9]+", job):
            print("Loading data: ", job)
            df = pd.read_csv(base_dir+'/'+job+"/loop/loop.csv", index_col=0, header=0, skipinitialspace=True)
            dfs.append(df)
    return pd.concat(dfs, axis=0, ignore_index=True)

def GetNameFromSettings(feature_cols, guess0=True, radar_raw=1, augment=0, odometry_coupled=1, cascaded=False):
    if feature_cols == ['sc-sim'] and guess0 and odometry_coupled == 0 and radar_raw == 1 and augment == 0 and cascaded == False:
        return "1) Radar Scan Context"

    elif feature_cols == ['sc-sim'] and guess0 and odometry_coupled == 0 and radar_raw == 0 and augment == 0 and cascaded == False:
        return "2) Aggregated point cloud map"

    elif feature_cols == ['sc-sim'] and guess0 and odometry_coupled == 0 and radar_raw == 0 and augment == 1 and cascaded == False:
        return "3) Origin augmentation"

    elif feature_cols == ['sc-sim', 'alignment_quality'] and guess0 and odometry_coupled == 0 and radar_raw == 0 and augment == 1 and cascaded == False:
        return "4) Alignment loop verification"

    elif feature_cols == ['odom-bounds', 'sc-sim', 'alignment_quality'] and guess0 and odometry_coupled == 0 and radar_raw == 0 and augment == 1 and cascaded == False:
        return "5) Odometry decoupled"

    elif feature_cols == ['odom-bounds', 'sc-sim', 'alignment_quality'] and guess0 and odometry_coupled == 1 and radar_raw == 0 and augment == 1 and cascaded == False:
        return "6) Odometry coupled"

    elif feature_cols == ['odom-bounds', 'sc-sim', 'alignment_quality'] and guess0 and odometry_coupled == 1 and radar_raw == 0 and augment == 1 and cascaded == True:
        return "7) Cascaded classifier"

    elif feature_cols == ['odom-bounds', 'sc-sim', 'alignment_quality'] and not guess0 and odometry_coupled == 1 and radar_raw == 0 and augment == 1 and cascaded == False:
        return "8) Multiple candidate selection"

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
    args = parser.parse_args()

    base_dir = args.dir if args.full_path == 'True' else os.environ["BAG_LOCATION"] + "/TBV_Eval/" + args.dir
    out_dir = args.output + "/output/loop_closure/" if args.output != '' else base_dir + "/output/loop_closure/"
    dataset = pd.read_csv(base_dir+"/job_0/pars.txt", index_col=0, header=0, skipinitialspace=True).T["dataset"].values[0]

    pathlib.Path(out_dir).mkdir(parents=True, exist_ok=True)
    print("Output dir:", out_dir)

    if (not pathlib.Path(base_dir)) or (not pathlib.Path(out_dir)):
        print(base_dir, "or", out_dir, "doesnt exist!")
        exit()

    pthreshold = args.p_threshold

    # Read experiment csv file
    df_full = LoadData(base_dir)

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

    # Combinaiton parameters
    guess_nr = [True, False]
    odometry_coupled = df_full["SC - odometry_coupled_closure"].unique()
    raw_scan_context = df_full["Scan Context - raw_scan_context"].unique()
    augmentations = df_full["SC - augment_sc"].unique()
    feature_cols_all = [['sc-sim'], ['odom-bounds', 'sc-sim'], ['sc-sim', 'alignment_quality'], ['odom-bounds', 'sc-sim', 'alignment_quality']]
    cascaded = [True, False]

    all_setings = [guess_nr, raw_scan_context, odometry_coupled, augmentations, feature_cols_all, cascaded]
    settings_combinations = product(*all_setings)

    # Loop over alla selected combinations
    for settings in settings_combinations:
        guess0, radar_raw, odometry_coupled, augmentations, feature_cols, cascaded = settings
        # See if current parameter combination is of interest
        name = GetNameFromSettings(feature_cols, guess0=guess0, radar_raw=radar_raw, odometry_coupled=odometry_coupled, augment=augmentations, cascaded=cascaded)
        if name != '':
            print(name)
            df = df_full[(df_full["SC - odometry_coupled_closure"] == odometry_coupled) & (df_full["Scan Context - raw_scan_context"] == radar_raw) & (df_full["SC - augment_sc"] == augmentations)]
            df = df.reset_index()
            df_filtered=df[ (df['guess_nr'] == 0 ) ]

            # Train classifier
            df_train = df_filtered[(df_filtered['prediction pos ok']==True) & (df_filtered['id_from']!=df_filtered['id_to']) ]
            y_train = df_train['is loop'].to_numpy()

            # If cascaded (two) classifiers or one combined classifier
            if cascaded == False:
                X_train = df_train[feature_cols].to_numpy()
                logreg, _ = TrainClassifier(X_train,y_train) #Train classifier on subset of data to avoid position errors
            else:
                X_sc_train = df_train[['odom-bounds', 'sc-sim']].to_numpy()
                sc_logreg, _ = TrainClassifier(X_sc_train,y_train) #Train classifier on subset of data to avoid position errors

                X_align_train = df_train[['alignment_quality']].to_numpy()
                align_logreg, _ = TrainClassifier(X_align_train,y_train) #Train classifier on subset of data to avoid position errors

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

            y = df_filtered['is loop'].to_numpy()

            # If cascaded (two) classifiers or one combined classifier
            if  cascaded == False:
                X_test = df_filtered[feature_cols].to_numpy()
                y_prob = logreg.predict_proba(X_test)[:,1] * df_filtered['prediction pos ok'].values
            else:
                sc_X_test = df_filtered[['odom-bounds', 'sc-sim']].to_numpy()
                align_X_test = df_filtered[['alignment_quality']].to_numpy()
                sc_y_pred = sc_logreg.predict(sc_X_test)
                y_prob = sc_y_pred * align_logreg.predict_proba(align_X_test)[:,1] * df_filtered['prediction pos ok'].values

            fpr, tpr, _ = metrics.roc_curve(y, y_prob)
            tpr[-1] = tpr[-2]
            roc_auc = metrics.auc(fpr, tpr)
            plt.figure(0)
            plt.plot(fpr, tpr, label = name, linewidth=2)

            precision, recall, _ = precision_recall_curve(y, y_prob)
            # pr_auc = metrics.auc(fpr, tpr)
            recall[0] = recall[1]
            precision[0] = precision[1]
            plt.figure(1)
            plt.plot(recall, precision, label = name, linewidth=2)

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
    plt.savefig(out_dir + "/ROC.png")

    # PR CURVE
    plt.figure(1)
    plt.xlim([0, 1])
    plt.ylim([0, 1])
    plt.ylabel('Precision')
    plt.xlabel('Recall')
    plt.title(dataset)
    handles, labels = plt.gca().get_legend_handles_labels()
    labels, handles = zip(*sorted(zip(labels, handles), key=lambda t: t[0]))
    plt.legend(handles, labels, loc = 'lower left')
    plt.savefig(out_dir + "/PR.pdf", bbox_inches='tight', format='pdf')
    plt.savefig(out_dir + "/PR.png")
