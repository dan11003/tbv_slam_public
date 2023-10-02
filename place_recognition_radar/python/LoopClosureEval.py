import pandas as pd
import os
import sklearn.metrics as metrics
import matplotlib.pyplot as plt
import math
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
from sklearn.metrics import precision_recall_curve

# Disable warnings (not sure if this is good)
def warn(*args, **kwargs):
    pass
import warnings
warnings.warn = warn

def WriteFile(filename, metrics, disable_output):
    acc_train, train_precision, train_recall, cnf_matrix_train, acc_test, test_precision, test_recall, cnf_matrix_test, coef, intercept, pthreshold, nr_correct_candidates, nr_loops, correct_loop_ratio = metrics
    f = open(filename, 'w')

    lines = []
    lines.append("Training accuracy[%], {:.3f}\n".format(acc_train*100) )
    lines.append("Training precision [%], {:.3f}\n".format(train_precision*100))
    lines.append("Training recall [%], {:.3f}\n".format(train_recall*100))

    lines.append("Testing accurac [%], {:.3f}\n".format(acc_test*100) )
    lines.append("Testing precision [%], {:.3f}\n".format(test_precision*100))
    lines.append("Testing recall [%], {:.3f}\n".format(test_recall*100))

    lines.append("nr correct candidates, {}\n".format(nr_correct_candidates))
    lines.append("nr loops, {}\n".format(nr_loops))
    lines.append("correct_loop_ratio [%], {:.3f}\n".format(correct_loop_ratio*100))


    lines.append("Coef, {}\n".format(coef))
    lines.append("Intercept {}\n".format(intercept))
    lines.append("Threshold, {}\n".format(pthreshold))


    for line in lines:
        if not disable_output:
            print(line, end=' ')
        f.writelines(line)

def SaveRocCurve(path, y, y_prob):
    if(y.size != y_prob.size):
        return
    plt.clf()
    plt.xlim([-0.01, 1])
    plt.ylim([0, 1])
    fpr, tpr, _ = metrics.roc_curve(y, y_prob)
    tpr[-1] = tpr[-2]
    roc_auc = metrics.auc(fpr, tpr)
    plt.ylabel('True Positive Rate')
    plt.xlabel('False Positive Rate')
    plt.title("ROC")
    plt.plot(fpr, tpr, 'b', label = 'AUC: = %0.2f' % roc_auc, linewidth=3)
    plt.plot([0, 1], [0, 1],'r--')
    plt.legend(loc = 'lower right')
    plt.savefig(path + "/ROC_loops.pdf", bbox_inches='tight', format='pdf')
    
    
    precision, recall, _ = precision_recall_curve(y, y_prob)
    # pr_auc = metrics.auc(fpr, tpr)
    plt.figure(1)
    plt.plot(recall, precision, linewidth=2)
    plt.ylabel('Precision')
    plt.xlabel('Recall')
    plt.savefig(path + "/precision_recall_loops.pdf", bbox_inches='tight', format='pdf')



if __name__ == '__main__':
    # Parse input arguments
    parser = argparse.ArgumentParser(description='Get script arguments.')
    parser.add_argument('--output_folder', type=str, required=True, help='output folder')
    parser.add_argument('--csv_file', type=str, default='result.txt', help='csv file input')
    parser.add_argument('--disable-output', type=str, default='', help='csv file input')
    parser.add_argument('--p-threshold', type=float, default=0.9, help='th for prediction')
    parser.add_argument('--max_distance', type=float, default=6, help='max distance for loop')
    parser.add_argument('--max_registration_distance', type=float, default=4, help='max registration distance for loop')
    parser.add_argument('--max_registration_rotation', type=float, default=2.5, help='max registration rotation for loop')
    parser.add_argument('--use_guess0', default="True", choices=["True", "False"], help='Use only guess 0, otherwise use best probability guess')
    parser.add_argument('--plot_type', type=str, default='all', choices=['2d','3d', 'all'], help='Plot either in 2d, 3d, or all')
    parser.add_argument('--show', type=str, default="False", choices=["True", "False"], help='Show matlibplot interactable window of loop figure')
    parser.add_argument('--save_roc', type=str, default="True", choices=["True", "False"], help='Save ROC curve')
    args = parser.parse_args()

    if(not os.path.isfile(args.csv_file)):
        print("No such csv file!")
        exit()

    disable_output = args.disable_output!=''
    pthreshold = args.p_threshold

    if not disable_output:
        print("Loading file: " + args.csv_file)
    # Read experiment csv file
    df = pd.read_csv(args.csv_file, sep=r',', skipinitialspace=True)
    datapoints = len(df.index)

    if not disable_output:
        print("Loaded: " + str(datapoints))

    df['max_distance'] = df.apply (lambda row: args.max_distance, axis=1)
    df['is loop'] = df.apply (lambda row: int(row["closest_loop_distance"]<row["max_distance"]), axis=1)
    # df['candidate close'] = df.apply (lambda row: (row["candidate_loop_distance"]<=row["max_distance"]) , axis=1)
    df['candidate close'] = df.apply (lambda row:( math.sqrt(row["diff.x"]*row["diff.x"] + row["diff.y"]*row["diff.y"]) < args.max_registration_distance) & (math.fabs(row["diff.z"]) < args.max_registration_rotation*math.pi/180.0), axis=1)
    df['candidate transl_error'] = df.apply (lambda row: math.sqrt(row["diff.x"]*row["diff.x"] + row["diff.y"]*row["diff.y"]), axis=1)
    df['candidate rot_error'] = df.apply (lambda row: 180.0/math.pi*math.fabs(row["diff.z"]), axis=1)
    df['prediction pos ok'] = df.apply (lambda row: int(not row["is loop"] or row["candidate close"]), axis=1) #Please verify this logic. only false if is loop while candidate is far away
    df_filtered=df[ (df['guess_nr']==0 ) ]
    df_all=df[(df['guess_nr']>= 0 )]
    df_all=df_all.reset_index()
    df_train = df_filtered[(df_filtered['prediction pos ok']==True) & (df['id_from']!=df['id_to']) ]

    nr_correct_candidates = df_filtered['candidate close'].sum()
    nr_loops = df_filtered['is loop'].sum()
    correct_loop_ratio = nr_correct_candidates/nr_loops
    print("Ok candidates " + str(nr_correct_candidates))
    print("Actual Loops " + str(nr_loops ) )
    print("Ratio " + str(correct_loop_ratio ) )

    if not disable_output:
        print("Remaining: " + str(len(df_train.index)/datapoints))

    feature_cols=['odom-bounds','sc-sim', 'alignment_quality']

    X_train = df_train[feature_cols].to_numpy()
    y_train = df_train['is loop'].to_numpy()
    y_train_pos_ok = df_train['candidate close'].to_numpy()

    logreg, y_train_pred = TrainClassifier(X_train,y_train) #Train classifier on subset of data to avoid position errors
    train_acc, train_precision, train_recall, tn, fp, fn, fp, cnf_matrix_train, _ = ComputeClassifierStatistics(y_train, y_train_pred, y_train_pos_ok) #extract statistics

    # Update df_fitered using best probabilites if use_guess0 == True (defualt)
    if args.use_guess0 == "False":
        X_test_all = df_all[feature_cols].to_numpy()
        y_test_prob_all = Predict_proba(logreg, X_test_all) #Predict on all datapoints
        df_all["y_test_prob"] = y_test_prob_all
        nr_guess = df_all['guess_nr'].to_numpy().max() + 1
        M_pred = y_test_prob_all.reshape(-1, nr_guess) #reshape into matrix
        argmax = np.argmax(M_pred, axis=1, keepdims=False) #highest probability
        max_idx = np.array(range(0, df_all.shape[0], nr_guess)) + argmax #index for df for highest probability
        df_filtered=df_all.loc[max_idx.tolist()] #create new df for highest probability candidates


    X_test = df_filtered[feature_cols].to_numpy()
    y_test = df_filtered['is loop'].to_numpy()
    y_test_pos_ok = df_filtered['candidate close'].to_numpy()
    y_test_pred = Predict(logreg, X_test, pthreshold) #Predict on all datapoints

    acc_test, test_precision, test_recall, tn, fp_test, fn, fp, cnf_matrix_test, y_test = ComputeClassifierStatistics(y_test, y_test_pred, y_test_pos_ok) #extract statistics and fix "y_test"

    df_filtered["y_test_pred"] = y_test_pred
    df_filtered["y_test"] = y_test

    if args.plot_type == '2d' or args.plot_type == 'all':
        vis = VisLoops(df_filtered)
        vis.Visualize()
        vis.Save(dir=args.output_folder, name="loops_2d.pdf")
        if args.show == "True":
            vis.Show()

    if args.plot_type == '3d' or args.plot_type == 'all':
        vis = VisLoops3d(df_filtered)
        vis.Visualize()
        vis.Save(dir=args.output_folder, name="loops_3d.pdf")
        if args.show == "True":
            vis.Show()

    #fpr, tpr, _ = metrics.roc_curve(y_test,  y_pred_proba)
    if not disable_output:
        print("-------------")
        print("Training")
        print(cnf_matrix_train)
        print("-------------\n")
        print("-------------")
        print("\ntest_precision")
        print(cnf_matrix_test)
        print("-------------\n")


    result_path = os.path.join(args.output_folder, "result.txt")
    WriteFile(result_path, [train_acc, train_precision, train_recall, cnf_matrix_train, acc_test, test_precision, test_recall, cnf_matrix_test, logreg.coef_, logreg.intercept_, pthreshold, nr_correct_candidates, nr_loops, correct_loop_ratio], disable_output)

    if args.save_roc == "True":
        # Save ROC-Curve
        y = df_filtered['is loop'].to_numpy()
        y_prob = logreg.predict_proba(X_test)[:,1] * df_filtered['prediction pos ok'].values
        SaveRocCurve(args.output_folder, y, y_prob)
