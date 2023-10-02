import pandas as pd
import os
import matplotlib.pyplot as plt
import argparse
import numpy as np
import sklearn.metrics as metrics
from sklearn.metrics import confusion_matrix

# Save plot
def draw_plot(df_full : pd.DataFrame, ths_coral : list = [0.5]) -> None:
    # ROC CURVE 
    n = 500
    ths=np.linspace(0.0, 1.00, num=n)

    tpr_sc = np.empty(n, dtype=float)
    fpr_sc = np.empty(n, dtype=float)
    for i in range(0,n,1):
        th=ths[i]
        df_full['prediction loop'] = df_full.apply (lambda row: row["quality_sc_sim"]<th, axis=1) #Please verify this logic. only false if is loop while candidate is far away
        df_full['prediction'] = df_full.apply (lambda row: (row["prediction pos ok"]) and row["prediction loop"], axis=1) #Please verify this logic. only false if is loop while candidate is far away
        y_true=np.multiply(np.array(df_full["is loop"]),1)
        y_pred=np.multiply(np.array(df_full["prediction"]),1)
        tn, fp, fn, tp = confusion_matrix(y_true, y_pred).ravel()
        tpr_sc[i]=tp/(tp+fn)
        fpr_sc[i]=fp/(fp+tn)

    for th_coral in ths_coral:
        tpr_sc_c = np.empty(n, dtype=float)
        fpr_sc_c = np.empty(n, dtype=float)
        for i in range(0,n,1):
            th=ths[i]
            df_full['prediction loop'] = df_full.apply (lambda row: (row["quality_sc_sim"]<th and row["quality_coral_prob"]>th_coral), axis=1) #Please verify this logic. only false if is loop while candidate is far away
            df_full['prediction'] = df_full.apply (lambda row: (row["prediction pos ok"]) and row["prediction loop"], axis=1) #Please verify this logic. only false if is loop while candidate is far away
            y_true=np.multiply(np.array(df_full["is loop"]),1)
            y_pred=np.multiply(np.array(df_full["prediction"]),1)
            tn, fp, fn, tp = confusion_matrix(y_true, y_pred).ravel()
            tpr_sc_c[i]=tp/(tp+fn)
            fpr_sc_c[i]=fp/(fp+tn)

        roc_auc_sc_c = metrics.auc(fpr_sc_c, tpr_sc_c)
        plt.plot(fpr_sc_c, tpr_sc_c, label = 'SC+Coral, th = %0.2f' % th_coral, marker='.')

    # plt.cla()
    roc_auc_sc = metrics.auc(fpr_sc, tpr_sc)
    plt.plot(fpr_sc, tpr_sc, label = 'SC' % roc_auc_sc, marker='.')

    #plt.plot(tpr, fpr, 'b', label = 'AUCz = %0.2f' % roc_auc)
    plt.legend(loc = 'lower right')
    # plt.plot([0, 1], [0, 1],'r--')
    plt.xlim([0, 1.1])
    plt.ylim([0, 1.1])
    plt.ylabel('True Positive Rate', labelpad=0)
    plt.xlabel('False Positive Rate', labelpad=0)
    plt.title("ROC", pad=0)

def reset_plot():
    plt.cla()
    plt.figure(num="Plot")
    plt.xlim([0, 1])
    plt.ylim([0, 1])
    plt.ylabel('True Positive Rate', labelpad=0)
    plt.xlabel('False Positive Rate', labelpad=0)


def save_plot(save_path : str, save_name : str):
    # Save plots in new folder
    if not os.path.exists(save_path):
        os.mkdir(save_path)

    plt.savefig(save_path + save_name + ".pdf", bbox_inches='tight', format='pdf')
    # plt.close()

def main():
    # Parse input arguments
    parser = argparse.ArgumentParser(description='Get script arguments.')
    parser.add_argument('--experiment_name', type=str, required=True, help='experiment name')
    parser.add_argument('--csv_file', type=str, default='merge.csv', help='experiment name')
    args = parser.parse_args()

    csv_path  = os.environ["BAG_LOCATION"] + "/place_recognition_eval/" + args.experiment_name
    csv_file = csv_path + "/" + args.csv_file
    if(not os.path.isfile(csv_file)):
        print("No such csv file!")
        exit()

    # Read experiment csv file
    print("Loading file: " + csv_path)
    df_full=pd.read_csv(csv_file, sep=r',',header=0, skipinitialspace=True)
    print("Loaded: " + str(len(df_full.index)))

    df_full['max_distance'] = df_full.apply (lambda row: 1, axis=1)
    df_full['is loop'] = df_full.apply (lambda row: row["closest_loop_distance"]<row["max_distance"], axis=1)
    df_full['candidate close'] = df_full.apply (lambda row: row["candidate_loop_distance"]<=row["max_distance"], axis=1)
    df_full['prediction pos ok'] = df_full.apply (lambda row: (not row["is loop"]) or row["candidate close"], axis=1) #Please verify this logic. only false if is loop while candidate is far away
    reset_plot()
    ths_coral = [0.5, 0.7, 0.9]
    draw_plot(df_full, ths_coral=ths_coral)
      
    save_plot(save_path=csv_path + "/plots/", save_name="ROC_"+args.csv_file)

if __name__ == '__main__':
    main()