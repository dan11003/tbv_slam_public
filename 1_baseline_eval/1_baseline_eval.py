import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np
from tabulate import tabulate
import pathlib
from pandas.api.types import CategoricalDtype
import glob
import argparse
import os

def LoadData(base_dir):
    dfs = []
    search_string="*slam_eval.csv"
    print("opening the following files:")
    for filename in glob.glob(base_dir+"/"+search_string, recursive=True):
        print(filename)
        df = pd.read_csv(filename, index_col=None, header=0, skipinitialspace=True)
        dfs.append(df)

    search_string="*odometry_eval.csv"
    for filename in glob.glob(base_dir+"/"+search_string, recursive=True):
        print(filename)
        df = pd.read_csv(filename, index_col=None, header=0, skipinitialspace=True)
        dfs.append(df)
    return pd.concat(dfs, axis=0, ignore_index=True)

def SavePlots(df, out_dir):
    plot_parameters = ["Trans.err.(%)", "Rot.err.(deg/100m)", "ATE(m)", "RMSE (m)"]
    plot_names = ["Trans_err", "Rot_err", "ATE", "RPE-RMSE"]
    sns.set_theme(style="ticks", color_codes=True)
    sns.set(style="ticks")

    # SAVE PLOT FOR SEQUENCES #
    for plot_par, plot_name in zip(plot_parameters,plot_names):
        g = sns.catplot(x="sequence", y=plot_par,
                        data=df, saturation=.5, hue="method",
                        kind="bar", ci=None, aspect=1)
        g.set_xticklabels(rotation=90)
        plt.savefig(out_dir + "/sequence_" + plot_name + ".pdf", bbox_inches='tight', format='pdf')

    # SAVE PLOT FOR METHOD #
    for plot_par, plot_name in zip(plot_parameters,plot_names):
        g = sns.catplot(x="method", y=plot_par,
                        data=df, saturation=.5,
                        kind="bar", ci=None, aspect=1)
        g.set_xticklabels(rotation=90)

        plt.savefig(out_dir + "/method_" + plot_name + ".pdf", bbox_inches='tight', format='pdf')

def SaveTable(df, out_dir):
    df_tables = []

    methods=df['method'].unique()
    sequences=df['sequence'].unique()
    np.append(methods, "mean")
    df["ATE(m)"]
    df_table = pd.DataFrame({"method" : methods})
    df_table.set_index("method", inplace=True)
    f = open(out_dir + "table.txt",'w')
    for met in methods:
        for seq in sequences:
            df_temp = df[(df["sequence"] == seq) & (df["method"] == met)]
            df_table.at[met,seq] = df_temp['for_copy'].values[0]
        df_temp = df[df["method"] == met]
        trans_mean = "{:2.2f}".format(df_temp["Trans.err.(%)"].mean())
        rot_mean = "{:2.2f}".format(df_temp["Rot.err.(deg/100m)"].mean())
        df_table.at[met,"mean"] = trans_mean + "/" + rot_mean
    df_tables.append(df_table.copy(deep=True))

    for met in methods:
        for seq in sequences:
            df_temp = df[(df["sequence"] == seq) & (df["method"] == met)]
            df_table.at[met,seq] = df_temp['for_copy_ATE'].values[0]
        df_temp = df[df["method"] == met]
        ate_mean = "{:2.2f}".format(df_temp["ATE(m)"].mean())
        df_table.at[met,"mean"] = ate_mean
    df_tables.append(df_table.copy(deep=True))

    for met in methods:
        for seq in sequences:
            df_temp = df[(df["sequence"] == seq) & (df["method"] == met)]
            df_table.at[met,seq] = df_temp['for_copy_RMSE'].values[0]
        df_temp = df[df["method"] == met]
        rpe_rmse_mean = "{:2.2f}".format(df_temp["RMSE (m)"].mean())
        df_table.at[met,"mean"] = rpe_rmse_mean
    df_tables.append(df_table.copy(deep=True))

    print("Odometry error transl[m/m]/rot[deg/100m]", file=f)
    print(tabulate(df_tables[0], headers='keys', tablefmt='fancy_grid'), file=f)
    print(file=f)
    print("ATE-RMSE [m]: ", file=f)
    print(tabulate(df_tables[1], headers='keys', tablefmt='fancy_grid'), file=f)
    print(file=f)
    print("RPE-RMSE [cm]: ", file=f)
    print(tabulate(df_tables[2], headers='keys', tablefmt='fancy_grid'), file=f)
    print("\n\n---------LATEX---------\n\n", file=f)
    print(tabulate(df_tables[0], headers='keys', tablefmt='latex_raw'), file=f)
    print(file=f)
    print(tabulate(df_tables[1], headers='keys', tablefmt='latex_raw'), file=f)
    print(file=f)
    print(tabulate(df_tables[2], headers='keys', tablefmt='latex_raw'), file=f)

    print("Saving table in ", out_dir)
    f.close()

def FormatDataframe(df, dataset):
    df = df.rename(columns={'resolution r': 'resolution r [m]'})
    df['for_copy'] = df.apply (lambda row: "{:2.2f}".format(row["Trans.err.(%)"])+"/"+"{:2.2f}".format(row["Rot.err.(deg/100m)"]) , axis=1)
    df['for_copy_ATE'] = df.apply (lambda row: "{:2.2f}".format(row["ATE(m)"]) , axis=1)
    df['for_copy_RMSE'] = df.apply (lambda row: "{:2.2f}".format(100*row["RMSE (m)"]) , axis=1)
    rpe_rmse_mean = "{:2.2f}".format(100*df["RMSE (m)"].mean())

    # PREPARE SEQUENCES DEPENDING ON DATASET #
    if(dataset == "Mulran"):
        ##  MULRAN  ##
        df=df[df["dataset"]==dataset]
        cat_size_order = CategoricalDtype(
        ['KAIST01','KAIST02','KAIST03','DCC01','DCC02','DCC03','Riverside01','Riverside02','Riverside03'],
            ordered=True
        )
        df['sequence'] = df["sequence"].astype(cat_size_order)
        df=df.sort_values('sequence')
    else:
        ###  OXFORD ###
        df=df[df["dataset"]==dataset]
        df['sequence'] = df.apply (lambda row: str(row["sequence"])[8:-20], axis=1)
        cat_size_order = CategoricalDtype(
            ["10-12-32", "16-13-09", "17-13-26", "18-14-14", "18-15-20", "10-11-46", "16-11-53", "18-14-46"],
            ordered=True
        )
        df['sequence'] = df["sequence"].astype(cat_size_order)
        df=df.sort_values(['sequence','method'])
    return df

def main():
    parser = argparse.ArgumentParser(description='Baseline eval')
    parser.add_argument('--dir', type=str, required=False,default="", help="Result directory")
    parser.add_argument('--dataset', type=str, required=False,default="Mulran", choices=["Mulran", "oxford"], help="Datset")
    parser.add_argument('--output', type=str, required=False, default="", help="Output dir for saved images")
    args = parser.parse_args()

    base_dir = os.environ["BAG_LOCATION"] + "/TBV_Eval/" + args.dir
    out_dir = args.output
    dataset = args.dataset
    if(args.dir == ""):
        if(dataset == "Mulran"):
            base_dir = str(pathlib.Path(__file__).parent.resolve()) + "/../data/full_mulran_eval_2022-09-23_18-16/"
        else:
            base_dir = str(pathlib.Path(__file__).parent.resolve()) + "/../data/full_oxford_eval_2022-09-24_11-17/"
    if(args.output == ""):
        out_dir = base_dir + "/output/baseline/"

    if(not pathlib.Path(base_dir)):
        print(base_dir, "doesnt exist!")
        exit()

    pathlib.Path(out_dir).mkdir(parents=True, exist_ok=True)
    print("created output dir at", out_dir)

    print("Loading data from", base_dir)
    df = LoadData(base_dir)

    print("Formatting dataframe ", out_dir)
    df = FormatDataframe(df, dataset)

    print("Saving plots in ", out_dir)
    SavePlots(df, out_dir)

    print("Saving plots in ", out_dir)
    SaveTable(df, out_dir)

if __name__ == '__main__':
    main()
