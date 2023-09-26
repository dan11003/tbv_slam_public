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

plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['ps.fonttype'] = 42

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
        rpe_rmse_mean = "{:2.2f}".format(100*df_temp["RMSE (m)"].mean())
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
    parser.add_argument('--dir', type=str, required=True, help="Result directory")
    parser.add_argument('--output', type=str, required=False, default="", help="Output dir for saved images")
    parser.add_argument('--full_path', type=str, default="False", choices=["True", "False"],help="If dir is full path, or in TBV_Eval")
    args = parser.parse_args()

    base_dir = args.dir if args.full_path == 'True' else os.environ["BAG_LOCATION"] + "/TBV_Eval/" + args.dir
    out_dir = args.output + "/output/baseline/" if args.output != '' else base_dir + "/output/baseline/"
    dataset = pd.read_csv(base_dir+"/job_0/pars.txt", index_col=0, header=0, skipinitialspace=True).T["dataset"].values[0]
    #dataset = pd.read_csv(base_dir+"pars.txt", index_col=0, header=0, skipinitialspace=True).T["dataset"].values[0]

    pathlib.Path(out_dir).mkdir(parents=True, exist_ok=True)
    print("Output dir:", out_dir)

    if (not pathlib.Path(base_dir)) or (not pathlib.Path(out_dir)):
        print(base_dir, "or", out_dir, "doesnt exist!")
        exit()

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
