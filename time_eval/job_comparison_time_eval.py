from email import header
from operator import index
import pandas as pd
import os
import argparse
import numpy as np
import seaborn as sns
import pathlib
from re import match
import matplotlib.pyplot as plt

def LoadData(base_dir):
    dfs = []
    print("opening the following files:")
    for job in os.listdir(base_dir):
        if match(r"^job_[0-9]+", job):
            print("Loading data: ", job)
            df_stat = pd.read_csv(base_dir+'/'+job+ "/time_statistics.txt", index_col=0, header=None, skipinitialspace=True).T
            stats_len = len(df_stat.columns)
            df_par = pd.read_csv(base_dir+'/'+job+ "/pars.txt", index_col=0, header=None, skipinitialspace=True).T
            vals = df_stat.iloc[0,:].values.tolist() +  df_par.iloc[0,:].values.tolist()
            names = df_stat.columns.tolist() +  df_par.columns.tolist()
            temp_dict = dict(zip(names, vals))
            df = pd.DataFrame(temp_dict, index=[0])
            df.at[0,"job"] = job
            dfs.append(df)
    return pd.concat(dfs, axis=0, ignore_index=True), stats_len


def SavePlots(df, out_dir, plot_parameters):
    sns.set_theme(style="ticks", color_codes=True)
    sns.set(style="ticks")
    par_list = ["sequence"]
    for x in par_list:
        folder_idx = 0
        for y in plot_parameters:
            if folder_idx == 3:
                folder_idx = 0
            if folder_idx == 0:
                folder_name = y[:-4]
            g = sns.catplot(x=x, y=y,
                                data=df, saturation=.5,
                                kind="bar", ci=None, aspect=1)
            g.set_xticklabels(rotation=90)
            plot_par_name = ''.join(e for e in y if e.isalnum())
            par_name = ''.join(e for e in x if e.isalnum())
            pathlib.Path(out_dir+"/" + folder_name).mkdir(parents=True, exist_ok=True)
            plt.savefig(out_dir + "/" + folder_name + "/" + plot_par_name + "_" + par_name + ".pdf", bbox_inches='tight', format='pdf')
            plt.close()
            folder_idx += 1

if __name__ == '__main__':
    # Parse input arguments
    parser = argparse.ArgumentParser(description='Get script arguments.')
    parser.add_argument('--dir', type=str, required=True, help='experiment dir')
    parser.add_argument('--output', type=str, default='', help='output folder')
    args = parser.parse_args()

    base_dir = os.environ["BAG_LOCATION"] + "/TBV_Eval/" + args.dir
    out_dir = args.output

    if(args.output == ""):
        out_dir = base_dir + "/output/time_eval/"

    if(not pathlib.Path(base_dir)):
        print(base_dir, "doesnt exist!")
        exit()

    pathlib.Path(out_dir).mkdir(parents=True, exist_ok=True)
    print("created output dir at", out_dir)  

    # Read experiment csv file
    df_full, stats_len = LoadData(base_dir)

    datapoints = len(df_full.index)
    print("Loaded: " + str(datapoints))

    df_full['sequence'] = df_full.apply (lambda row: str(row["sequence"])[8:-20], axis=1)
    SavePlots(df_full, out_dir, df_full.columns[0:stats_len])