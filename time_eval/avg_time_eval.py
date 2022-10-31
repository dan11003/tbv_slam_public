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
from tabulate import tabulate

def LoadData(base_dir):
    dfs = []
    print("opening the following files:")
    for job in os.listdir(base_dir):
        if match(r"^job_[0-9]+", job):
            print("Loading data: ", job)
            df = pd.read_csv(base_dir+'/'+job+ "/time_statistics.txt", index_col=0, header=None, skipinitialspace=True).T
            dfs.append(df)
    return pd.concat(dfs, axis=0, ignore_index=True)

def SaveTable(df, out_dir):
    f = open(out_dir + "table.txt",'w')
    print(file=f)
    print(tabulate(df, headers='keys', tablefmt='fancy_grid'), file=f)
    print("\n\n---------LATEX---------\n\n", file=f)
    print(tabulate(df, headers='keys', tablefmt='latex_raw'), file=f)

def SavePlots(df, out_dir):
    sns.set_theme(style="ticks", color_codes=True)
    sns.set(style="ticks")
    plots = ["avg", "std", "tot"]
    for plot in plots:
        g = sns.catplot(x="task", y=plot,
                            data=df, saturation=.5,
                            kind="bar", ci=None, aspect=1)
        g.set_xticklabels(rotation=90)
        plt.savefig(out_dir + "/" + plot + ".pdf", bbox_inches='tight', format='pdf')

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
    df_full = LoadData(base_dir)

    datapoints = len(df_full.index)
    print("Loaded: " + str(datapoints))

    task = df_full[df_full.columns[0::3]].columns.values.tolist()
    task = [x[:-4] for x in task]
    avg_time = df_full[df_full.columns[0::3]].mean(axis=0).values
    std_time = df_full[df_full.columns[1::3]].mean(axis=0).values
    count = df_full[df_full.columns[2::3]].mean(axis=0).values
    tot_time = avg_time * count
    df_mean = pd.DataFrame({"task" : task, "avg" : avg_time, "std" : std_time, "count" : count, "tot" : tot_time})

    SavePlots(df_mean, out_dir)

    df_mean.set_index("task", inplace=True)
    SaveTable(df_mean.T, out_dir)