from genericpath import exists
from tkinter.tix import Tree
from matplotlib import pyplot as plt
import numpy as np
import os
from glob import glob
import argparse
import glob
import pathlib
import os
import pandas as pd


def LoadData(est1, est2, gt, align):
    paths=[est1,est2,gt]
    concatinated=[]
    for path in paths:
        print("load: "+path)
        est3 = np.loadtxt(path)
        est_np=np.array(est3)
        R = np.transpose([ [est_np[0,0],est_np[0,1]],[est_np[0,4],est_np[0,5]]])
        t =[ [est_np[0,3]],[ est_np[0,7] ] ]
        x=np.array(est3[:,3])
        y=np.array(est3[:,7])
        p=np.array([x,y])
        p=R@(p-t)
        concatinated.append(np.transpose(p))
    return concatinated[0], concatinated[1], concatinated[2]


def PlotTrajectory(poses_result, poses_result2, poses_gt, title, dataset, output):
    fontsize_ = 20
    pose_flip = 1
    if dataset=="oxford":
        pose_flip = -1

    for i in range(2):
        flip_text = ""
        if i == 1:
            poses_gt=np.transpose([[0,-1],[1,0]]@np.transpose(poses_gt))
            poses_result=np.transpose([[0,-1],[1,0]]@np.transpose(poses_result))
            poses_result2=np.transpose([[0,-1],[1,0]]@np.transpose(poses_result2))
            flip_text = "_flip"
    
        fig = plt.figure()
        ax = plt.gca()
        ax.set_aspect('equal')
        plt.plot(poses_gt[:, 0], pose_flip*poses_gt[:, 1],"--", color="orange", label="Ground truth")
        plt.plot(poses_gt[-1,0], poses_gt[-1,1], "s",color="orange",markersize=15,markeredgewidth=3,fillstyle='none')
        plt.plot(poses_gt[0,0], poses_gt[0,1], "x",color="black",markersize=15,markeredgewidth=3,fillstyle='none')

        plt.plot(poses_result[:, 0], pose_flip*poses_result[:, 1],"b", label="TBV SLAM-1")
        plt.plot(poses_result[-1,0], pose_flip*poses_result[-1,1], "bs", markersize=15,markeredgewidth=3,fillstyle='none')

        plt.plot(poses_result2[:, 0],  pose_flip*poses_result2[:, 1],"g", label="CFEAR-3")
        plt.plot(poses_result2[-1,0],  pose_flip*poses_result2[-1,1], "gs", markersize=15,markeredgewidth=3,fillstyle='none')
        plt.title(title, loc='left')

        plt.legend( loc='lower left', prop={'size': fontsize_}) #
        plt.xticks(fontsize=fontsize_)
        plt.xticks(rotation=45)
        plt.yticks(fontsize=fontsize_)
        plt.xlabel('x (m)', fontsize=fontsize_)
        plt.ylabel('y (m)', fontsize=fontsize_)
        fig.set_size_inches(10, 10)

        fig_pdf =  output + "/" + title + flip_text +".pdf"
        plt.savefig(fig_pdf, bbox_inches='tight', pad_inches=0)

def main():
    # PARAMETR PARSER #
    parser = argparse.ArgumentParser(description='Baseline eval')
    parser.add_argument('--dir', type=str, required=False,default="",
                        help="Result directory")
    parser.add_argument('--dataset', type=str, required=False,default="Mulran", choices=["Mulran", "oxford"],
                        help="Datset")
    parser.add_argument('--output', type=str, required=False,default="",
                        help="Output dir for saved images")
    args = parser.parse_args()

    base_dir = os.environ["BAG_LOCATION"] + "/TBV_Eval/" + args.dir
    out_dir = args.output
    dataset = args.dataset

    if(args.dir == ""):
        base_dir = str(pathlib.Path(__file__).parent.resolve()) + "/data/"
    if(args.output == ""):
        out_dir = base_dir + "/output/plotTrajectory/"

    if(not pathlib.Path(base_dir)):
        print(base_dir, "doesnt exist!")
        exit()

    pathlib.Path(out_dir).mkdir(parents=True, exist_ok=True)
    print("created output dir at", out_dir)

    # LOAD DATA AND SAVE PLOTS #
    jobs = glob.glob(base_dir + "/job_?")
    for job in jobs:
        df = pd.read_csv(job+"/pars.txt", index_col=0, header=0, skipinitialspace=True).T
        sequence = df["sequence"].values[0]
        est_slam_base = job + "/est/??.txt"
        est_odom_base = job + "/odom/??.txt"
        gt_base = job + "/gt/??.txt"
        est_slam_path = glob.glob(est_slam_base)
        est_odom_path = glob.glob(est_odom_base)
        gt_path = glob.glob(gt_base)
        est_slam, est_odom, gt = LoadData(est_slam_path[0], est_odom_path[0], gt_path[0], dataset=="Mulran")
        PlotTrajectory(est_slam, est_odom, gt, sequence, dataset, out_dir)

if __name__ == '__main__':
    main()