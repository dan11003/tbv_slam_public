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
        est = np.loadtxt(path)
        est_np=np.array(est)
        R = np.transpose([ [est_np[0,0],est_np[0,1]],[est_np[0,4],est_np[0,5]]])
        t =[ [est_np[0,3]],[ est_np[0,7] ] ]
        x=np.array(est[:,3])
        y=np.array(est[:,7])
        p=np.array([x,y])
        p=R@(p-t)
        concatinated.append(np.transpose(p))
    if(align):
        r, c, t = kabsch_umeyama(concatinated[-1], concatinated[0])
        concatinated[0] = np.array([t + c * r @ b for b in concatinated[0]])
    return concatinated[0], concatinated[1], concatinated[2]

# (https://zpl.fi/aligning-point-patterns-with-kabsch-umeyama-algorithm/)
def kabsch_umeyama(A, B):
    assert A.shape == B.shape
    n, m = A.shape

    EA = np.mean(A, axis=0)
    EB = np.mean(B, axis=0)
    VarA = np.mean(np.linalg.norm(A - EA, axis=1) ** 2)

    H = ((A - EA).T @ (B - EB)) / n
    U, D, VT = np.linalg.svd(H)
    d = np.sign(np.linalg.det(U) * np.linalg.det(VT))
    S = np.diag([1] * (m - 1) + [d])

    R = U @ S @ VT
    c = VarA / np.trace(np.diag(D) @ S)
    t = EA - c * R @ EB

    return R, c, t

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
        plt.plot(poses_gt[:, 0], pose_flip*poses_gt[:, 1],"-", color="red", label="Ground truth")
        plt.plot(poses_gt[-1,0], poses_gt[-1,1], "s",color="red",markersize=15,markeredgewidth=3,fillstyle='none')
        plt.plot(poses_gt[0,0], poses_gt[0,1], "x",color="black",markersize=15,markeredgewidth=3,fillstyle='none')

        plt.plot(poses_result[:, 0], pose_flip*poses_result[:, 1],"--", color="blue", label="TBV SLAM-1")
        plt.plot(poses_result[-1,0], pose_flip*poses_result[-1,1], "bs", markersize=15,markeredgewidth=3,fillstyle='none')

        plt.plot(poses_result2[:, 0],  pose_flip*poses_result2[:, 1],"--", color="green", label="CFEAR-3")
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
    parser.add_argument('--align', type=bool, required=False,default=True,
                    help="Align, true/false")
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
        out_dir = base_dir + "/output/plotTrajectory/"

    if(not pathlib.Path(base_dir)):
        print(base_dir, "doesnt exist!")
        exit()

    pathlib.Path(out_dir).mkdir(parents=True, exist_ok=True)
    print("Output dir:", out_dir)

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
        print("Load data:", job)
        est_slam, est_odom, gt = LoadData(est_slam_path[0], est_odom_path[0], gt_path[0], args.align)
        print("Save plots in:", out_dir)
        PlotTrajectory(est_slam, est_odom, gt, sequence, dataset, out_dir)

if __name__ == '__main__':
    main()