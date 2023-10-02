from matplotlib import pyplot as plt
import numpy as np
import os
from glob import glob
import argparse
import glob
import pathlib
import os
import pandas as pd

plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['ps.fonttype'] = 42

def LoadData(paths, align):
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
    return concatinated

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

def PlotTrajectory(trajectories, title, dataset, output, gt, names, flip=False):
    fontsize_ = 20
    colors = ["b","g","r","c","m","y"]

    pose_flip = 1
    if dataset=="oxford":
        pose_flip = -1

    _flip = ''
    if flip:
        _flip += "_flip"
        for index in range(len(trajectories)):
            trajectories[index] = np.transpose([[0,-1],[1,0]]@np.transpose(trajectories[index]))
    fig = plt.figure()
    ax = plt.gca()
    ax.set_aspect('equal')
    if(gt):
        poses_gt = trajectories.pop(-1)
        plt.plot(poses_gt[:, 0], pose_flip*poses_gt[:, 1],"--", color="red", label="Ground truth")
        plt.plot(poses_gt[-1,0], poses_gt[-1,1], "s",color="red",markersize=15,markeredgewidth=3,fillstyle='none')
        plt.plot(poses_gt[0,0], poses_gt[0,1], "x",color="black",markersize=15,markeredgewidth=3,fillstyle='none')
    else:
        plt.plot(trajectories[-1][0,0], trajectories[-1][0,1], "x",color="black",markersize=15,markeredgewidth=3,fillstyle='none')

    colors = colors[0:len(trajectories)]
    for traj, name,color in zip(trajectories,names,colors):
        plt.plot(traj[:, 0], pose_flip*traj[:, 1], "--", color=color, label=name)
        plt.plot(traj[-1,0], pose_flip*traj[-1,1], "s", color=color, markersize=15,markeredgewidth=3,fillstyle='none')

    plt.title(title, loc='left')
    plt.legend( loc='lower left', prop={'size': fontsize_}) #
    plt.xticks(fontsize=fontsize_)
    plt.xticks(rotation=45)
    plt.yticks(fontsize=fontsize_)
    plt.xlabel('x (m)', fontsize=fontsize_)
    plt.ylabel('y (m)', fontsize=fontsize_)
    fig.set_size_inches(10, 10)

    fig_pdf =  output + "/" + title + _flip + ".pdf"
    plt.savefig(fig_pdf, bbox_inches='tight', pad_inches=0)

def main():
    # PARAMETR PARSER #
    parser = argparse.ArgumentParser(description='Baseline eval')
    parser.add_argument('--dir', type=str, required=True,help="Result directory")
    parser.add_argument('--output', type=str, required=False,default="",help="Output dir for saved images")
    parser.add_argument('--align', type=str ,default="True", choices=["True", "False"],help="Align, True/False")
    parser.add_argument('--gt', type=str, default="True", choices=["True", "False"],help="Ground Truth, True/False")
    parser.add_argument('--flip', type=str, default="False", choices=["True", "False"],help="Flip plots, True/False")
    parser.add_argument('--full_path', type=str, default="False", choices=["True", "False"],help="If dir is full path, or in TBV_Eval")
    args = parser.parse_args()

    base_dir = args.dir if args.full_path == 'True' else os.environ["BAG_LOCATION"] + "/TBV_Eval/" + args.dir
    out_dir = args.output + "/output/plot_trajectory/" if args.output != '' else base_dir + "/output/plot_trajectory/"
    dataset = pd.read_csv(base_dir+"/job_0/pars.txt", index_col=0, header=0, skipinitialspace=True).T["dataset"].values[0]

    gt = args.gt == "True"
    align = args.align == "True"
    flip = args.flip == "True"

    pathlib.Path(out_dir).mkdir(parents=True, exist_ok=True)
    print("Output dir:", out_dir)

    if (not pathlib.Path(base_dir)) or (not pathlib.Path(out_dir)):
        print(base_dir, "or", out_dir, "doesnt exist!")
        exit()


    # LOAD DATA AND SAVE PLOTS #
    jobs = glob.glob(base_dir + "/job_?")
    for job in jobs:
        df = pd.read_csv(job+"/pars.txt", index_col=0, header=0, skipinitialspace=True).T
        sequence = df["sequence"].values[0]
        method = df["method"].values[0]
        est_slam_base = job + "/est/??.txt"
        est_slam_path = glob.glob(est_slam_base)
        est_odom_base = job + "/odom/??.txt"
        est_odom_path = glob.glob(est_odom_base)
        paths = [est_slam_path[0], est_odom_path[0]]

        if(gt):
            gt_base = job + "/gt/??.txt"
            gt_path = glob.glob(gt_base)
            paths.append(gt_path[0])
        trajectories = LoadData(paths, align)

        print("Save plots in:", out_dir)
        PlotTrajectory(trajectories=trajectories, title=sequence, dataset=dataset, output=out_dir, gt=gt, names=[method, "CFEAR-3"], flip=flip)

if __name__ == '__main__':
    main()