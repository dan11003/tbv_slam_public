# Copyright (C) Huangying Zhan 2019. All rights reserved.
#!/usr/bin/env python3
import argparse

from kitti_odometry import KittiEvalOdom

parser = argparse.ArgumentParser(description='KITTI evaluation')
parser.add_argument('--dir', type=str, required=True,
                    help="Result directory")
parser.add_argument('--align', type=str, 
                    choices=['scale', 'scale_7dof', '7dof', '6dof'],
                    default=None,
                    help="alignment type")
parser.add_argument('--seqs', 
                    nargs="+",
                    type=int, 
                    help="sequences to be evaluated",
                    default=None)
parser.add_argument('--force', type=str,
                    choices=['yes', 'no'],
                    default='no',
                    help="do not ask for user input")
parser.add_argument('--step-size', type=int,
                    default=10,
                    help="do not ask for user input")
parser.add_argument('--reg_expression', type=str,
                    default="??",
                    help="regex for .txt files")                    
args = parser.parse_args()

eval_tool = KittiEvalOdom(args.step_size)
main_dir = args.dir
gt_dir = main_dir+"/gt"
result_dir = main_dir+"/est"
print("gt dir "+gt_dir)
print("results dir "+result_dir)

if args.force=='yes':
    continue_flag = 'y'
else:
    continue_flag = input("Evaluate result in {}? [y/n]".format(result_dir))

if continue_flag == "y":
    eval_tool.eval(
        gt_dir,
        result_dir,
        alignment=args.align,
        seqs=args.seqs,
        reg_expression=args.reg_expression
        )
else:
    print("Double check the path!")
