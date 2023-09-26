import argparse
import os
import re
import csv
import numpy as np
import pandas as pd
import os.path


parser = argparse.ArgumentParser(description='Merges all datapoints from odometry evaluation')
parser.add_argument('--dir', type=str, required=True,
                    help="Result directory")
parser.add_argument('--prefix', type=str, required=False,default="",
                    help="File name")
parser.add_argument('--file', type=str, required=True, action='append',
                    help="file to merge")                                    

args = parser.parse_args()

main_dir = args.dir
prefix = args.prefix
merges = args.file

print("Looking for jobs under: {}",main_dir)
print("Job directory"+main_dir)
print("Files to merge files:", merges)

if(len(merges) < 2):
  print("Must input atleast 2 files to merge")
  exit()

regex = re.compile('job_*')
data=[]
count=0
for root, dirs, files in os.walk(main_dir):
  for dir in dirs:
      if regex.match(dir):
        headers=[]
        values=[]
        for merge in merges:
          merge_path=main_dir+dir+"/"+merge
          if not os.path.isfile(merge_path):
            continue
          if os.stat(merge_path).st_size == 0:
            continue

          file_handle = pd.read_csv(merge_path)
          dfile = np.array(pd.DataFrame(file_handle))
          headers.append(dfile[:,0])
          values.append(dfile[:,1])

        if count == 0:
          header=np.concatenate(headers)
          data = np.transpose(header)
          count=count+1
        vals=np.concatenate(values)

        data = np.vstack((data, np.transpose(vals)))
path=main_dir+"/"+prefix+'_eval.csv'
print("Save evaluation to ",path)
pd.DataFrame(data).to_csv(path,header=None,index=None)


