#!/usr/bin/python

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import classification_report, confusion_matrix
from sklearn import preprocessing
from sklearn.model_selection import train_test_split
import seaborn as sns
import argparse
from sklearn import metrics
from utils import *

#plt.rc("font", size=14)
#sns.set(style="white")
#sns.set(style="whitegrid", color_codes=True)

parser = argparse.ArgumentParser()
parser.add_argument("--directory", "-d", help="path to data directory")
parser.add_argument("--filename", "-f", help="filename", default="eval.txt")
args = parser.parse_args()
if args.directory:
    print("directory %s" % args.directory)
if args.filename:
    print("filename %s" % args.filename)

file_path=os.path.join(args.directory, args.filename)

output_directory=os.path.join(args.directory, "output")
print(output_directory)
exit
#output_directory = args.directory+"/output"

col_names=['score1','score2','score3']
df = pd.read_csv(file_path)
X = df[col_names]
y = df.aligned
X_train,X_test,y_train,y_test=train_test_split(X,y,test_size=0.01,random_state=0)
X_test=X_train
y_test=y_train

#print(X_train)
#print(y_train)
data=np.array(y_train)
misaligned = np.count_nonzero(data==0)
aligned = np.count_nonzero(data==1)
ratio = misaligned/aligned
w= {0:1, 1:ratio}
print("datapoints - aligned="+str(aligned)+", misaligned="+str(misaligned))
print("ratio: "+str(ratio)+", w="+str(w) )

#print("aligned: "+str(aligned))
#print("misaligned: "+str(aligned))


logreg = LogisticRegression(class_weight='balanced')
logreg.fit(X_train,y_train)
y_pred=logreg.predict(X_test)
cnf_matrix = metrics.confusion_matrix(y_test, y_pred,normalize='true')
cnf_matrix_un = metrics.confusion_matrix(y_test, y_pred)
#cnf_matrix = cnf_matrix /cnf_matrix.astype(float).sum(axis=1)

print(cnf_matrix)
#print(y_pred)
#print(y_test)

accuracy = PrintConfusionMatric(cnf_matrix, cnf_matrix_un, output_directory)
auc = PrintROC(logreg, X_test, y_test, output_directory)

# Store confusion matrix as text file
f = open(output_directory + '/results.txt','w')
f.writelines([str(accuracy)+ ',' + str(auc) + ',' + str(cnf_matrix_un[0][0]) + ',' + str(cnf_matrix_un[0][1]) + ',' + str(cnf_matrix_un[1][0]) + ',' + str(cnf_matrix_un[1][1]) ]) 
f.close()

#accuracy cm11 cm12 cm21 cm22 nr1 nr0

#dg = pd.read_csv(args.path)
#df = pd.DataFrame(dg,columns = ['aligned','score1','score2','score3'])
##X = df['score1','score2','score3']
#y = df.aligned
