# This Python file uses the following encoding: utf-8
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import pandas as pd
from sklearn import metrics
import os
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import classification_report, confusion_matrix
from sklearn import preprocessing
from sklearn.model_selection import train_test_split
import seaborn as sns
import argparse

def PrintConfusionMatric(cnf_matrix, cnf_matrix_un, directory):

    if not os.path.exists(directory):
        os.mkdir(directory)

    plt.clf()
    class_names=[0,1] # name  of classes
    fig, ax = plt.subplots()
    tick_marks = np.arange(len(class_names))
    plt.xticks(tick_marks, class_names)
    plt.yticks(tick_marks, class_names)
    # create heatmap
    sns.heatmap(pd.DataFrame(cnf_matrix), annot=True,fmt='g') #cmap="YlGnBu"
    ax.xaxis.set_label_position("top")
    plt.tight_layout()
    plt.title('Confusion matrix', y=1.1)
    plt.ylabel('Actual label')
    plt.xlabel('Predicted label')
    path = os.path.join(directory,"confusion")
    plt.savefig(path+".png")
    plt.savefig(path+".pdf", bbox_inches='tight')
    #plt.show()

    # Calculate accuracy
    accuracy = (cnf_matrix_un[0][0]+cnf_matrix_un[1][1])/(cnf_matrix_un[0][0]+cnf_matrix_un[0][1]+cnf_matrix_un[1][0]+cnf_matrix_un[1][1])
    return accuracy

def SaveROC(X,y,directory, name = "ROC"):
    if not os.path.exists(directory):
        os.mkdir(directory)
    
    plt.clf()
    logreg = LogisticRegression(class_weight='balanced')
    logreg.fit(X,y)
    y_pred_proba = logreg.predict_proba(X)[::,1]
    fpr, tpr, _ = metrics.roc_curve(y,  y_pred_proba, drop_intermediate=True)
    roc_auc = metrics.roc_auc_score(y, y_pred_proba)
    plt.plot(fpr, tpr, 'b', label = 'AUC = %0.2f' % roc_auc, marker='.')
    plt.legend(loc = 'lower right')
    plt.plot([0, 1], [0, 1],'r--')
    plt.xlim([0, 1])
    plt.ylim([0, 1])
    plt.ylabel('True Positive Rate', labelpad=0)
    plt.xlabel('False Positive Rate', labelpad=0)
    plt.title("ROC", pad=0)

    path = os.path.join(directory, name)
    plt.savefig(path+".png")
    plt.savefig(path+".pdf", bbox_inches='tight')

def PrintROC(logreg,X_test,y_test,directory):
    if not os.path.exists(directory):
        os.mkdir(directory)
    
    plt.clf()
    y_pred_proba = logreg.predict_proba(X_test)[::,1]
    fpr, tpr, _ = metrics.roc_curve(y_test,  y_pred_proba)
    auc = metrics.roc_auc_score(y_test, y_pred_proba)
    plt.plot(fpr,tpr,label="data 1, auc="+str(auc))
    plt.legend(loc=4)

    path = os.path.join(directory,"ROC")
    plt.savefig(path+".png")
    plt.savefig(path+".pdf", bbox_inches='tight')
    #plt.show()
    return auc

def TrainClassifier(df):
    col_names=['score1','score2','score3']
    #df = pd.read_csv(file_path)
    X = df[col_names]
    y = df.aligned

    y=y.astype('int')

    #print(X_train)
    #print(y_train)
    logreg = LogisticRegression(class_weight='balanced')
    logreg.fit(X,y)
    y_pred=logreg.predict(X)
    accuracy = metrics.balanced_accuracy_score(y,y_pred)
    #print("accuracy: "+str(accuracy))
    cnf_matrix = metrics.confusion_matrix(y, y_pred,normalize=None) #'pred'
    return accuracy,cnf_matrix
    #print(cnf_matrix)
    #print(y_pred)
    #print(y)

    #PrintConfusionMatric(cnf_matrix, output_directory)
    #PrintROC(logreg, X_test, y_test, output_directory)
