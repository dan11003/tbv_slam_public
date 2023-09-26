import pandas as pd
import os
import sklearn.metrics as metrics
import matplotlib.pyplot as plt
from matplotlib.widgets import RadioButtons
from matplotlib.widgets import Button
import math
import argparse
import numpy as np
from sklearn.metrics import confusion_matrix
from sklearn.metrics import accuracy_score
from sklearn.linear_model import LogisticRegression
from sklearn import tree

from sklearn.metrics import precision_score
from sklearn.metrics import recall_score
from sklearn.metrics import f1_score



def TrainClassifier(X,y):
    logreg = LogisticRegression(random_state=16) #, class_weight='balanced'
    logreg.fit(X, y)
    y_pred = logreg.predict(X)
    return logreg, y_pred

def Predict(logreg, X, th):
    y_pred = (logreg.predict_proba(X)[:,1] >= th).astype(int)
    return y_pred
def Predict_proba(logreg, X):
    y_pred = logreg.predict_proba(X)[:,1]
    return y_pred

def CorrectLabelForPosition(y, y_pred, y_pos_ok):
    y_corr = y
    count = 0
    if len(y) != len(y_pred):
        exit

    for i, x in enumerate(y_pred):
        if (y[i] == 1) and (y_pred[i]==1) and (y_pos_ok[i] == False) :
            count = count + 1
            y_corr[i] = 0

    return y_corr

def ComputeClassifierStatistics(y, y_pred, y_pos_ok):
    y_corrected = CorrectLabelForPosition(y, y_pred, y_pos_ok) # Always use this after getting a prediction
    acc = accuracy_score(y_corrected, y_pred)
    precision = precision_score(y_corrected, y_pred)
    recall = recall_score(y_corrected, y_pred)
    cnf_matrix = metrics.confusion_matrix(y_corrected, y_pred)
    tn, fp, fn, tp = cnf_matrix.ravel()
    return acc, precision, recall, tn, fp, fn, fp, cnf_matrix, y_corrected
