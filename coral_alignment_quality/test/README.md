# Testing for [alignmentinterface](../include/alignment_checker/alignmentinterface.h) functionality

In [python_classifier_interface_tests.cpp](scan_learning_interface_tests.cpp) are tests for __PythonClassifierInterface__.

- logisticRegressionPredictTest
    - Testing __predict()__ using Logistic Regression with some dummy training data.
- logisticRegressionPredictProbaTest
    - Testing __predict_proba()__ using Logistic Regression with some dummy training data.
- decisionTreePredictTest
    - Testing __predict()__ using Decision Trees with some dummy training data.
- decisionTreePredictProbaTest
    - Testing __predict_proba()__ using Decision Trees with some dummy training data.
- accuracyTest
    - Testing __Accuracy()__ after adding some more training data. Expect that accurcy is between [0.5,1].
- saveAndLoadDataTest
    - Testing __SaveData()__ and __LoadData()__ by first saving data (*/data/test_data/training_data.txt*), and then loading it and compare results from loaded and original classifier.
- saveROCCurveTest
    - Testing __SaveROCCurve()__ by first saving ROC-curve images (*/data/test_data/ROC.pdf*), and then checking if the images exists.


In [scan_learning_interface_tests.cpp](scan_learning_interface_tests.cpp) are tests for __ScanLearningInterface__.

- predAlignmentTest
    - Testing __PredAlignment()__ using Logistic Regression with training data loaded from [/data/simple_graph.sgh](../data/simple_graph.sgh).
- saveAndLoadDataTest
    - Testing __SaveData()__ and __LoadData()__ by first saving data (*/data/test_data/CFEAR.txt* & */data/test_data/CorAl.txt*), and then loading from saved files. 
- saveROCCurvesTest
    - Testing __SaveROCCurves()__ by first saving ROC-curve images (*/data/test_data/CFEAR/ROC.pdf*, */data/test_data/CorAl/ROC.pdf*), and then checking if the images exists.



Make tests 
```
catkin_make tests
```

Run __PythonClassifierInterface__ test
```
rosrun alignment_checker alignment_checker_python_classifier_test
```

Run __ScanLearningInterface__ test
```
rosrun alignment_checker alignment_checker_scan_learning_test
```