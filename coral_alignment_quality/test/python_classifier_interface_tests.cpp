#include "alignment_checker/alignmentinterface.h"
#include <gtest/gtest.h>
#include <ros/package.h>

/* PythonClassifierInterface tests */

class PythonClassifierInterfaceTest : public ::testing::Test {
 protected:

  void SetUp() override {
    if (!boost::filesystem::exists(data_path))
      boost::filesystem::create_directory(data_path);

    Eigen::MatrixXd X_training(6,1);
    X_training << 1,2,3,4,5,6;

    Eigen::VectorXd y(6);
    y << 0,0,0,1,1,1;

    python_classifier.AddDataPoint(X_training, y);
  }
  const std::string data_path = ros::package::getPath("alignment_checker") + "/data/test_data/";
  CorAlignment::PythonClassifierInterface python_classifier;
};

TEST_F(PythonClassifierInterfaceTest, logisticRegressionPredictTest){
  python_classifier.fit("LogisticRegression");

  Eigen::VectorXd X_test(2,1);
  X_test << 3,4;

  // Get prediction class (0 or 1)
  Eigen::VectorXd y_pred = python_classifier.predict(X_test);

  EXPECT_EQ(y_pred(0), 0);
  EXPECT_EQ(y_pred(1), 1);
}

TEST_F(PythonClassifierInterfaceTest, logisticRegressionPredictProbaTest){
  python_classifier.fit("LogisticRegression");

  Eigen::VectorXd X_test(2,1);
  X_test << 3,4;

  // Get probability value [0,1]
  Eigen::VectorXd y_prob = python_classifier.predict_proba(X_test);

  EXPECT_LT(y_prob(0), 0.5);
  EXPECT_GT(y_prob(1), 0.5);
}

TEST_F(PythonClassifierInterfaceTest, decisionTreePredictTest){
  python_classifier.fit("DecisionTreeClassifier");

  Eigen::VectorXd X_test(2,1);
  X_test << 3,4;

  // Get prediction class (0 or 1)
  Eigen::VectorXd y_pred = python_classifier.predict(X_test);
  
  EXPECT_EQ(y_pred(0), 0);
  EXPECT_EQ(y_pred(1), 1);
}

TEST_F(PythonClassifierInterfaceTest, decisionTreePredictProbaTest){
  python_classifier.fit("DecisionTreeClassifier");

  Eigen::VectorXd X_test(2,1);
  X_test << 3,4;

  // Get probability value [0,1]
  Eigen::VectorXd y_prob = python_classifier.predict_proba(X_test);

  EXPECT_LT(y_prob(0), 0.5);
  EXPECT_GT(y_prob(1), 0.5);
}

TEST_F(PythonClassifierInterfaceTest, accuracyTest){
  Eigen::MatrixXd X_train(2,1);
  X_train << 2,5;
  Eigen::VectorXd y_train(2);
  y_train << 1,0;
  python_classifier.AddDataPoint(X_train, y_train);

  python_classifier.fit("LogisticRegression");
  EXPECT_LT(python_classifier.Accuracy(), 1);
  EXPECT_GT(python_classifier.Accuracy(), 0.5);
}

TEST_F(PythonClassifierInterfaceTest, saveAndLoadDataTest){
  // Save training data
  python_classifier.SaveData(data_path + "training_data.txt");

  // Load training data in new classifier
  CorAlignment::PythonClassifierInterface python_classifier_loaded;
  python_classifier_loaded.LoadData(data_path + "training_data.txt");

  // Fit models
  python_classifier.fit("LogisticRegression");
  python_classifier_loaded.fit("LogisticRegression");

  // Test data
  Eigen::VectorXd X_test(1);
  X_test(0) = 3;

  // Get probability for original and loaded classifier
  Eigen::VectorXd y_pred = python_classifier.predict_proba(X_test);
  Eigen::VectorXd y_pred_loaded = python_classifier_loaded.predict_proba(X_test);

  // Compare results
  EXPECT_FLOAT_EQ(y_pred(0), y_pred_loaded(0));
}

TEST_F(PythonClassifierInterfaceTest, saveROCCurveTest){
  python_classifier.SaveROCCurve(data_path);

  EXPECT_TRUE(boost::filesystem::exists(data_path + "ROC.pdf"));
  EXPECT_TRUE(boost::filesystem::exists(data_path + "ROC.png"));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_python_classifier_interface");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}