#include "alignment_checker/alignmentinterface.h"
#include "cfear_radarodometry/types.h"
#include <gtest/gtest.h>
#include <ros/package.h>

/* ScanLearninigInterface tests */

class ScanLearninigInterfaceTest : public ::testing::Test {
 protected:

  void SetUp() override {
    if (!boost::filesystem::exists(data_path))
      boost::filesystem::create_directory(data_path);

    const std::string simple_graph_path = ros::package::getPath("alignment_checker") + "/data/simple_graph.sgh";

    CFEAR_Radarodometry::simple_graph sg;
    CFEAR_Radarodometry::LoadSimpleGraph(simple_graph_path, sg);

    for(int i = 0; i < sg.size()-1; i++){
      prev = current;
      CFEAR_Radarodometry::RadarScan scan = sg.at(i).first;
			current.T = scan.GetPose();
      current.cld = scan.cloud_nopeaks_;
      current.cldPeaks = scan.cloud_peaks_;
      current.CFEAR = scan.cloud_normal_;

      scan_learner.AddTrainingData(current);
    }
    scan_learner.FitModels("LogisticRegression");
    
  }
  const std::string data_path = ros::package::getPath("alignment_checker") + "/data/test_data/";
  CorAlignment::ScanLearningInterface::s_scan current, prev;
  CorAlignment::ScanLearningInterface scan_learner;
};


TEST_F(ScanLearninigInterfaceTest, predAlignmentTest){
  std::map<std::string,double> quality, quality_offset;
  scan_learner.PredAlignment(current, prev, quality);

  current.T.translate(Eigen::Vector3d(1,1,0));
  scan_learner.PredAlignment(current, prev, quality_offset);

  EXPECT_GT(quality["Coral"], quality_offset["Coral"]);
  EXPECT_GT(quality["CFEAR"], quality_offset["CFEAR"]);
}


TEST_F(ScanLearninigInterfaceTest, saveAndLoadDataTest){
  scan_learner.SaveData(data_path);

  CorAlignment::ScanLearningInterface scan_learner_loaded;
  scan_learner_loaded.LoadData(data_path);

  scan_learner.FitModels("LogisticRegression");
  scan_learner_loaded.FitModels("LogisticRegression");
  
  std::map<std::string,double> quality, quality_loaded;

  scan_learner.PredAlignment(current, prev, quality);
  scan_learner_loaded.PredAlignment(current, prev, quality_loaded);

  EXPECT_FLOAT_EQ(quality_loaded["CorAl"], quality["CorAl"]);
  EXPECT_FLOAT_EQ(quality_loaded["CFEAR"], quality["CFEAR"]);
}

TEST_F(ScanLearninigInterfaceTest, saveROCCurvesTest){
  scan_learner.SaveROCCurves(data_path);

  EXPECT_TRUE(boost::filesystem::exists(data_path + "/CorAl_ROC.pdf"));
  EXPECT_TRUE(boost::filesystem::exists(data_path + "/CorAl_ROC.png"));

  EXPECT_TRUE(boost::filesystem::exists(data_path + "/CFEAR_ROC.pdf"));
  EXPECT_TRUE(boost::filesystem::exists(data_path + "/CFEAR_ROC.png"));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_scan_learner_interface");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}