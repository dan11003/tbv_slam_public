#pragma once
#include "alignment_checker/AlignmentQuality.h"
#include "alignment_checker/ScanType.h"
#include "map"
#include <memory.h>
#include <Eigen/Dense>
#include <pybind11/embed.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <ros/package.h>

#define CFEAR_COST "CFEAR"
#define CORAL_COST "coral"
#define COMBINED_COST "alignment_quality"

namespace py = pybind11;
using namespace pybind11::literals; 


//!*
//! Binary classification only, can be used for multiple tasks
//!*/
namespace CorAlignment{
using std::cout; using std::cerr; using std::endl;

template<typename Derived>
inline bool is_finite(const Eigen::MatrixBase<Derived>& x)
{
  return ( (x - x).array() == (x - x).array()).all();
}

class PythonClassifierInterface{

protected:
public:

  PythonClassifierInterface();

  //!*
  //! Interface of binary classification
  //*

  virtual void fit() = 0;

  Eigen::VectorXd predict_proba() {return predict_proba(X_);} // X_{n x m}. n rows samples, m quality measures. return y_pred_{n x 1}

  Eigen::VectorXd predict_proba(const Eigen::MatrixXd& X); // X_{n x m}. n rows samples, m quality measures. return y_pred_{n x 1}

  Eigen::VectorXd predict(const Eigen::MatrixXd& X);

  Eigen::VectorXd predict(){return this->predict(this->X_);} // Uses predict proba internally, rahter then reimplementing it

  void AddDataPoint(Eigen::MatrixXd X_i, Eigen::VectorXd y_i); // extends X_ and y_ with an additional datapoint(s)

  double Accuracy() {return Accuracy(this->y_, predict());};

  double Accuracy(const Eigen::VectorXd& y_true, const Eigen::VectorXd& y_pred);

  Eigen::MatrixXd ConfusionMatrix() {return ConfusionMatrix(this->y_, predict());};

  Eigen::MatrixXd ConfusionMatrix(const Eigen::VectorXd& y_true, const Eigen::VectorXd& y_pred);

  //! INPUT /OUTPUT
  //! \brief LoadData containing rows of [x_{i,:} y_i]
  //! \param path
  //!

  void LoadData(const std::string& path); //

  virtual void LoadCoefficients(const std::string& path) = 0;

  //!
  //! \brief SaveData
  //! \param path
  //!
  void SaveData(const std::string& path); // Containing rows of [x_{i,:} y_i]

  virtual void SaveCoefficients(const std::string& path) = 0; // to avoid learning of the model

  bool DataValid();

  void SaveROCCurve(const std::string& path, const std::string& file_name = "ROC");

  //! Members

  Eigen::MatrixXd X_; // training data
  Eigen::VectorXd y_; // training labels

  bool IsFit(){ return is_fit_; }

  protected:
  bool is_fit_ = false;

  //Pybind11 objects
  static py::scoped_interpreter guard_;
  py::module numpy_;
  py::object py_clf_;



};

class LogisticRegression : public PythonClassifierInterface{

public:
    LogisticRegression();

    void fit();

    void LoadCoefficients(const std::string& path); 
    void SaveCoefficients(const std::string& path); 

    Eigen::VectorXd predict_linear(const Eigen::MatrixXd& X);

private:

Eigen::VectorXd coef_;
double intercept_;
};




// Encapsulates CorAl specific tasks


class ScanLearningInterface{
  public:

  typedef struct scan
  {
    Eigen::Affine3d T;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cld;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cldPeaks;
    CFEAR_Radarodometry::MapNormalPtr CFEAR;

    void Print(){
      std::string scld   = (cld==nullptr) ? "Null" : std::to_string(cld->size());
      std::string speak  = (cldPeaks==nullptr) ? "Null" : std::to_string((int)cldPeaks->size());
      std::string scfear = (CFEAR==nullptr) ? "Null" : std::to_string(CFEAR->GetSize());

      cout << "Cld:   " << scld << endl;
      cout << "Peak:  " << speak << endl;
      cout << "CFEAR: " << scfear << endl;
    }
  }s_scan;

  ScanLearningInterface();

  //!
  //! \brief AddTrainingData perform synthetic missalignment and adds positive and nevative training data
  //! \param T pose of current scan
  //! \param cloud Current Scan non-peaks point cloud
  //! \param cloud_peaks peaks in current point cloud
  //! \param CFEARScan CFEAR features
  //!
  void AddTrainingData(const s_scan& current);

  //!
  //! \brief PredAlignment produce probability values for alignment for current and prev scans 
  //! \param current Current scan
  //! \param prev Previous scan
  //! \param quality Return parameter, result given by quality["CFEAR"] and quality["CorAl"]
  //!
  void PredAlignment(const s_scan& current, const s_scan& prev, std::map<std::string,double>& quality);

  void PredAlignment(const scan& current, const s_scan& prev, std::map<std::string,double>& quality, Eigen::MatrixXd& X_CorAl, Eigen::MatrixXd& X_CFEAR, bool& valid);

  //!
  //! \brief FitModels fits models using given model type
  //! \param model classification model (currently supported: LogisticRegression, DecisionTreeClassifier)
  //!
  void FitModels(const std::string& model = "LogisticRegression");

  //!
  //! \brief LoadData loads data from dir/CFEAR.txt and dir/CorAL.txt
  //! \param dir
  //!  
  void LoadData(const std::string& dir);
  void LoadCoefficients(const std::string& dir); 

  //!
  //! \brief SaveData Stores dir/CFEAR.txt and dir/CorAL.txt
  //! \param dir
  //!
  void SaveData(const std::string& dir);
  void SaveCoefficients(const std::string& dir); 

  //!
  //! \brief SaveROCCurves Generate and save ROC-curves in dir/CFEAR and dir/CorAl
  //! \param dir
  //!
  void SaveROCCurves(const std::string& dir);

  private:
  const double range_error_ = 0.5;
  const double min_dist_btw_scans_ = 0.5;
  
  Eigen::MatrixXd getCorAlQualityMeasure(const s_scan& current, const s_scan& prev, bool& valid, const Eigen::Affine3d Tperturbation = Eigen::Affine3d::Identity());
  Eigen::MatrixXd getCFEARQualityMeasure(const s_scan& current, const s_scan& prev, bool& valid, const Eigen::Affine3d Tperturbation = Eigen::Affine3d::Identity());

  void CreatePerturbations();

  // Python classifiers for CFEAR and CorAl data
  std::unique_ptr<LogisticRegression> cfear_class, coral_class, combined_class;
  s_scan prev_;
  unsigned int frame_ = 0;
  bool small_erors_ = true, medium_errors_ = true, large_errors_ = true;
  bool combined_ = true;

  const double small_th_err = 0.5*M_PI/180.0;
  const double medium_th_err = 2*M_PI/180.0;
  const double large_th_err = 15*M_PI/180.0;
  std::vector< std::vector<double> >vek_perturbation_;

  const bool visualize_ = false;

};


}
