#include "alignment_checker/alignmentinterface.h"
namespace CorAlignment {

py::scoped_interpreter PythonClassifierInterface::guard_;

PythonClassifierInterface::PythonClassifierInterface(){
    py::module sys = py::module::import("sys");
    py::print(sys.attr("version"));

    // Import sklearn.linear_model
    this->numpy_ = py::module::import("numpy");
}


void PythonClassifierInterface::fit(){
}

void PythonClassifierInterface::LoadCoefficients(const std::string& path){}
void PythonClassifierInterface::SaveCoefficients(const std::string& path){}

Eigen::VectorXd PythonClassifierInterface::predict_proba(const Eigen::MatrixXd& X){
    // If model is not fitted (must run fit() before)
    if(!is_fit_){
        std::cout << "Warning! Model is not fitted yet. Return probability as zero(s)" << std::endl;
        Eigen::VectorXd y_prob(X.rows());
        y_prob.setZero();
        return y_prob;
    }
    auto np_X = py::cast(X);
    auto result = this->py_clf_.attr("predict_proba")(np_X);
    auto y_prob = numpy_.attr("delete")(result, 0, 1);
    return y_prob.cast<Eigen::VectorXd>();
}


Eigen::VectorXd PythonClassifierInterface::predict(const Eigen::MatrixXd& X){
    // If model is not fitted (must run fit() before)
    if(!is_fit_){
        std::cout << "Warning! Model is not fitted yet. Return probability as zero(s)" << std::endl;
        Eigen::VectorXd y_prob(X.rows());
        y_prob.setZero();
        return y_prob;
    }
    auto np_X = py::cast(X);
    auto y_pred = this->py_clf_.attr("predict")(np_X);
    return y_pred.cast<Eigen::VectorXd>();
}


void PythonClassifierInterface::AddDataPoint(Eigen::MatrixXd X_i, Eigen::VectorXd y_i){
    if(X_.rows() == 0){
        X_ = X_i;
    }else{
        Eigen::MatrixXd X_temp = X_;
        this->X_.conservativeResize(this->X_.rows() + X_i.rows(), X_i.cols());
        this->X_<< X_temp, X_i;
    }

    if(y_.rows() == 0){
        y_ = y_i;
    }else{
        Eigen::MatrixXd y_temp = y_;
        this->y_.conservativeResize(this->y_.rows() + y_i.rows(), 1);
        this->y_<< y_temp, y_i;
    }
}


double PythonClassifierInterface::Accuracy(const Eigen::VectorXd& y_true, const Eigen::VectorXd& y_pred){
    if(y_true.rows() != y_pred.rows()){
        std::cout << "y_true and y_pred must be equal length!" << std::endl;
        return -1;
    }else if(y_true.rows() == 0){
        std::cout << "Input vectors length can't be zero!" << std::endl;
        return -1;
    }
    auto sklearn_ = py::module::import("sklearn.metrics");
    auto results = sklearn_.attr("balanced_accuracy_score")(py::cast(y_true), py::cast(y_pred));
    return results.cast<double>();
}


Eigen::MatrixXd PythonClassifierInterface::ConfusionMatrix(const Eigen::VectorXd& y_true, const Eigen::VectorXd& y_pred){
    if(y_true.rows() != y_pred.rows()){
        std::cout << "y_true and y_pred must be equal length!" << std::endl;
        return Eigen::Matrix2d::Zero();
    }else if(y_true.rows() == 0){
        std::cout << "Input vectors length can't be zero!" << std::endl;
        return Eigen::Matrix2d::Zero();
    }
    auto sklearn_ = py::module::import("sklearn.metrics");
    auto results = sklearn_.attr("confusion_matrix")(py::cast(y_true), py::cast(y_pred));
    return results.cast<Eigen::MatrixXd>();
}

void PythonClassifierInterface::LoadData(const std::string& path){
    std::ifstream file;
    file.open(path);
    std::string line;
    std::vector<double> X_values;
    std::vector<double> y_values;
    unsigned int rows = 0;

    // Load vectors with values from file
    while (std::getline(file, line)){
        std::stringstream line_stream(line);
        std::string value;

        std::getline(line_stream, value, ',');
        y_values.push_back(std::stod(value));
        while (std::getline(line_stream, value, ','))
            X_values.push_back(std::stod(value));
        ++rows;
    }

    // Load matrices with values from vectors
    if(rows > 0){
        const int cols = X_values.size() / rows;
        this->X_.resize(rows, cols);
        this->y_.resize(rows, 1);
        for (int i = 0; i < rows; i++){
            this->y_(i) = y_values.at(i);
            for (int j = 0; j < cols; j++)
                this->X_(i,j) = X_values.at(cols*i+j);
        }
        std::cout << "Loaded training data from " << path << std::endl;
    }
    else
        std::cout << "No training data in " << path << std::endl;
}


void PythonClassifierInterface::SaveROCCurve(const std::string& save_path, const std::string& file_name){
    if(X_.rows()==0 && y_.rows()==0){
        std::cout << "No training data..." << std::endl;
        return;
    }
    // Finds path to ../python/utils.py
    std::string python_path = ros::package::getPath("alignment_checker") + "/python";

    // Import utils.py
    auto path = py::module::import("sys").attr("path").attr("append")(python_path);
    auto utils = py::module::import("utils");

    utils.attr("SaveROC")(py::cast(X_), py::cast(y_), save_path, file_name);
}


void PythonClassifierInterface::SaveData(const std::string& path){
    std::ofstream result_file;
    result_file.open(path, std::ofstream::out);

    if(!result_file.is_open()){
        std::cout << "Could not save training data in " << path << std::endl;
        return;
    }

    // Loop over rows and columns in saved data X_ and y_
    const int cols = X_.cols();
    for(int i = 0; i < this->y_.rows(); i++){
        result_file << y_(i);
        for (int j = 0; j < cols; j++){
            result_file << "," << X_(i,j);
        }
        result_file << std::endl;
    }
    result_file.close();

    std::cout << "Saved training data in " << path << std::endl;
}

bool PythonClassifierInterface::DataValid(){
    if(this->X_.rows() != this->y_.rows()){
        cout << "Number of examples does not match for features and labels!" << endl;
        return false;
    }
    else if(!is_finite(X_) || !is_finite(y_)){
        cout << "Matrix not finite: " << endl;
        return false;
    }else if(1 > this->y_.rows()){
        cout << "No examples in training data!" << endl;
        return false;
    }
    return true;
}


LogisticRegression::LogisticRegression(){

}

void LogisticRegression::fit(){

    if(!DataValid()){
        exit(0);
    }

    // Creating copies, might better to use references instead...
    auto np_y = py::cast(this->y_);
    auto np_X = py::cast(this->X_);

    auto sklearn_ = py::module::import("sklearn.linear_model");
    this->py_clf_ = sklearn_.attr("LogisticRegression")("class_weight"_a="balanced", "max_iter"_a=1000).attr("fit")(np_X, np_y);

    /*else if(model == "DecisionTreeClassifier"){
        auto sklearn_ = py::module::import("sklearn.tree");
        this->py_clf_ = sklearn_.attr("DecisionTreeClassifier")("class_weight"_a="balanced", "max_iter"_a=1000).attr("fit")(np_X, np_y);
    }*/
    this->is_fit_ = true;
    std::cout << "Fitted logistic regression model after training data" << std::endl;
    std::cout << "Accuracy:\n" << Accuracy() << std::endl << "Confusion matrix:\n" << ConfusionMatrix() << std::endl;

    // cout << this->X_.block(0,0,13,X_.cols()) << endl;
    auto c_result = this->py_clf_.attr("coef_");
    coef_ =  c_result.cast<Eigen::MatrixXd>().row(0).transpose();
    auto i_result = this->py_clf_.attr("intercept_");
    intercept_ = i_result.cast<Eigen::MatrixXd>()(0,0);
    cout <<"intercept: " << intercept_ << endl;
    cout <<"coef: " <<  coef_ << endl;
    //Eigen::VectorXd  score = predict_linear(this->X_);
    //cout <<"scores: " << score << endl;
}

void LogisticRegression::LoadCoefficients(const std::string& path) {
    auto sklearn_ = py::module::import("sklearn.linear_model");
    this->py_clf_ = sklearn_.attr("LogisticRegression")("class_weight"_a="balanced", "max_iter"_a=1000);
    std::ifstream file;
    try {
        file.open(path);
    }
    catch (std::ios_base::failure& e) {
        std::cerr << e.what() << '\n';
    }
    std::string line;
    std::vector<double> coefs;
    unsigned int rows = 0;

    // Load vectors with values from file
    while (std::getline(file, line)){
        std::stringstream line_stream(line);
        std::string value;

        std::getline(line_stream, value, ',');
        intercept_ = std::stod(value);
        while (std::getline(line_stream, value, ',')) {
            coefs.push_back(std::stod(value));
        }
    }
    coef_ = Eigen::Map<Eigen::VectorXd>(coefs.data(), coefs.size());
    this->py_clf_.attr("coef_") = coef_;
    this->py_clf_.attr("intercept_") = intercept_;
    is_fit_ = true;
}

void LogisticRegression::SaveCoefficients(const std::string& path){
    std::cout << "Save coefficients!"<<std::endl;
    std::ofstream file(path);
    if (file.is_open())
    {
        file << intercept_ << ",";
        for (int i = 0; i < coef_.size(); i++) {
            file << coef_(i);
            if (i+1 != coef_.size()) {
                file<< ",";
            }
        }
        file << "\n";
    }
}

Eigen::VectorXd LogisticRegression::predict_linear(const Eigen::MatrixXd& X){

    Eigen::VectorXd score(X.rows());
    for(int i = 0; i < X.rows() ; i++){
        const Eigen::VectorXd x = X.block(i,0,1,X.cols()).transpose();
        score(i) = coef_.dot(x) + intercept_;
    }
    return score;
}
/******************* END PythonClassifierInterface *************************/






/******************* ScanLearningInterface *************************/

ScanLearningInterface::ScanLearningInterface() {
    CreatePerturbations();
    cfear_class = std::make_unique<LogisticRegression>(LogisticRegression());
    coral_class = std::make_unique<LogisticRegression>(LogisticRegression());
    combined_class = std::make_unique<LogisticRegression>(LogisticRegression());
}

void ScanLearningInterface::AddTrainingData(const s_scan& current){
    // If no previous scan
    if (this->frame_++ == 0){
        this->prev_ = current;
        return;
    }

    // If below minimum distance to previous scan
    const double distance_btw_scans = (current.T.translation() - prev_.T.translation()).norm();
    if (distance_btw_scans < min_dist_btw_scans_){
        return;
    }

    ros::Time t0 = ros::Time::now();
    for(auto && verr : vek_perturbation_)
    {
        const Eigen::Affine3d Tperturbation = VectorToAffine3dxyez(verr);

        double sum = 0;
        for(auto && e : verr)
            sum+=fabs(e);
        bool aligned = sum < 0.0001;

        Eigen::VectorXd y(1);
        y(0) = aligned;

        bool valid1 = false, valid2 = false;;
        Eigen::MatrixXd X_CorAl = this->getCorAlQualityMeasure(current, prev_, valid1, Tperturbation); /* CorAl */
        Eigen::MatrixXd X_CFEAR = this->getCFEARQualityMeasure(current, prev_, valid2, Tperturbation);/* CFEAR */
        //cout << X_CFEAR << ", " << y(0) << endl;
        if(combined_){
            Eigen::MatrixXd X_combined(1, X_CorAl.cols() + X_CFEAR.cols());
            X_combined <<X_CorAl, X_CFEAR ;
            this->combined_class->AddDataPoint(X_combined, y);
        }
        else{
            this->coral_class->AddDataPoint(X_CorAl, y);
            this->cfear_class->AddDataPoint(X_CFEAR, y);
        }

        // Visualize training data (point clouds with and without perturbations)
        if(visualize_){
            auto temp_cloud = current.cldPeaks;
            AlignmentQualityPlot::PublishCloud("demo/pertubation_cloud", prev_.cldPeaks, prev_.T * Tperturbation, "prev");
            AlignmentQualityPlot::PublishCloud("demo/current_cloud", temp_cloud, current.T, "curr");
            ros::Duration(0.25).sleep();
        }
    }
    this->prev_ = current;
    ros::Time t1 = ros::Time::now();
    CFEAR_Radarodometry::timing.Document("Add training data",CFEAR_Radarodometry::ToMs(t1-t0));
}

void ScanLearningInterface::PredAlignment(const scan& current, const s_scan& prev, std::map<std::string,double>& quality, Eigen::MatrixXd& X_CorAl, Eigen::MatrixXd& X_CFEAR, bool& valid){

    bool valid1 = false, valid2 = false;
    X_CorAl = this->getCorAlQualityMeasure(current, prev, valid, Eigen::Affine3d::Identity());
    X_CFEAR = this->getCFEARQualityMeasure(current, prev, valid, Eigen::Affine3d::Identity());

    if(combined_){
        Eigen::MatrixXd X_combined(1, X_CorAl.cols() + X_CFEAR.cols());
        X_combined << X_CorAl, X_CFEAR ;
        Eigen::VectorXd y_combined = this->combined_class->predict_linear(X_combined);
        quality[COMBINED_COST] = y_combined(0);
    }
    else{
        Eigen::VectorXd y_CorAl = this->coral_class->predict_proba(X_CorAl);
        Eigen::VectorXd y_CFEAR = this->cfear_class->predict_proba(X_CFEAR);
        quality[CORAL_COST] = y_CorAl(0);
        quality[CFEAR_COST] = y_CFEAR(0);
    }
    valid = valid1 && valid2;
}
void ScanLearningInterface::PredAlignment(const scan& current, const s_scan& prev, std::map<std::string,double>& quality){
    Eigen::MatrixXd X_CorAl;
    Eigen::MatrixXd X_CFEAR;
    bool valid;
    PredAlignment(current, prev, quality, X_CorAl, X_CFEAR, valid);
}


void ScanLearningInterface::LoadData(const std::string& dir){
    if(combined_){
        this->combined_class->LoadData(dir + "/combined.txt");
    }else{
        this->coral_class->LoadData(dir + "/CorAl.txt");
        this->cfear_class->LoadData(dir + "/CFEAR.txt");
    }
}


void ScanLearningInterface::SaveData(const std::string& dir){
    if(combined_){
        this->combined_class->SaveData(dir + "/combined.txt");
    }else {
        this->coral_class->SaveData(dir + "/CorAl.txt");
        this->cfear_class->SaveData(dir + "/CFEAR.txt");
    }
}

void ScanLearningInterface::LoadCoefficients(const std::string& dir){
    if(combined_){
        this->combined_class->LoadCoefficients(dir + "trained_alignment_classifier.txt");
    }else{
        this->coral_class->LoadCoefficients(dir + "trained_alignment_classifier_CorAl.txt");
        this->cfear_class->LoadCoefficients(dir + "trained_alignment_classifier_CFEAR.txt");
    }
}

void ScanLearningInterface::SaveCoefficients(const std::string& dir){
    if(combined_){
        this->combined_class->SaveCoefficients(dir + "/trained_alignment_classifier.txt");
    }else {
        this->coral_class->SaveCoefficients(dir + "/trained_alignment_classifier_CorAl.txt");
        this->cfear_class->SaveCoefficients(dir + "/trained_alignment_classifier_CFEAR.txt");
    }
}

void ScanLearningInterface::SaveROCCurves(const std::string& save_path){
    if(combined_){
        this->combined_class->SaveROCCurve(save_path, "CorAl_ROC");
    }else{
        this->coral_class->SaveROCCurve(save_path, "CorAl_ROC");
        this->cfear_class->SaveROCCurve(save_path, "CFEAR_ROC");
    }
}

void ScanLearningInterface::FitModels(const std::string& model){
    if(combined_){
        cout << "Fit CFEAR-CorAl model" << endl;
        this->combined_class->fit();
    }
    else{
        cout << "Fit CorAl model" << endl;
        this->coral_class->fit();
        cout << "Fit CFEAR model" << endl;
        this->cfear_class->fit();
    }
}


Eigen::MatrixXd ScanLearningInterface::getCorAlQualityMeasure(const s_scan& current, const s_scan& prev, bool& valid, const Eigen::Affine3d Toffset){
    CorAlignment::PoseScan::Parameters posescan_par; posescan_par.scan_type = CorAlignment::scan_type::kstrongStructured; posescan_par.compensate = false;
    CorAlignment::PoseScan_S scan_curr = CorAlignment::PoseScan_S(new CorAlignment::kstrongStructuredRadar(posescan_par, current.cldPeaks, current.T, Eigen::Affine3d::Identity()));
    CorAlignment::PoseScan_S scan_prev = CorAlignment::PoseScan_S(new CorAlignment::kstrongStructuredRadar(posescan_par, prev.cldPeaks, prev.T, Eigen::Affine3d::Identity()));

    AlignmentQuality::parameters quality_par;

    quality_par.method = "Coral"; quality_par.radius = 1.0; quality_par.weight_res_intensity = false; quality_par.output_overlap = true;

    AlignmentQuality_S quality_type = AlignmentQualityFactory::CreateQualityType(scan_curr, scan_prev, quality_par, Toffset);
    auto quality = quality_type->GetQualityMeasure();

    Eigen::MatrixXd quality_measure = Eigen::Map<Eigen::MatrixXd>(quality.data(), 1, quality.size());
    //cout <<"coral: "<<  quality_measure << endl;
    valid = quality_type->valid_;
    //cout << "CorAl - quality" << valid << endl;
    //cout << quality_measure << endl;

    return quality_measure;
}


Eigen::MatrixXd ScanLearningInterface::getCFEARQualityMeasure(const s_scan& current, const s_scan& prev, bool& valid, const Eigen::Affine3d Toffset){
    CorAlignment::PoseScan::Parameters posescan_par; posescan_par.scan_type = CorAlignment::scan_type::cfear; posescan_par.compensate = false;
    CorAlignment::PoseScan_S scan_curr = CorAlignment::PoseScan_S(new CorAlignment::CFEARFeatures(posescan_par, current.CFEAR, current.T, Eigen::Affine3d::Identity()));
    CorAlignment::PoseScan_S scan_prev = CorAlignment::PoseScan_S(new CorAlignment::CFEARFeatures(posescan_par, prev.CFEAR, prev.T, Eigen::Affine3d::Identity()));

    AlignmentQuality::parameters quality_par;
    quality_par.method = "P2L";
    // quality_par.weight_res_intensity = true
    AlignmentQuality_S quality_type = AlignmentQualityFactory::CreateQualityType(scan_curr, scan_prev, quality_par, Toffset);

    auto quality = quality_type->GetQualityMeasure();
    const Eigen::MatrixXd quality_measure = Eigen::Map<Eigen::MatrixXd>(quality.data(), 1, quality.size());
    //cout <<"cfear: "<<  quality_measure << endl;
    //cout << quality_measure << endl;
    valid = quality_type->valid_;
    //cout << "CFEAR - quality: " <<valid<< endl;
    //cout << quality_measure << endl;
    return quality_measure;
}

void ScanLearningInterface::CreatePerturbations(){

    const std::vector<std::vector<double>> vek_aligned = { /*aligned*/{0, 0, 0}};
    const std::vector<std::vector<double>> vek_small_error = { {range_error_, 0, small_th_err}, {0, range_error_, small_th_err}, {-range_error_, 0, small_th_err}, {0, -range_error_, small_th_err} };
    const std::vector<std::vector<double>> vek_medium_error = {{2*range_error_, 0, medium_th_err}, {0, 2*range_error_, medium_th_err}, {-2*range_error_, 0, medium_th_err}, {0, -2*range_error_, medium_th_err}};
    const std::vector<std::vector<double>> vek_large_error = {{4*range_error_, 0, large_th_err}, {0, 4*range_error_, large_th_err}, {-4*range_error_, 0, large_th_err}, {0, -4*range_error_, large_th_err}};
    vek_perturbation_ = vek_aligned;
    if(small_erors_){
        vek_perturbation_.insert(vek_perturbation_.end(), vek_small_error.begin(), vek_small_error.end());
    }
    if(medium_errors_){
        vek_perturbation_.insert(vek_perturbation_.end(), vek_medium_error.begin(), vek_medium_error.end());
    }
    if(large_errors_){
        vek_perturbation_.insert(vek_perturbation_.end(), vek_large_error.begin(), vek_large_error.end());
    }
}

}
