#include "alignment_checker/Utils.h"

namespace CorAlignment {

void SetScanLocations(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds,  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > &poses){
  for(int i = 0 ; i<clouds.size() ; i++)
    SetScanLocation(clouds[i], poses[i]);
}
void SetScanLocation(pcl::PointCloud<pcl::PointXYZ>::Ptr  &cloud,  Eigen::Affine3d &pose){
  cloud->sensor_origin_ = Eigen::Vector4f(pose.translation()(0), pose.translation()(1), pose.translation()(2), 1);
}

void FilterCloudsByDistance(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds, std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > &poses, double radius){
  float squaredradius = radius*radius;
  for(int i=0;i<clouds.size();i++){
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ p_sensor;
    p_sensor.x = poses[i].translation()(0);
    p_sensor.y = poses[i].translation()(1);
    p_sensor.z = poses[i].translation()(2);

    for(auto j : clouds[i]->points){
      if((j.x-p_sensor.x)*(j.x-p_sensor.x) + (j.y-p_sensor.y)*(j.y-p_sensor.y) + (j.z-p_sensor.z)*(j.z-p_sensor.z) > squaredradius)
        tmp->push_back(j);
    }
    clouds[i] = tmp;
  }
}

void ReadPosesFromFile(const std::string &filepath, std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > &poses){


  cout<<"Opening file: "<<filepath<<endl;

  string line;
  int index =0;
  std::ifstream myfile (filepath);
  if (myfile.is_open()){
    while ( getline (myfile,line) ){
      std::vector<double> pose_compoments;
      std::vector<std::string> tokens;
      boost::split( tokens, line, boost::is_any_of(" ") );
      for(int i=1;i<tokens.size();i++){
        pose_compoments.push_back((double)atof(tokens[i].c_str()));
      }
      Eigen::Affine3d pose = TransRotvectorToAffine3d(pose_compoments);
      poses.push_back(pose);
    }
    myfile.close();
  }
  else{
    std::cout<<"couldn't open file"<<endl;
    exit(0);
  }
}
void ReadCloudsFromFile(const std::string directory, const std::string prefix, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds, int start_index){

  std::string filepath=directory+"/"+prefix;
  int count = start_index;
  cout<<"Searching for point clouds at :"<<filepath+std::to_string(count)+".pcd"<<std::endl;

  while(ros::ok()){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(filepath+std::to_string(count++)+".pcd", *cloud) != -1 )
      clouds.push_back(cloud);
    else
      break;
  }
}

Eigen::Affine3d TransRotvectorToAffine3d(const std::vector<double> &v) {
  Eigen::Quaterniond q(v[6], v[3], v[4], v[5]);
  Eigen::Affine3d T;
  T.linear()=q.toRotationMatrix();
  T.translation()<<v[0], v[1], v[2];
  return T;
}

Eigen::Affine3d VectorToAffine3dxyez(const std::vector<double>& vek){
  assert(vek.size()==3);
  return VectorToAffine3dxyez(vek[0], vek[1], vek[2]);
}
Eigen::Affine3d VectorToAffine3d(const Eigen::Matrix<double, 6,1> &v) {

  return Eigen::Translation<double, 3>(v(0), v(1), v(2)) *
      Eigen::AngleAxis<double>(v(3), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxis<double>(v(4), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxis<double>(v(5), Eigen::Vector3d::UnitZ());
}

Eigen::Affine3d VectorToAffine3dxyez(double x, double y, double theta) {

  return Eigen::Translation<double, 3>(x,y,0) *
      Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxis<double>(theta, Eigen::Vector3d::UnitZ());
}

void SegmentGround(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &filtered, double height){
  unsigned int size_clouds=0;
  unsigned int filtered_size=0;
  for(int i=0;i<clouds.size();i++){

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_tmp(new pcl::PointCloud<pcl::PointXYZ>);
    filtered_tmp->header.frame_id=clouds[i]->header.frame_id;
    filtered_tmp->header.stamp=clouds[i]->header.stamp;
    for(int j=0 ; j<clouds[i]->size() ;j++){
      size_clouds++;
      if((*clouds[i])[j].z > height){
        filtered_tmp->push_back((*clouds[i])[j]);
        filtered_size++;
      }
    }
    filtered_size+=filtered_tmp->size();
    filtered.push_back(filtered_tmp);
  }

  //double ratio = size_clouds==0 ? 0 : (double)(filtered_size-size_clouds)/((double)size_clouds);
  // cout<<"Segmentation, from: "<<size_clouds<<" to "<<filtered_size<<endl;
}



void DownSampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr  &cloud, pcl::PointCloud<pcl::Normal>::Ptr  &normal,float voxelsize){
  //cout<<"Downsample from size "<<cloud->size();
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr to_filter( new pcl::PointCloud<pcl::PointXYZINormal>());
  pcl::PointXYZINormal p;
  p.intensity=0;
  for(int i = 0 ; i<cloud->size() ; i++){
    p.x=(*cloud)[i].x;
    p.y=(*cloud)[i].y;
    p.z=(*cloud)[i].z;
    memcpy(p.normal,(*normal)[i].normal,4*sizeof(float));
    to_filter->push_back(p);
  }

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_filtered( new pcl::PointCloud<pcl::PointXYZINormal>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr  tmp_cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::Normal>::Ptr  tmp_normal(new pcl::PointCloud<pcl::Normal>());
  pcl::VoxelGrid<pcl::PointXYZINormal> sor;
  sor.setInputCloud (to_filter);
  sor.setLeafSize (voxelsize, voxelsize, voxelsize);
  sor.filter (*cloud_filtered);

  pcl::PointXYZ p_tmp;
  pcl::Normal n;
  for(int i=0;i<cloud_filtered->size();i++){
    pcl::PointXYZINormal tmp =  (*cloud_filtered)[i];
    memcpy(p_tmp.data, tmp.data, 3*sizeof(float));
    memcpy(n.normal, tmp.normal, 4*sizeof(float));
    tmp_cloud_out->push_back(p_tmp);
    tmp_normal->push_back(n);
  }
  cloud=tmp_cloud_out;
  normal=tmp_normal;

  cout<<" to size "<<cloud->size()<<endl;
  //r.getIndices()
}
//



void FilterClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds, std::vector< pcl::PointCloud<pcl::Normal>::Ptr > &normals){
  unsigned int total=0,removed=0;
  for(int i=0;i<clouds.size();i++){
    total+=clouds[i]->size();
    pcl::PointCloud<pcl::Normal>::Ptr tmp_normal(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    tmp_cloud->header=clouds[i]->header;
    for(int j = 0 ; j<(*clouds[i]).size() ; j++){
      pcl::Normal n2j = (*normals[i])[j];
      if(n2j.normal_x==n2j.normal_x && n2j.normal_y==n2j.normal_y && n2j.normal_z==n2j.normal_z){
        tmp_normal->push_back(n2j);
        tmp_cloud->push_back( (*clouds[i])[j]);
      }
      else
        removed++;
    }
    clouds[i] = tmp_cloud;
    normals[i] = tmp_normal;
  }
  cout<<"of a total of "<<total<<" points, "<<removed<<" was removed"<<endl;
}

void PublishCloud(const std::string& topic, pcl::PointCloud<pcl::PointXYZ>& cld){
  static std::map<std::string,ros::Publisher> pubs;
  std::map<std::string, ros::Publisher>::iterator it = pubs.find(topic);
  if (it == pubs.end()){
    ros::NodeHandle nh("~");
    pubs[topic] =  nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(topic,100);
    it = pubs.find(topic);
  }
  //cout<<"publish to "<<topic<<endl;

  it->second.publish(cld);
}


pcl::PointCloud<pcl::PointXY>::Ptr pcl3dto2d(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, std::vector<double>& intensity){
  pcl::PointCloud<pcl::PointXY>::Ptr output = pcl::PointCloud<pcl::PointXY>::Ptr(new pcl::PointCloud<pcl::PointXY>());
  output->resize(input->size());
  intensity.resize(input->size());
  assert(input !=NULL);
  int index = 0;
  for (auto && p : input->points) {
    output->points[index].x = p.x;
    output->points[index].y = p.y;
    intensity[index++] = p.intensity;
  }
  return output;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr pclAddIntensity(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, const std::vector<double>& intensity){
  pcl::PointCloud<pcl::PointXYZI>::Ptr cld(new pcl::PointCloud<pcl::PointXYZI>());
  assert( intensity.size() == input->size() );
  cld->resize(input->size());
  for(int i=0;i<input->size();i++){
    cld->points[i].x = input->points[i].x;
    cld->points[i].y = input->points[i].y;
    cld->points[i].z = input->points[i].z;
    cld->points[i].intensity = intensity[i];
  }
  return cld;
}

double get_azimuth_index(const std::vector<double> &azimuths, double azimuth) {
    auto lower = std::lower_bound(azimuths.begin(), azimuths.end(), azimuth);
    double closest = std::distance(azimuths.begin(), lower);
    uint M = azimuths.size();
    if (closest >= M)
        closest = M - 1;

    if (azimuths[closest] < azimuth) {
        double delta = 0;
        if (closest < M - 1) {
            if (azimuths[closest + 1] == azimuths[closest])
                delta = 0.5;
            else
                delta = (azimuth - azimuths[closest]) / (azimuths[closest + 1] - azimuths[closest]);
        }
        closest += delta;
    } else if (azimuths[closest] > azimuth){
        double delta = 0;
        if (closest > 0) {
            if (azimuths[closest - 1] == azimuths[closest])
                delta = 0.5;
            else
                delta = (azimuths[closest] - azimuth) / (azimuths[closest] - azimuths[closest - 1]);
        }
        closest -= delta;
    }
    return closest;
}
void radar_polar_to_cartesian(const cv::Mat &polar_in, const std::vector<double> &azimuths_in, cv::Mat &cart_out,
    float radar_resolution, float cart_resolution, int cart_pixel_width, bool fix_wobble) {

    float cart_min_range = (cart_pixel_width / 2) * cart_resolution;
    if (cart_pixel_width % 2 == 0)
        cart_min_range = (cart_pixel_width / 2 - 0.5) * cart_resolution;

    cv::Mat map_x = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);
    cv::Mat map_y = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);

#pragma omp parallel for collapse(2)
    for (int j = 0; j < map_y.cols; ++j) {
        for (int i = 0; i < map_y.rows; ++i) {
            map_y.at<float>(i, j) = -1 * cart_min_range + j * cart_resolution;
        }
    }

#pragma omp parallel for collapse(2)
    for (int i = 0; i < map_x.rows; ++i) {
        for (int j = 0; j < map_x.cols; ++j) {
            map_x.at<float>(i, j) = cart_min_range - i * cart_resolution;
        }
    }






    cv::Mat range = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);
    cv::Mat angle = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);

    uint M = azimuths_in.size();
    double azimuth_step = (azimuths_in[M - 1] - azimuths_in[0]) / (M - 1);

#pragma omp parallel for collapse(2)
    for (int i = 0; i < range.rows; ++i) {
        for (int j = 0; j < range.cols; ++j) {
            float x = map_x.at<float>(i, j);
            float y = map_y.at<float>(i, j);
            float r = (sqrt(pow(x, 2) + pow(y, 2)) - radar_resolution / 2) / radar_resolution;
            if (r < 0)
                r = 0;
            range.at<float>(i, j) = r;
            float theta = atan2f(y, x);
            if (theta < 0)
                theta += 2 * M_PI;
            if (fix_wobble and radar_resolution == 0.0596) {  // fix wobble in CIR204-H data
                angle.at<float>(i, j) = get_azimuth_index(azimuths_in, theta);
            } else {
                angle.at<float>(i, j) = (theta - azimuths_in[0]) / azimuth_step;
            }
        }
    }
    // interpolate cross-over
    cv::Mat a0 = cv::Mat::zeros(1, polar_in.cols, CV_32F);
    cv::Mat aN_1 = cv::Mat::zeros(1, polar_in.cols, CV_32F);
    for (int j = 0; j < polar_in.cols; ++j) {
        a0.at<float>(0, j) = polar_in.at<float>(0, j);
        aN_1.at<float>(0, j) = polar_in.at<float>(polar_in.rows-1, j);
    }
    cv::Mat polar = polar_in.clone();
    //cv::vconcat(aN_1, polar, polar);
    //cv::vconcat(polar, a0, polar);
    //angle = angle + 1;
    // polar to cart warp
    cout<<"polar: "<<polar.size<<", cart: "<<cart_out.size<<", range: "<<range.size<<", angle: "<<angle.size<<endl;
    cv::remap(polar, cart_out, range, angle, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
}

void RotoTranslation(const cv::Mat& input, cv::Mat& output, const Eigen::Affine3d& T, const float image_res){
    Eigen::Vector3d pars;
    CFEAR_Radarodometry::Affine3dToEigVectorXYeZ(T,pars);
    cv::Point2f center((input.cols - 1) / 2.0, (input.rows - 1) / 2.0);
    cv::Mat rotation_matix = getRotationMatrix2D(center, pars(2), 1.0);
    cv::Mat rotated_image;
    warpAffine(input, rotated_image, rotation_matix, input.size());
    const float tx = (float)pars(0)/image_res;
    const float ty = (float)pars(1)/image_res;
    float warp_values[] = { 1.0, 0.0, tx, 0.0, 1.0, ty };

    cv::Mat translation_matrix = cv::Mat(2, 3, CV_32F, warp_values);
    cv::warpAffine(rotated_image, output, translation_matrix, rotated_image.size());

}
cv_bridge::CvImagePtr CreateImage(cv_bridge::CvImagePtr ref){
    cv_bridge::CvImagePtr tmp_new = boost::make_shared<cv_bridge::CvImage>();
    tmp_new->header =   ref->header;
    tmp_new->encoding = ref->encoding;
    return tmp_new;
}

// Runtime: 0.035s
double cen2018features(cv::Mat fft_data, Eigen::MatrixXd &targets, float zq, int sigma_gauss, int min_range) {


    std::vector<float> sigma_q(fft_data.rows, 0);
    // Estimate the bias and subtract it from the signal
    cv::Mat q = fft_data.clone();
    for (int i = 0; i < fft_data.rows; ++i) {
        float mean = 0;
        for (int j = 0; j < fft_data.cols; ++j) {
            mean += fft_data.at<float>(i, j);
        }
        mean /= fft_data.cols;
        for (int j = 0; j < fft_data.cols; ++j) {
            q.at<float>(i, j) = fft_data.at<float>(i, j) - mean;
        }
    }

    // Create 1D Gaussian Filter (0.09)
    assert(sigma_gauss % 2 == 1);
    int fsize = sigma_gauss * 3;
    int mu = fsize / 2;
    float sig_sqr = sigma_gauss * sigma_gauss;
    cv::Mat filter = cv::Mat::zeros(1, fsize, CV_32F);
    float s = 0;
    for (int i = 0; i < fsize; ++i) {
        filter.at<float>(0, i) = exp(-0.5 * (i - mu) * (i - mu) / sig_sqr);
        s += filter.at<float>(0, i);
    }
    filter /= s;
    cv::Mat p;
    cv::filter2D(q, p, -1, filter, cv::Point(-1, -1), 0, cv::BORDER_REFLECT101);

    // Estimate variance of noise at each azimuth (0.004)
    for (int i = 0; i < fft_data.rows; ++i) {
        int nonzero = 0;
        for (int j = 0; j < fft_data.cols; ++j) {
            float n = q.at<float>(i, j);
            if (n < 0) {
                sigma_q[i] += 2 * (n * n);
                nonzero++;
            }
        }
        if (nonzero)
            sigma_q[i] = sqrt(sigma_q[i] / nonzero);
        else
            sigma_q[i] = 0.034;
    }

    // Extract peak centers from each azimuth
    std::vector<std::vector<cv::Point2f>> t(fft_data.rows);
#pragma omp parallel for
    for (int i = 0; i < fft_data.rows; ++i) {
        std::vector<int> peak_points;
        float thres = zq * sigma_q[i];
        for (int j = min_range; j < fft_data.cols; ++j) {
            float nqp = exp(-0.5 * pow((q.at<float>(i, j) - p.at<float>(i, j)) / sigma_q[i], 2));
            float npp = exp(-0.5 * pow(p.at<float>(i, j) / sigma_q[i], 2));
            float b = nqp - npp;
            float y = q.at<float>(i, j) * (1 - nqp) + p.at<float>(i, j) * b;
            if (y > thres) {
                peak_points.push_back(j);
            } else if (peak_points.size() > 0) {
                t[i].push_back(cv::Point(i, peak_points[peak_points.size() / 2]));
                peak_points.clear();
            }
        }
        if (peak_points.size() > 0)
            t[i].push_back(cv::Point(i, peak_points[peak_points.size() / 2]));
    }

    int size = 0;
    for (uint i = 0; i < t.size(); ++i) {
        size += t[i].size();
    }
    targets = Eigen::MatrixXd::Ones(3, size);
    int k = 0;
    for (uint i = 0; i < t.size(); ++i) {
        for (uint j = 0; j < t[i].size(); ++j) {
            targets(0, k) = t[i][j].x;
            targets(1, k) = t[i][j].y;
            k++;
        }
    }


    return 0.0;
}

struct Point {
    float i;
    int a;
    int r;
    Point(float i_, int a_, int r_) {i = i_; a = a_; r = r_;}
};

struct greater_than_pt {
    inline bool operator() (const Point& p1, const Point& p2) {
        return p1.i > p2.i;
    }
};

static void findRangeBoundaries(cv::Mat &s, int a, int r, int &rlow, int &rhigh) {
    rlow = r;
    rhigh = r;
    if (r > 0) {
        for (int i = r - 1; i >= 0; i--) {
            if (s.at<float>(a, i) < 0)
                rlow = i;
            else
                break;
        }
    }
    if (r < s.rows - 1) {
        for (int i = r + 1; i < s.cols; i++) {
            if (s.at<float>(a, i) < 0)
                rhigh = i;
            else
                break;
        }
    }
}

static bool checkAdjacentMarked(cv::Mat &R, int a, int start, int end) {
    int below = a - 1;
    int above = a + 1;
    if (below < 0)
        below = R.rows - 1;
    if (above >= R.rows)
        above = 0;
    for (int r = start; r <= end; r++) {
        if (R.at<float>(below, r) || R.at<float>(above, r))
            return true;
    }
    return false;
}

static void getMaxInRegion(cv::Mat &h, int a, int start, int end, int &max_r) {
    int max = -1000;
    for (int r = start; r <= end; r++) {
        if (h.at<float>(a, r) > max) {
            max = h.at<float>(a, r);
            max_r = r;
        }
    }
}

// Runtime: 0.050s
double cen2019features(cv::Mat fft_data, Eigen::MatrixXd &targets, int max_points, int min_range) {


    // Calculate gradient along each azimuth using the Prewitt operator
    cv::Mat prewitt = cv::Mat::zeros(1, 3, CV_32F);
    prewitt.at<float>(0, 0) = -1;
    prewitt.at<float>(0, 2) = 1;
    cv::Mat g;
    cv::filter2D(fft_data, g, -1, prewitt, cv::Point(-1, -1), 0, cv::BORDER_REFLECT101);
    g = cv::abs(g);
    double maxg = 1, ming = 1;
    cv::minMaxIdx(g, &ming, &maxg);
    g /= maxg;

    // Subtract the mean from the radar data and scale it by 1 - gradient magnitude
    float mean = cv::mean(fft_data)[0];
    cv::Mat s = fft_data - mean;
    cv::Mat h = s.mul(1 - g);
    float mean_h = cv::mean(h)[0];
    cout<<"1"<<endl;

    // Get indices in descending order of intensity
    std::vector<Point> vec;
    for (int i = 0; i < fft_data.rows; ++i) {
        for (int j = 0; j < fft_data.cols; ++j) {
            if (h.at<float>(i, j) > mean_h)
                vec.push_back(Point(h.at<float>(i, j), i, j));
        }
    }
    std::sort(vec.begin(), vec.end(), greater_than_pt());
    cout<<"2"<<endl;
    // Create a matrix, R, of "marked" regions consisting of continuous regions of an azimuth that may contain a target
    int false_count = fft_data.rows * fft_data.cols;
    uint j = 0;
    int l = 0;
    cv::Mat R = cv::Mat::zeros(fft_data.rows, fft_data.cols, CV_32F);
    while (l < max_points && j < vec.size() && false_count > 0) {
        if (!R.at<float>(vec[j].a, vec[j].r)) {
            int rlow = vec[j].r;
            int rhigh = vec[j].r;
            findRangeBoundaries(s, vec[j].a, vec[j].r, rlow, rhigh);
            bool already_marked = false;
            for (int i = rlow; i <= rhigh; i++) {
                if (R.at<float>(vec[j].a, i)) {
                    already_marked = true;
                    continue;
                }
                R.at<float>(vec[j].a, i) = 1;
                false_count--;
            }
            if (!already_marked)
                l++;
        }
        j++;
    }
cout<<"3"<<endl;
    std::vector<std::vector<cv::Point2f>> t(fft_data.rows);

#pragma omp parallel for
    for (int i = 0; i < fft_data.rows; i++) {
        // Find the continuous marked regions in each azimuth
        int start = 0;
        int end = 0;
        bool counting = false;
        for (int j = min_range; j < fft_data.cols; j++) {
            if (R.at<float>(i, j)) {
                if (!counting) {
                    start = j;
                    end = j;
                    counting = true;
                } else {
                    end = j;
                }
            } else if (counting) {
                // Check whether adjacent azimuths contain a marked pixel in this range region
                if (checkAdjacentMarked(R, i, start, end)) {
                    int max_r = start;
                    getMaxInRegion(h, i, start, end, max_r);
                    t[i].push_back(cv::Point(i, max_r));
                }
                counting = false;
            }
        }
    }
cout<<"4"<<endl;
    int size = 0;
    for (uint i = 0; i < t.size(); ++i) {
        size += t[i].size();
    }
    targets = Eigen::MatrixXd::Ones(3, size);
    int k = 0;
    for (uint i = 0; i < t.size(); ++i) {
        for (uint j = 0; j < t[i].size(); ++j) {
            targets(0, k) = t[i][j].x;
            targets(1, k) = t[i][j].y;
            k++;
        }
    }
    return 0.0;
}

const std::string Vec2String(const std::vector<std::string>& vec){

    if (vec.empty())
        return "";
    std::ostringstream oss;
    for(size_t i=0 ; i < vec.size()-1 ; i++)
        oss << vec[i] << ",";
    oss << vec.back();
    return oss.str();
}
void NormalizeIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double imin){

double imax = cloud->points.front().intensity;
for(auto&& p : cloud->points)
    imax = std::max(imax, (double)p.intensity);

//cout<<"max: "<<imax<<", min:"<<imin<<endl;
for(auto&& p : cloud    ->points)
    p.intensity = (p.intensity - imin)/(imax-imin);

}


}

