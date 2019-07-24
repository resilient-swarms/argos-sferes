#include "caffe/caffe.hpp"
#include <tuple>
#include <string>
#include <boost/shared_ptr.hpp>

#ifdef USE_OPENCV
    #include <opencv2/core/core.hpp>
    #include <opencv2/highgui/highgui.hpp>
    #include <opencv2/imgproc/imgproc.hpp>
#endif  // USE_OPENCV




class CaffeNet {
 public:
  CaffeNet(const std::string& model_file,
             const std::string& trained_file="");

  //std::vector<Prediction> Classify(const cv::Mat& img, int N = 5);

 public:
  void SetMean(const std::string& mean_file);

  std::vector<float> Predict(const cv::Mat& img);

  void WrapInputLayer(std::vector<cv::Mat>* input_channels);

  void Preprocess(const cv::Mat& img,
                  std::vector<cv::Mat>* input_channels);

 private:
  boost::shared_ptr<caffe::Net<float> > net_;
  cv::Size input_geometry_;
  int num_channels_;
  cv::Mat mean_;
};


