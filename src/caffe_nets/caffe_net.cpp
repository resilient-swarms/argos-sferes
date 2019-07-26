
#include <src/caffe_nets/caffe_net.h>
#include <algorithm>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace caffe;  // NOLINT(build/namespaces)
using std::string;


// CaffeNet::CaffeNet(const std::string& model_file,const std::string& trained_file) {
// #ifdef CPU_ONLY
//   Caffe::set_mode(Caffe::CPU);
// #else
//   Caffe::set_mode(Caffe::GPU);
// #endif

//   /* Load the network. */
//   test_net_.reset(new Net<float>(model_file, TEST));

//   if(trained_file != "")
//   {
//       net_->CopyTrainedLayersFrom(trained_file);
//   }

//   Blob<float>* input_layer = net_->input_blobs()[0];
//   num_channels_ = input_layer->channels();
//   CHECK(num_channels_ == 3 || num_channels_ == 1)
//     << "Input layer should have 1 or 3 channels.";
//   input_geometry_ = cv::Size(input_layer->width(), input_layer->height());
// }




// std::vector<float> CaffeNet::Predict(const cv::Mat& img) {
//   Blob<float>* input_layer = net_->input_blobs()[0];
//   input_layer->Reshape(1, num_channels_,
//                        input_geometry_.height, input_geometry_.width);
//   /* Forward dimension change to all layers. */
//   net_->Reshape();

//   std::vector<cv::Mat> input_channels;
//   WrapInputLayer(&input_channels);

//   Preprocess(img, &input_channels);

//   net_->Forward();

//   /* Copy the output layer to a std::vector */
//   Blob<float>* output_layer = net_->output_blobs()[0];
//   const float* begin = output_layer->cpu_data();
//   const float* end = begin + output_layer->channels();
//   return std::vector<float>(begin, end);
// }


// /* Wrap the input layer of the network in separate cv::Mat objects
//  * (one per channel). This way we save one memcpy operation and we
//  * don't need to rely on cudaMemcpy2D. The last preprocessing
//  * operation will write the separate channels directly to the input
//  * layer. */
// void CaffeNet::WrapInputLayer(std::vector<cv::Mat>* input_channels) {
//   Blob<float>* input_layer = net_->input_blobs()[0];

//   int width = input_layer->width();
//   int height = input_layer->height();
//   float* input_data = input_layer->mutable_cpu_data();
//   for (int i = 0; i < input_layer->channels(); ++i) {
//     cv::Mat channel(height, width, CV_32FC1, input_data);
//     input_channels->push_back(channel);
//     input_data += width * height;
//   }
// }

// void CaffeNet::Preprocess(const cv::Mat& img,
//                             std::vector<cv::Mat>* input_channels) {
//   /* Convert the input image to the input image format of the network. */
//   cv::Mat sample;
//   if (img.channels() == 3 && num_channels_ == 1)
//     cv::cvtColor(img, sample, cv::COLOR_BGR2GRAY);
//   else if (img.channels() == 4 && num_channels_ == 1)
//     cv::cvtColor(img, sample, cv::COLOR_BGRA2GRAY);
//   else if (img.channels() == 4 && num_channels_ == 3)
//     cv::cvtColor(img, sample, cv::COLOR_BGRA2BGR);
//   else if (img.channels() == 1 && num_channels_ == 3)
//     cv::cvtColor(img, sample, cv::COLOR_GRAY2BGR);
//   else
//     sample = img;

//   cv::Mat sample_resized;
//   if (sample.size() != input_geometry_)
//     cv::resize(sample, sample_resized, input_geometry_);
//   else
//     sample_resized = sample;

//   cv::Mat sample_float;
//   if (num_channels_ == 3)
//     sample_resized.convertTo(sample_float, CV_32FC3);
//   else
//     sample_resized.convertTo(sample_float, CV_32FC1);

//   cv::Mat sample_normalized;
//   cv::subtract(sample_float, mean_, sample_normalized);

//   /* This operation will write the separate BGR planes directly to the
//    * input layer of the network because it is wrapped by the cv::Mat
//    * objects in input_channels. */
//   cv::split(sample_normalized, *input_channels);

//   CHECK(reinterpret_cast<float*>(input_channels->at(0).data)
//         == net_->input_blobs()[0]->cpu_data())
//     << "Input channels are not wrapping the input layer of the network.";
// }


// void input(const std::vector<float><std::vector<float>& input_data)
// {
//   Blob<float>* input_layer = net_->input_blobs()[0];
//   float* input_data = input_layer->mutable_cpu_data();  // get pointer to data storage
//   for ( int i=0; i < 250; i++ ) {
//     for ( int j=0; j < 250; j++ ) {
//         input_data[i*250 + j] = data[i][j];  // I hope I did not filp anything here...
//     }
//   }

//   net_->forward();  // do forward pass
// }

// int main(int argc, char** argv) {

//   ::google::InitGoogleLogging(argv[0]);

//   string model_file   = argv[1];
//   string trained_file = argv[2];

//   CaffeNet classifier(model_file, trained_file);

//   string file = argv[3];

//   std::cout << "---------- Prediction for "
//             << file << " ----------" << std::endl;

//   cv::Mat img = cv::imread(file, -1);
//   CHECK(!img.empty()) << "Unable to decode image " << file;
//   std::vector<float> predictions = classifier.Predict(img);

//   /* Print the top N predictions. */
//   for (size_t i = 0; i < predictions.size(); ++i) {
//     std::cout << std::fixed << std::setprecision(4) << predictions[i] << " - \"" << std::endl;
//   }
// }






int main(int argc, char** argv)
{
  CaffeNet<caffe::NesterovSolver<float>> net = CaffeNet<caffe::NesterovSolver<float>>("/home/david/argos-sferes/src/caffe_nets/MLP_solver.prototxt");
  std::vector<std::vector<float>> input_data = 
  {
    {0.2,0.5,0.5,0.5},
    {0.4,0.5,0.5,0.5},
    {0.6,0.5,0.5,0.5},
    {0.8,0.5,0.5,0.5},
    {1.0,0.5,0.5,0.5}
  };
  std::vector<std::vector<float>> target_data = 
  {
    {0.2},
    {0.4},
    {0.6},
    {0.8},
    {1.0}
  };
  net.Solve(input_data,target_data);

  net.get_trainable_params();
  return 0;

}