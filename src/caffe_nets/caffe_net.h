#include "caffe/caffe.hpp"
#include "caffe/sgd_solvers.hpp"
#include <tuple>
#include <string>
#include <boost/shared_ptr.hpp>
#include <src/core/statistics.h>

// #ifdef USE_OPENCV
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #endif // USE_OPENCV

// class CaffeNet {
//  public:
//   CaffeNet(const std::string& model_file,
//              const std::string& trained_file="");

//   //std::vector<Prediction> Classify(const cv::Mat& img, int N = 5);

//  public:
//   void SetMean(const std::string& mean_file);

//   void Train(const std::vector<float>& inputs);

//   std::vector<float> Predict(const std::vector<float>& input);

//   void WrapInputLayer(std::vector<cv::Mat>* input_channels);

//   void Preprocess(const cv::Mat& img,
//                   std::vector<cv::Mat>* input_channels);

//  private:
//   boost::shared_ptr<caffe::Net<float> > net_;
//   cv::Size input_geometry_;
//   int num_channels_;
//   cv::Mat mean_;
// };

template <typename SolverType>
class CaffeNet
{
public:
  CaffeNet()
  {
  }
  CaffeNet(const std::string &solverparam_file)
  {
    solver = new SolverType(solverparam_file);
  }

  //std::vector<Prediction> Classify(const cv::Mat& img, int N = 5);

public:
  caffe::Solver<float> *solver;
  bool include_bias=true;

  void input(const std::vector<std::vector<float>> &input_data,
             const std::vector<std::vector<float>> &target_data)
  {
    /*input data*/
    caffe::Blob<float> *input_layer =solver->net()->input_blobs()[0];
    float *input_ptr = input_layer->mutable_cpu_data(); // get pointer to data storage
    size_t num_data = 5;
    size_t input_shape = 4;
    for (int i = 0; i < num_data; ++i)
    {
      for (int j = 0; j < input_shape; j++)
      {
        input_ptr[i * input_shape + j] = input_data[i][j];
      }
    }
    /*target data*/
    caffe::Blob<float> *target_layer = solver->net()->input_blobs()[1];
    float *target_ptr = target_layer->mutable_cpu_data(); // get pointer to data storage
    size_t target_shape = 1;
    for (int i = 0; i < num_data; ++i)
    {
      for (int j = 0; j < target_shape; j++)
      {
        target_ptr[i * target_shape + j] = target_data[i][j];
      }
    }
  }

  void Solve(const std::vector<std::vector<float>> &input_data,
             const std::vector<std::vector<float>> &target_data)
  {
    input(input_data, target_data);
    solver->Solve();
  }

  /* get the trainable params from a batch as a single vector  */
  std::vector<float> get_trainable_params()
  {
    std::vector<float> params;
    std::vector<std::string> layerNames = solver->net()->layer_names();
    for(int i=0; i<layerNames.size(); i++){
        // std::cout << layerNames[i] << std::endl;
        std::vector<boost::shared_ptr<caffe::Blob<float> > >& layer = solver->net()->layer_by_name(layerNames[i])->blobs();
        // std::cout << layer.size() << std::endl;
        if (layer.size() == 0) continue;
        float * W = layer[0]->mutable_cpu_data();//weights
        std::vector<float> converted_Wdata {W, W + layer[0]->count()};
        append<float>(params,converted_Wdata);
        
        std::cout << "W size" << layer[0]->count() << std::endl;
        for(int j=0 ; j < layer[0]->count(); ++j)
        {
          //params.push_back(W[j]);
          std::cout<<W[j]<<",";
        }
        std::cout<<std::endl;
        if (include_bias)
        {
          float * B = layer[0]->mutable_cpu_data();//weights
          std::cout << "B size" << layer[1]->count() << std::endl;
          for(int j=0 ; j < layer[1]->count(); ++j)
          {
            params.push_back(B[j]);
            std::cout<<B[j]<<",";
          }
        }
        std::cout<<std::endl;

    }
    std::cout<<"number of weights "<<params.size()<<std::endl;
    std::cout<<print_vec<float>(params)<<std::endl;
  }


  /* get the activations from a batch as a single vector  */
  std::vector<float> get_activations()
  {
    std::vector<float> activations;
    std::vector<std::string> blobNames = solver->net()->blob_names();
    for(int i=0; i<blobNames.size(); i++){
        std::cout << blobNames[i] << std::endl;
        const boost::shared_ptr<caffe::Blob<float> >& blob = solver->net()->blob_by_name(blobNames[i]);
        std::cout<<"blobcount "<<blob->count()<<std::endl;
        float* blobdata = blob->mutable_cpu_data();
        std::vector<float> converted_blobdata {blobdata, blobdata + blob->count()};
        append<float>(activations,converted_blobdata);

        std::cout<<blob->count()<<std::endl;
        for (int j =0; j < blob->count(); ++j)
        {
          std::cout<<blobdata[j]<<",";
          //activations.push_back(blobdata[j]);
        }
        std::cout<<std::endl;
    }
    std::cout<<"number of activations "<<activations.size()<<std::endl;
    std::cout<<print_vec<float>(activations)<<std::endl;
  }
};