
#ifndef STATISTICS
#define STATISTICS



#include <vector>
#include <argos3/core/utility/math/vector3.h>
#include <algorithm>
#include <cmath>
#include <boost/math/constants/constants.hpp>
#include <iostream>

#define BOOST_PI boost::math::constants::pi<float>()

#define EPS std::numeric_limits<float>::epsilon()
#define M_2PI 2.0*BOOST_PI

template<class T>
std::string print_vec(const std::vector<T>& v){
    typename std::vector<T>::iterator it;
    std::string s = "{ ";
    for(auto it = v.begin(); it != v.end(); ++it){
                 s= s + std::to_string(*it) + ",";
    }
    s = s + " } ";
    return s;
}

template <typename T>
void append(std::vector<T>& a, const std::vector<T>& b)
{
    a.reserve(a.size() + b.size());
    a.insert(a.end(), b.begin(), b.end());
}

template <typename T>
void element_wise_addition(std::vector<T> &result, const std::vector<T> &other)
{
    assert(result.size() == other.size());

    std::transform(result.begin(), result.end(), other.begin(),
                   result.begin(), std::plus<T>());
}
template <typename T>
std::vector<T> element_wise_additiondiv(std::vector<T> result, const std::vector<T> &other,const T divisor)
{
    assert(result.size() == other.size());
    for (int i=0; i < result.size(); ++i)
    {
        result[i]= (result[i] + other[i])/divisor;
    }
    return result;//note this is a copy !
}

template<typename Array_t>
float sum_array(const Array_t& arr)
{
    float sum = 0.0f;
    for(auto i:arr) 
    {
        sum+=i;
    }
    return sum;
}

template<typename Array_t>
void check_sums(float number, const Array_t& arr, size_t batch)
{
    
    int j=0;
    while (j < arr.size())
    {
        float sum = 0.0f;
        for(auto it=  arr.begin()+j; it != arr.begin() + j + batch; ++it ) 
        {
            sum+=*it;
        }
        if (std::abs(sum - number) > 0.05)
        {
            std::cout<<"sum: "<<sum<<std::endl;
        }
        j+=batch;
    }
}

template <int N>
struct Factorial 
{
    enum { value = N * Factorial<N - 1>::value };
};

class RunningStat
{
  public:
    size_t n;
    float _M, _S;
    RunningStat() : n(0), _M(0.), _S(0.)
    {
    }

    void push(float x)
    {
        ++n;
        if (n == 1)
        {
            _M = x;
        }
        else
        {
            float oldM = _M;
            _M += (x - oldM) / (float)n;
            _S = _S + (x - oldM) * (x - _M);
        }
    }
    float mean()
    {

        return _M;
    }
    float var()
    {

        return n > 1 ? _S / (float)(n - 1) : _M * _M;
    }
    float standard_error()
    {

        return var()/(float) n;
    }
    float std()
    {
        return sqrt(var());
    }
};
static const float EULER=std::exp(1);
class StatFuns
{
  public:
    template<typename T>
    static std::vector<std::vector<T> > transpose(const std::vector<std::vector<T> > data) {
    // this assumes that all inner vectors have the same size and
    // allocates space for the complete result in advance
    std::vector<std::vector<T> > result(data[0].size(),
                                          std::vector<T>(data.size()));
    for (size_t i = 0; i < data[0].size(); i++) 
        for (size_t j = 0; j < data.size(); j++) {
            result[i][j] = data[j][i];
        }
    return result;
    }
    static size_t get_bin(float activation, float min_act, float max_act, size_t num_bins);
    static bool in_range(float num1,float a, float b);
    static float clip(float num1,float a, float b);
    static bool float_equal(float num1,float num2);
    static bool float_smallerorequal(float num1,float num2);
    static float log(float number, float base = EULER);
    /*  Combine info across trials  */
    static std::vector<float> geometric_median(const std::vector<std::vector<float>>& results,size_t iterations=200);
    static float mean(const std::vector<float>& results);
    static float quantile(std::vector<float> results, float cumul_dens, bool sorted = false);
    static float standard_dev(const std::vector<float>& results);
    static argos::CVector2 XY_standard_dev(const std::vector<argos::CVector3>& results);
    static float min(const std::vector<float>& results);
    static float max(const std::vector<float>& results);
    static float range(const std::vector<float>& results);
    static float IQR(std::vector<float> results, bool sorted = false);
    static float sum(const std::vector<float>& results);
    static std::vector<float> divide(std::vector<float> results,float C);
    /* normalisation of probabilities */
    static void normalise(std::vector<float> &results,float C);
    /* normalisation to [0,1] (clip if exceeded); from a custom min and max*/
    static void normalise_01(std::vector<float> &results,const float min, const float max);
    static float laplace_smoothing(float count, float C, float alpha, size_t num_options);

    /* get the minkowski-distance between two 3D vectors ; k is the parameter that determines e.g. manhattan vs Euclid vs 3D movements 
           on a flat surface, the Thymio can only move in two directions, so in that case use k=2
           in general, surface may not be flat, then k=3 makes sense

           alternative in Argos is Distance
           https://www.argos-sim.info/api/a00293_source.php#l00205
           l. 684; problem is it always uses k=2
    */
    static float get_minkowski_distance(const argos::CVector3& x, const argos::CVector3& y, size_t k = 2);
    static float get_minkowski_distance(const std::vector<float>& x, const std::vector<float>& y, size_t k = 2);
    static float uniform_prob(size_t n);
    static float max_variation_distance(size_t n);
    static float min_variation_distance();

    /* variation distance is a measure of distance between distributions (here to uniform distr); other options: KL divergence, Kolmogorov distance  
    *  it is defined as the largest possible probability difference for the same event
    */
    static float uniformity(std::vector<float> probabilities);
    static float get_avg_dist(const std::vector<argos::CVector3>& pos, const argos::CVector3& cm);

    /* entropy based on log with custom base; however, does the correction imply always use natural log ? */
    static std::pair<float, float> entropy(std::vector<float> p, float time, float base = EULER);
    static float joint_entropy(std::vector<float> joint_p, float S_x, float S_y, float time, float base = EULER);
    static float mutual_information(std::vector<float> joint_p, std::vector<float> p_x, std::vector<float> p_y, float time, float base = EULER);

    /* similar to mutual information but not symmetric; it measures the difference between distributions 
       Note: only use if p(x_i)=0 implies p(y_i)=0
       can be seen as the average number of bits required to obtain p from q
    */
    static float relative_entropy(std::vector<float> p_x, std::vector<float> p_y, float time, float base);

    /* Calculate the maximal entropy, can also be used as maximal mutual info*/
    static float max_entropy(size_t num_bins,float base);

    float algorithmic_prob(double complexity, size_t symbols);
    //float StatFuns::complexity();// use zlib's compress2
    // /* normalised compression distance, similar to mutual information in AIT */  // use ncd.h
    // float StatFuns::NCD(NcdDataObject object1,std::vector<NcdDataObject> objectsVector, bool averageValues);
};

#endif
