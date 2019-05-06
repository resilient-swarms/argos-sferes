
// #ifndef STATISTICS
// #define STATISTICS



#include <vector>
#include <argos3/core/utility/math/vector3.h>
#include <algorithm>



#define EPS std::numeric_limits<float>::epsilon()

template <typename T>
void element_wise_addition(std::vector<T> &result, const std::vector<T> &other)
{
    assert(result.size() == other.size());

    std::transform(result.begin(), result.end(), other.begin(),
                   result.begin(), std::plus<T>());
}

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
    float std()
    {
        return sqrt(var());
    }
};
static const float EULER=std::exp(1);
class StatFuns
{
  public:
    static bool in_range(float num1,float a, float b);
    static bool float_equal(float num1,float num2);
    static bool float_smallerorequal(float num1,float num2);
    static float log(float number, size_t base = EULER);
    /*  Combine info across trials  */
    static float mean(std::vector<float> results);
    static float quantile(std::vector<float> results, float cumul_dens, bool sorted = false);
    static float standard_dev(std::vector<float> results);
    static float min(std::vector<float> results);
    static float max(std::vector<float> results);
    static float range(std::vector<float> results);
    static float IQR(std::vector<float> results, bool sorted = false);
    static float sum(std::vector<float> results);
    static float normalise(std::vector<float> &results,float C);

    /* get the minkowski-distance between two 3D vectors ; k is the parameter that determines e.g. manhattan vs Euclid vs 3D movements 
           on a flat surface, the Thymio can only move in two directions, so in that case use k=2
           in general, surface may not be flat, then k=3 makes sense

           alternative in Argos is Distance
           https://www.argos-sim.info/api/a00293_source.php#l00205
           l. 684; problem is it always uses k=2
    */
    static float get_minkowski_distance(argos::CVector3 x, argos::CVector3 y, size_t k = 2);
    static float get_minkowski_distance(std::vector<float> x, std::vector<float> y, size_t k = 2);
    static float uniform_prob(size_t n);
    static float max_variation_distance(size_t n);
    static float min_variation_distance();

    /* variation distance is a measure of distance between distributions (here to uniform distr); other options: KL divergence, Kolmogorov distance  
    *  it is defined as the largest possible probability difference for the same event
    */
    static float uniformity(std::vector<float> probabilities);
    static float get_avg_dist(std::vector<argos::CVector3> pos, argos::CVector3 cm);

    /* entropy based on log with custom base; however, does the correction imply always use natural log ? */
    static std::pair<float, float> entropy(std::vector<float> p, float time, size_t base = EULER);
    static float joint_entropy(std::vector<float> joint_p, float S_x, float S_y, float time, size_t base = EULER);
    static float mutual_information(std::vector<float> joint_p, std::vector<float> p_x, std::vector<float> p_y, float time, size_t base = EULER);

    /* similar to mutual information but not symmetric; it measures the difference between distributions 
       Note: only use if p(x_i)=0 implies p(y_i)=0
       can be seen as the average number of bits required to obtain p from q
    */
    static float relative_entropy(std::vector<float> p_x, std::vector<float> p_y, float time, size_t base);

    /* Calculate the maximal entropy, can also be used as maximal mutual info*/
    static float max_entropy(size_t num_bins, size_t base);

    float algorithmic_prob(double complexity, size_t symbols);
    //float StatFuns::complexity();// use zlib's compress2
    // /* normalised compression distance, similar to mutual information in AIT */  // use ncd.h
    // float StatFuns::NCD(NcdDataObject object1,std::vector<NcdDataObject> objectsVector, bool averageValues);
};

// #endif
