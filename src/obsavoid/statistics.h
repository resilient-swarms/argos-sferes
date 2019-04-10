
// #ifndef STATISTICS
// #define STATISTICS

#include <vector>
#include <argos3/core/utility/math/vector3.h>
#include <algorithm>


template <typename T>
void element_wise_addition(std::vector<T>& result, const std::vector<T>& other)
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
    RunningStat(): n(0),_M(0.),_S(0.)
    {

    }

    void push(float x)
    {
        ++n;
        if (n==1)
        {
            _M = x;
        }
        else
        {
            float oldM = _M;
            _M += (x - oldM)/(float)n;
            _S = _S + (x - oldM)*(x - _M);
        }

    }
    float mean()
    {

        return _M;
    }
    float var()
    {

        return n > 1 ? _S/(float)(n - 1) : _M*_M;
    }
    float std()
    {
        return sqrt(var());
    }

};

class StatFuns{
public:
    /*  Combine info across trials  */
    static float mean(std::vector<float> results);
    static float standard_dev(std::vector<float> results);
    static float min(std::vector<float> results) ;
    static float sum(std::vector<float> results);

    /* get the minkowski-distance between two 3D vectors ; k is the parameter that determines e.g. manhattan vs Euclid vs 3D movements 
           on a flat surface, the Thymio can only move in two directions, so in that case use k=2
           in general, surface may not be flat, then k=3 makes sense

           alternative in Argos is Distance
           https://www.argos-sim.info/api/a00293_source.php#l00205
           l. 684; problem is it always uses k=2
        */
    static float get_minkowski_distance(argos::CVector3 x, argos::CVector3 y,int k=3);
    static float uniform_prob(size_t n);
    static float max_variation_distance(size_t n);
    static float min_variation_distance();

    /* variation distance is a measure of distance between distributions (here to uniform distr); other options: KL divergence, Kolmogorov distance  
    *  it is defined as the largest possible probability difference for the same event
    */
    static float uniformity(std::vector<float> probabilities);
    static float get_avg_dist(std::vector<argos::CVector3> positions , argos::CVector3 cm);
};


// #endif