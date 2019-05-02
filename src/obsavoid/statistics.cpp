

#include <src/obsavoid/statistics.h>

#include <tuple>
#include <cmath>
#include <numeric>
#include "ncd.h"
#include <assert.h>

/* logarithm with custom base */
float StatFuns::log(float number, size_t base)
{
    return base == EULER ? std::log(number) : std::log(number) / std::log(base);
}

/*  Combine info across trials  */
float StatFuns::mean(std::vector<float> results)
{
    float sum = std::accumulate(results.begin(), results.end(), 0.0);
    return sum / (float)results.size();
}
float StatFuns::quantile(std::vector<float> results, float cumul_dens, bool sorted)
{
    if (!sorted)
    {
        std::sort(results.begin(), results.end());
    }

    const auto quantile = results.size() % 2
                              ? results[results.size() / 2]
                              : ((double)results[results.size() / 2 - 1] + results[results.size() / 2]) * .5;
    return quantile;
}

float StatFuns::standard_dev(std::vector<float> results)
{
    float mean = StatFuns::mean(results);
    float var = 0.0;
    for (size_t n = 0; n < results.size(); n++)
    {
        var += (results[n] - mean) * (results[n] - mean);
    }
    var /= (float)results.size();
    return sqrt(var);
}
float StatFuns::min(std::vector<float> results)
{
    return *std::min_element(results.begin(), results.end());
}
float StatFuns::max(std::vector<float> results)
{
    return *std::max_element(results.begin(), results.end());
}
float StatFuns::range(std::vector<float> results)
{
    return StatFuns::max(results) - StatFuns::min(results);
}
float IQR(std::vector<float> results, bool sorted)
{
    if (!sorted)
    {
        std::sort(results.begin(), results.end());
    }
    float Q3 = StatFuns::quantile(results, 0.75, true);
    float Q1 = StatFuns::quantile(results, 0.25, true);
    return Q3 - Q1;
}
float StatFuns::sum(std::vector<float> results)
{
    return std::accumulate(results.begin(), results.end(), 0.0);
}

float StatFuns::get_minkowski_distance(argos::CVector3 x, argos::CVector3 y, size_t k)
{
    /* get the minkowski-distance between two 3D vectors ; k is the parameter that determines e.g. manhattan vs Euclid vs 3D movements 
       on a flat surface, the Thymio can only move in two directions, so in that case use k=2
       in general, surface may not be flat, then k=3 makes sense

       alternative in Argos is Distance
       https://www.argos-sim.info/api/a00293_source.php#l00205
       l. 684; problem is it always uses k=2
    */
    float sum = 0.0f;
    sum += std::pow(std::abs(x.GetX() - y.GetX()), k);

    sum += std::pow(std::abs(x.GetY() - y.GetY()), k);

    sum += std::pow(std::abs(x.GetZ() - y.GetZ()), k);

    return std::pow(sum, 1.0 / (float)k);
}
float StatFuns::get_minkowski_distance(std::vector<float> x, std::vector<float> y, size_t k)
{
    /* get the minkowski-distance between two 3D vectors ; k is the parameter that determines e.g. manhattan vs Euclid vs 3D movements 
       on a flat surface, the Thymio can only move in two directions, so in that case use k=2
       in general, surface may not be flat, then k=3 makes sense

       alternative in Argos is Distance
       https://www.argos-sim.info/api/a00293_source.php#l00205
       l. 684; problem is it always uses k=2
    */
    float sum = 0.0f;
    size_t s = x.size();
    assert(s == y.size());
    for (size_t i = 0; i < s; ++i)
    {
        sum += std::pow(std::abs(x[i] - y[i]), k);
    }

    return std::pow(sum, 1.0 / (float)k);
}
float StatFuns::uniform_prob(size_t n)
{
    float uniform_p = 1.0f / (float)n;
    return uniform_p;
}
float StatFuns::max_variation_distance(size_t n)
{
    float uniform_prob = StatFuns::uniform_prob(n);
    return 1.0 - uniform_prob;
}
float StatFuns::min_variation_distance()
{

    return 0.;
}
float StatFuns::uniformity(std::vector<float> probabilities)
{
    /* variation distance is a measure of distance between distributions (here to uniform distr); other options: KL divergence, Kolmogorov distance  
    *  it is defined as the largest possible probability difference for the same event
    */
    float uni_p = StatFuns::uniform_prob(probabilities.size());
    float dist = 0.;
    for (float p : probabilities)
    {
        dist += std::abs(p - uni_p);
    }
    dist *= 0.50f;
    float m = StatFuns::max_variation_distance(probabilities.size());
    assert(dist <= m);
    return m - dist;
}


float StatFuns::normalise(std::vector<float> &probabilities,float C)
{
    for(float &prob: probabilities)
    {
        prob/=C;
    }
}


float StatFuns::get_avg_dist(std::vector<argos::CVector3> positions, argos::CVector3 cm)
{
    float avg_dist = 0.0f;
    for (argos::CVector3 pos : positions)
    {
        float dist = StatFuns::get_minkowski_distance(pos, cm);
#ifdef PRINTING
        std::cout << "position: " << pos << std::endl;
        std::cout << "dist: " << dist << std::endl;
#endif
        avg_dist += dist;
    }

    avg_dist /= (float)positions.size();
#ifdef PRINTING
    std::cout << "avg_dist: " << avg_dist << std::endl;
#endif
    return avg_dist;
}
std::pair<float, float> StatFuns::entropy(std::vector<float> p, float time, size_t base)
{
    float entropy = 0.0f;
    float S = 0.0;
    for (int i = 0; i < p.size(); ++i)
    {
        if (p[i] > 0)
        {
            entropy -= p[i] * StatFuns::log(p[i], base);
            S += 1.0;
        }
    }
    entropy += (S - 1.) / (2. * time);
    std::pair<float, float> pair(entropy, S);
    return pair;
}
float StatFuns::joint_entropy(std::vector<float> joint_p, float S_x, float S_y, float time, size_t base)
{
    float entropy = 0.0f;
    float S_xy = 0.0f;
    for (int i = 0; i < joint_p.size(); ++i)
    {
        if (joint_p[i] > 0)
        {
            entropy -= joint_p[i] * StatFuns::log(joint_p[i], base);
            S_xy += 1.0;
        }
    }
    entropy += (S_x + S_y - S_xy - 1.) / (2 * time);
    return entropy;
}
float StatFuns::mutual_information(std::vector<float> joint_p, std::vector<float> p_x, std::vector<float> p_y, float time, size_t base)
{
    if (p_y.size()*p_y.size() != joint_p.size())
    {
        throw std::runtime_error("supply equally sized arrays");
    }

    std::pair<float, float> pair = StatFuns::entropy(p_x, time, base);
    float H_x = pair.first;
    float S_x = pair.second;
    std::pair<float, float> pair2 = StatFuns::entropy(p_x, time, base);
    float H_y = pair2.first;
    float S_y = pair2.second;
    float H_xy = StatFuns::joint_entropy(joint_p, S_x, S_y, time, base);
    return H_x + H_y - H_xy;
}

/* similar to mutual information but not symmetric; it measures the difference between distributions 
       Note: only use if p(x_i)=0 implies p(y_i)=0
       can be seen as the average number of bits required to obtain p from q
*/
float StatFuns::relative_entropy(std::vector<float> p_x, std::vector<float> p_y, float time, size_t base)
{
    float entropy = 0.0f;
    float S = 0.0;
    if (p_y.size() != p_x.size())
    {
        throw std::runtime_error("supply equally sized arrays");
    }
    for (int i = 0; i < p_x.size(); ++i)
    {
        if (p_x[i] > 0)
        {
            entropy += p_x[i] * StatFuns::log(p_x[i] / p_y[i], base); // need positive sign here
            //S+=1.0;
        }
    }
    //entropy += (S - 1.)/(2*time);  // do not use a correction here, don't know if that works the same way
    return entropy;
}

// float StatFuns::complexity()
// {

// }

float StatFuns::algorithmic_prob(double complexity, size_t symbols)
{
    return std::pow(symbols, -complexity);
}
/* normalised compression distance, similar to mutual information in AIT */
// float StatFuns::NCD(NcdDataObject object1,std::vector<NcdDataObject> objectsVector, bool averageValues)
// {
//     return GetNcdValue(object1,objectsVector,averageValues);
// }
//  float StatFuns::mutual_info(std::vector<float> p_xy,std::vector<float> p_x , std::vector<float> p_y)
// {

// }
