



#include <src/obsavoid/statistics.h>

#include <tuple>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <assert.h>


/*  Combine info across trials  */
 float StatFuns::mean(std::vector<float> results){
    float sum = std::accumulate(results.begin(), results.end(), 0.0);
    return sum / (float) results.size();
}
 float StatFuns::standard_dev(std::vector<float> results){
    float mean=StatFuns::mean(results);
    float var = 0.0;
    for( size_t n = 0; n < results.size(); n++ )
    {
        var += (results[n] - mean) * (results[n] - mean);
    }
    var /= (float) results.size();
    return sqrt(var);
}
 float StatFuns::min(std::vector<float> results)  
{
    return *std::min_element(results.begin(), results.end());
}
 float StatFuns::sum(std::vector<float> results)
{
    return std::accumulate(results.begin(), results.end(), 0.0);
}

 float StatFuns::get_minkowski_distance(argos::CVector3 x, argos::CVector3 y,int k) {
    /* get the minkowski-distance between two 3D vectors ; k is the parameter that determines e.g. manhattan vs Euclid vs 3D movements 
       on a flat surface, the Thymio can only move in two directions, so in that case use k=2
       in general, surface may not be flat, then k=3 makes sense

       alternative in Argos is Distance
       https://www.argos-sim.info/api/a00293_source.php#l00205
       l. 684; problem is it always uses k=2
    */
    float sum = 0.0f;
    sum += std::pow(std::abs(x.GetX() -y.GetX()), k);

    
    sum += std::pow(std::abs(x.GetY() -y.GetY()), k);


    sum += std::pow(std::abs(x.GetZ() -y.GetZ()), k);

    return std::pow(sum, 1.0 / (float) k);
}
 float StatFuns::uniform_prob(size_t n)
{
    float uniform_p = 1.0f/(float)n;
    return uniform_p;
}
 float StatFuns::max_variation_distance(size_t n)
{
    float uniform_prob=StatFuns::uniform_prob(n);
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
    for ( float p : probabilities ) 
    {
        dist += std::abs(p - uni_p);
    }
    dist*=0.50f;
    float m=StatFuns::max_variation_distance(probabilities.size());
    assert (dist<=m);
    return m - dist;
}
 float StatFuns::get_avg_dist(std::vector<argos::CVector3> positions , argos::CVector3 cm)
{
    float avg_dist=0.0f;
    for(argos::CVector3 pos:positions)
    {
        float dist=StatFuns::get_minkowski_distance(pos,cm);
        #ifdef PRINTING
            std::cout<<"position: "<<pos<<std::endl;
            std::cout<<"dist: "<<dist<<std::endl;
        #endif
        avg_dist+=dist;
    }
    
    avg_dist/=(float)positions.size();
    #ifdef PRINTING
        std::cout<<"avg_dist: "<<avg_dist<<std::endl;
    #endif
    return avg_dist;
}

//  float StatFuns::mutual_info(std::vector<float> p_xy,std::vector<float> p_x , std::vector<float> p_y)
// {
   
// }