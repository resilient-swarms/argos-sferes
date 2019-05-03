#ifndef ARENA_UTILS
#define ARENA_UTILS
#include <src/obsavoid/evol_loop_functions.h>
//#include <boost/math/constants/constants.hpp>
/****************************************/

// static const float PI = boost::math::constants::pi<float>();

class CoverageCalc
{
public:
    std::map<std::tuple<int, int, int>, size_t> unique_visited_positions;
    //RunningStat* velocity_stats;
    argos::CVector3 center;
    float grid_step;
    float total_size;
    CoverageCalc(){};
    CoverageCalc(CObsAvoidEvolLoopFunctions* cLoopFunctions);

    /* get the number of squares completely occupied by cylindrical obstacles*/
    float get_obstacle_area(CObsAvoidEvolLoopFunctions *cLoopFunctions);
    
    /* get the actual coverage of a single trial */
    float get_coverage(size_t num_updates);

    /* get bin corresponding to a position in the space */
    std::tuple<int, int, int> get_bin(argos::CVector3 vec);

    /* updated the visited positions */
    void update(argos::CVector3 pos);
    
    /* after trial is finished, reset stats */
    void after_trial();

    /* get the visitation probabilities for each bin  */
    std::vector<float> get_probs(size_t num_updates); 
};



#endif