#ifndef ARENA_UTILS
#define ARENA_UTILS
#include <src/obsavoid/evol_loop_functions.h>
//#include <boost/math/constants/constants.hpp>
/****************************************/

// static const float PI = boost::math::constants::pi<float>();

class CoverageCalc
{
public:
    typedef std::tuple<int, int, int> location_t;

    std::map<location_t, size_t> unique_visited_positions;
    //RunningStat* velocity_stats;
    argos::CVector3 center;
    float grid_step;
    float total_size;
    float obstacle_cells;
    CoverageCalc(){};
    CoverageCalc(CObsAvoidEvolLoopFunctions* cLoopFunctions);

    /* get the number of squares completely occupied by cylindrical obstacles*/
    void get_obstacle_area(CSimulator &sim);
    
    /* get the actual coverage of a single trial */
    float get_coverage() const;

    /* get bin corresponding to a position in the space */
    location_t get_bin(argos::CVector3 vec) const;

    /* updated the visited positions */
    void update(argos::CVector3 pos);
    
    /* after trial is finished, reset stats */
    void after_trial();

    /* get the visitation probabilities for each bin  */
    std::vector<float> get_probs(size_t num_updates) const; 

    /* print xy-location of an element in the visited positions */
    void print_xy(location_t) const;
};



#endif