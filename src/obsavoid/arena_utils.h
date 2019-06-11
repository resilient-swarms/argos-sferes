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
    float num_cells;
    CoverageCalc(){};
    CoverageCalc(CObsAvoidEvolLoopFunctions* cLoopFunctions);

    virtual void define_grid(CObsAvoidEvolLoopFunctions* cLoopFunctions);

    /* get the number of cells*/
    virtual void get_num_cells(CObsAvoidEvolLoopFunctions &cLoopFunctions);
    
    /* get the actual coverage of a single trial */
    float get_coverage() const;

    /* get bin corresponding to a position in the space */
    virtual location_t get_bin(argos::CVector3 vec) const;

    /* updated the visited positions */
    virtual void update(argos::CVector3 pos);
    
    /* after trial is finished, reset stats */
    void after_trial();

    /* get the visitation probabilities for each bin  */
    std::vector<float> get_probs(size_t num_updates) const; 

    /* print xy-location of an element in the visited positions */
    void print_xy(location_t) const;
};

class BorderCoverageCalc : public CoverageCalc
{
public:
    using CoverageCalc::location_t;
    const location_t null_location{-1,-1,-1}; 
    int max_bin_x, max_bin_y;
    BorderCoverageCalc(CObsAvoidEvolLoopFunctions* cLoopFunctions);


    /* check whether bin is on the border*/
    bool is_on_border(location_t bin) const;

    /* get the number of cells*/
    virtual void get_num_cells(CObsAvoidEvolLoopFunctions &cLoopFunctions);

    /* get bin corresponding to a position in the space */
    virtual location_t get_bin(argos::CVector3 vec) const;

    /* updated the visited positions */
    virtual void update(argos::CVector3 pos);
};

#endif