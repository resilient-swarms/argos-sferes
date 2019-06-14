#ifndef ARENA_UTILS
#define ARENA_UTILS

//#include <boost/math/constants/constants.hpp>
/****************************************/

// static const float PI = boost::math::constants::pi<float>();

class BaseLoopFunctions;

typedef std::tuple<int, int, int> location_t;

class CoverageCalc
{
public:
    

    std::map<location_t, size_t> unique_visited_positions;
    //RunningStat* velocity_stats;
    argos::CVector3 center;
    float grid_step;
    float num_cells;
    CoverageCalc(){};
    CoverageCalc(BaseLoopFunctions* cLoopFunctions);

    virtual void define_grid(BaseLoopFunctions* cLoopFunctions);

    /* get the number of cells*/
    virtual void get_num_cells(BaseLoopFunctions &cLoopFunctions);
    
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
    const location_t null_location{-1,-1,-1}; 
    int max_bin_x, max_bin_y;
    BorderCoverageCalc(BaseLoopFunctions* cLoopFunctions);


    /* check whether bin is on the border*/
    bool is_on_border(location_t bin) const;

    /* get the number of cells*/
    virtual void get_num_cells(BaseLoopFunctions &cLoopFunctions);

    /* get bin corresponding to a position in the space */
    virtual location_t get_bin(argos::CVector3 vec) const;

    /* updated the visited positions */
    virtual void update(argos::CVector3 pos);
};


class DecayCoverageCalc
{
public:
    // create a grid of grid_size*grid_size
    // fill it up with zeros initially
    // upon visitation set to 1, then decay with decay rate
    const float decay_rate=0.005;
    const size_t grid_size_x=10;
    const size_t grid_size_y=10;
    float grid_step_x, grid_step_y;
    float accumulator;
    std::map<location_t,float> grid;
    DecayCoverageCalc(std::string init_type,BaseLoopFunctions* cLoopFunctions);

    /* get bin corresponding to a position in the space */
    location_t get_bin(argos::CVector3 vec) const;

    /* end the trial */
    void end_trial();

    /* updated the visited positions */
    void update(argos::CVector3 pos);

    /* decay */
    void decay();

    /* add the summed value across the grid to the acummulator */
    void get_grid_sum();
};

#endif