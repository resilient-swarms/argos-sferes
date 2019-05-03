

// #ifndef FITNESS_FUNCTIONS
// #define FITNESS_FUNCTIONS

#include <vector>
#include <src/obsavoid/arena_utils.h>
class CObsAvoidEvolLoopFunctions;

class FitFun
{
  public:
    std::vector<float> fitness_per_trial;
    FitFun(){};
    /*after a single step of a single agent */
    virtual void after_step(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions){};
    /*after completing trial, calc fitness*/
    virtual void apply(CObsAvoidEvolLoopFunctions &cLoopFunctions, Real time) = 0;

    /*after completing all trials, combine fitness*/
    virtual float after_trials() = 0;

    /*after completing a trial, print some statistics (if desired)*/
    virtual void print_progress(size_t trial){};
};

class FloreanoMondada : public FitFun
{
    //Floreano & Mondada '94; minimum of the linear speed, controlling for collision
  public:

    Real nb_coll = 0;
    float speed = 0.0f;
    float lin_speed = 0.0f;
    float num_ds = 0.0f;
    FloreanoMondada();
    /*after a single step of single agent */
    virtual void after_step(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions);
    /*after completing trial, calc fitness*/
    virtual void apply(CObsAvoidEvolLoopFunctions &cLoopFunctions, Real time);

    /*after completing all trials, combine fitness*/
    virtual float after_trials();

    /*after completing a trial, print some statistics (if desired)*/
    virtual void print_progress(size_t trial);
};

class MeanSpeed : public FitFun
{
    // same as FloreanoMondada but take mean instead of minimum
  public:

    Real nb_coll = 0;
    float speed = 0.0f;
    float lin_speed = 0.0f;
    float num_ds = 0.0f;
    MeanSpeed();
    /*after a single step of single agent */
    virtual void after_step(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions);
    /*after completing trial, calc fitness*/
    virtual void apply(CObsAvoidEvolLoopFunctions &cLoopFunctions, Real time);

    /*after completing all trials, combine fitness*/
    virtual float after_trials();

    /*after completing a trial, print some statistics (if desired)*/
    virtual void print_progress(size_t trial);


};

class Coverage : public FitFun
{
    // same as FloreanoMondada but take mean instead of minimum
  public:
    size_t num_updates=0;
    CoverageCalc coverageCalc;
    Coverage(CObsAvoidEvolLoopFunctions* cLoopFunctions);

    /*after a single step of single agent */
    virtual void after_step(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions);
    /*after completing trial, calc fitness*/
    virtual void apply(CObsAvoidEvolLoopFunctions &cLoopFunctions, Real time);

    /*after completing all trials, combine fitness*/
    virtual float after_trials();

    /*after completing a trial, print some statistics (if desired)*/
    virtual void print_progress(size_t trial);
};

class Aggregation : public FitFun
{
    //Floreano & Mondada '94; minimum of the linear speed
  public:
    Aggregation();

    /*after completing trial, calc fitness*/
    virtual void apply(CObsAvoidEvolLoopFunctions &cLoopFunctions, Real time);
    /*after completing all trials, combine fitness*/
    virtual float after_trials();
    float get_mass(CThymioEntity *robot);

    std::pair<std::vector<argos::CVector3>, argos::CVector3> centre_of_mass(CObsAvoidEvolLoopFunctions &cLoopFunctions);
};

// #endif