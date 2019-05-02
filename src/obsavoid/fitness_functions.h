


// #ifndef FITNESS_FUNCTIONS
// #define FITNESS_FUNCTIONS

#include <vector>

class CObsAvoidEvolLoopFunctions;



class FitFun{
public:
    std::vector<float> fitness_per_trial;
    FitFun(){};


    /*after completing trial, calc fitness*/
    virtual void apply(CObsAvoidEvolLoopFunctions& cLoopFunctions,Real time)=0;

    /*after completing all trials, combine fitness*/
    virtual float after_trials()=0;


};


class FloreanoMondada : public FitFun{
    //Floreano & Mondada '94; minimum of the linear speed, controlling for collision 
public:
    FloreanoMondada();

    /*after completing trial, calc fitness*/
    virtual void apply(CObsAvoidEvolLoopFunctions& cLoopFunctions,Real time);

    /*after completing all trials, combine fitness*/
    virtual float after_trials();


};

class MeanSpeed : public FitFun{
    // same as FloreanoMondada but take mean instead of minimum
public:
    MeanSpeed();

    /*after completing trial, calc fitness*/
    virtual void apply(CObsAvoidEvolLoopFunctions& cLoopFunctions,Real time);

    /*after completing all trials, combine fitness*/
    virtual float after_trials();


};


class Aggregation : public FitFun{
    //Floreano & Mondada '94; minimum of the linear speed
public:
    Aggregation();

    /*after completing trial, calc fitness*/
    virtual void apply(CObsAvoidEvolLoopFunctions& cLoopFunctions,Real time);
    /*after completing all trials, combine fitness*/
    virtual float after_trials();
    float get_mass(CThymioEntity* robot);

    std::pair<std::vector<argos::CVector3>,argos::CVector3> centre_of_mass(CObsAvoidEvolLoopFunctions&  cLoopFunctions);


};







// #endif