


// #ifndef FITNESS_FUNCTIONS
// #define FITNESS_FUNCTIONS

#include <vector>

class CObsAvoidEvolLoopFunctions;



class FitFun{
public:
    std::vector<float> fitness_per_trial;
    FitFun(){};


    /*after getting inputs, can update the descriptor if needed*/
    virtual void apply(CObsAvoidEvolLoopFunctions& cLoopFunctions,Real time)=0;

    /*after getting outputs, can update the descriptor if needed*/
    virtual float after_trials()=0;


};


class FloreanoMondada : public FitFun{
    //Floreano & Mondada '94; minimum of the linear speed, controlling for collision 
public:
    FloreanoMondada();

    /*after getting inputs, can update the descriptor if needed*/
    virtual void apply(CObsAvoidEvolLoopFunctions& cLoopFunctions,Real time);

    /*after getting outputs, can update the descriptor if needed*/
    virtual float after_trials();


};

class MeanSpeed : public FitFun{
    // same as FloreanoMondada but take mean instead of minimum
public:
    MeanSpeed();

    /*after getting inputs, can update the descriptor if needed*/
    virtual void apply(CObsAvoidEvolLoopFunctions& cLoopFunctions,Real time);

    /*after getting outputs, can update the descriptor if needed*/
    virtual float after_trials();


};


class Aggregation : public FitFun{
    //Floreano & Mondada '94; minimum of the linear speed
public:
    Aggregation();

    /*after getting inputs, can update the descriptor if needed*/
    virtual void apply(CObsAvoidEvolLoopFunctions& cLoopFunctions,Real time);
    /*after getting outputs, can update the descriptor if needed*/
    virtual float after_trials();
    float get_mass(CThymioEntity* robot);

    std::pair<std::vector<argos::CVector3>,argos::CVector3> centre_of_mass(CObsAvoidEvolLoopFunctions&  cLoopFunctions);


};







// #endif