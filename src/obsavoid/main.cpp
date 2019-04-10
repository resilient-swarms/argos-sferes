
/****************************************/
/****************************************/
/* ARGoS-related headers */
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/loop_functions.h>


#include <src/obsavoid/evol_loop_functions.h>
#include <src/obsavoid/statistics.h>
#include <src/obsavoid/fitness_functions.h>
#include <src/obsavoid/descriptors.h>

/****************************************/
/****************************************/


int main(int argc, char** argv)
{
    srand(time(0));

    /*
     * Initialize ARGoS
     */
    /* The CSimulator class of ARGoS is a singleton. Therefore, to
     * manipulate an ARGoS experiment, it is enough to get its instance */
    argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();
    /* Set the .argos configuration file
     * This is a relative path which assumed that you launch the executable
     * from argos3-examples (as said also in the README) */
    cSimulator.SetExperimentFileName("experiments/obsavoid_evol.argos");
    /* Load it to configure ARGoS */
    cSimulator.LoadExperiment();

    

    static CObsAvoidEvolLoopFunctions& cLoopFunctions = dynamic_cast<CObsAvoidEvolLoopFunctions&>(cSimulator.GetLoopFunctions());


    //typedef FitObstacle<Params> fit_t;
    typedef FitObstacleMapElites<Params> fit_t;
    typedef phen::Dnn<robots_nn::gen_t, fit_t, ParamsDnn> phen_t;
    typedef eval::Eval<Params> eval_t; //eval::Parallel gives cryptic seg fault for nn. Unrelated but make sure visualization is disabled when parallelizing
    //typedef boost::fusion::vector<sferes::stat::ParetoFront<phen_t, Params> >  stat_t;

    typedef boost::fusion::vector<
            sferes::stat::Map<phen_t, Params>,
            sferes::stat::MapProgress<phen_t, Params>
            >  stat_t;

    //MODIFIER
    typedef modif::Dummy<> modifier_t;
    //typedef ea::Nsga2<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;
    typedef ea::MapElites<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;

    ea_t ea;

    run_ea(argc, argv, ea);


    /*
    * Dispose of ARGoS stuff
    */
    argos::CSimulator::GetInstance().Destroy();

    /* All is OK */
    return 0;
}

/****************************************/
/****************************************/
