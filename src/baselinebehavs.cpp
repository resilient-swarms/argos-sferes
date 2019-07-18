
/****************************************/
/****************************************/
/* ARGoS-related headers */
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/loop_functions.h>


#include <src/baseline-behavs/baseline-behavs-loopfunc.h>

/****************************************/
/****************************************/


int main(int argc, char **argv)
{

    /*
     * Initialize ARGoS
     */
        /* The CSimulator class of ARGoS is a singleton. Therefore, to
    * manipulate an ARGoS experiment, it is enough to get its instance */
    argos::CSimulator &cSimulator = argos::CSimulator::GetInstance();
    /* Set the .argos configuration file
     * This is a relative path which assumed that you launch the executable
     * from argos3-examples (as said also in the README) */
    cSimulator.SetExperimentFileName(argv[1]);

    /* Load it to configure ARGoS */
    cSimulator.LoadExperiment();

    static CBaselineBehavsLoopFunctions &cLoopFunctions = dynamic_cast<CBaselineBehavsLoopFunctions &>(cSimulator.GetLoopFunctions());
    cLoopFunctions.run_all_trials(cSimulator);

#ifdef RECORD_FIT
    cLoopFunctions.fitness_writer.close();
#endif
    /*
    * Dispose of ARGoS stuff
    */
    argos::CSimulator::GetInstance().Destroy();

    /* All is OK */
    return 0;
}

/****************************************/
/****************************************/
