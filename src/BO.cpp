
/****************************************/
/****************************************/
/* ARGoS-related headers */
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/loop_functions.h>

#include <src/exec_tools.h>

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

    static MainLoopFunctions &cLoopFunctions = dynamic_cast<MainLoopFunctions &>(cSimulator.GetLoopFunctions());
    // expects two arguments
    std::string fitfile = argv[2];
    // override the usual fitness file
    cLoopFunctions.fitness_writer.close();
    cLoopFunctions.fitness_writer.open(fitfile,std::ios::out);
#ifndef RECORD_FIT
    throw std::runtime_error("need to set RECORD_FIT true if using BO");
#endif

#ifdef CVT
    EAParams::ea::centroids = load_centroids(cLoopFunctions.centroids_folder + "/centroids_" + std::to_string(EAParams::ea::number_of_clusters) + "_" + std::to_string(EAParams::ea::number_of_dimensions) + ".dat");
#endif
    
    configure_and_run_ea<serial_ea_t>(argc,argv);
    cLoopFunctions.fitness_writer.close();
    /*
    * Dispose of ARGoS stuff
    */
    argos::CSimulator::GetInstance().Destroy();



    /* All is OK */
    return 0;
}

/****************************************/
/****************************************/
