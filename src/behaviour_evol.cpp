
/****************************************/
/****************************************/
/* ARGoS-related headers */
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/loop_functions.h>

#include <src/evolution/evol_loop_functions.h>
#include <src/exec_tools.h>

/****************************************/
/****************************************/

int main(int argc, char **argv)
{
#if ARGOS_PARALLEL
    /* note: needs to be in a cpp */
    /* Initialize ARGoS */
    /* Redirect LOG and argos::LOGERR to dedicated files to prevent clutter on the screen */
    std::ofstream cLOGFile(std::string("ARGoS_LOG_" + argos::ToString(getpid())).c_str(), std::ios::out);
    argos::LOG.DisableColoredOutput();
    argos::LOG.GetStream().rdbuf(cLOGFile.rdbuf());
    std::ofstream cLOGERRFile(std::string("ARGoS_LOGERR_" + argos::ToString(getpid())).c_str(), std::ios::out);
    argos::LOGERR.DisableColoredOutput();
    argos::LOGERR.GetStream().rdbuf(cLOGERRFile.rdbuf());
    argos::LOG << "starting "<< argv[1] << std::endl;// tell which job it is
    argos::LOGERR << "starting "<< argv[1] << std::endl;// tell which job it is
#endif

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

#ifdef CVT
    static EvolutionLoopFunctions &cLoopFunctions = dynamic_cast<EvolutionLoopFunctions &>(cSimulator.GetLoopFunctions());
    EAParams::ea::centroids = load_centroids(cLoopFunctions.centroids_folder + "/centroids_" + std::to_string(EAParams::ea::number_of_clusters) + "_" + std::to_string(EAParams::ea::number_of_dimensions) + ".dat");
#endif
#ifdef ARGOS_PARALLEL
    init_shared_mem<EAParams>();
    configure_and_run_ea<parallel_ea_t>(argc,argv);
#else
    configure_and_run_ea<serial_ea_t>(argc,argv);
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
