
/****************************************/
/****************************************/
/* ARGoS-related headers */
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/loop_functions.h>


#include <src/evolution/evol_loop_functions.h>
#include <src/evolution/descriptors.h>
#include <src/exec_tools.h>

/******************************************************/



struct EnvirParams
{
    struct ea
    {
        SFERES_CONST double epsilon = Params::ea::epsilon;
        SFERES_CONST size_t behav_dim = 6;
        SFERES_ARRAY(size_t, behav_shape, 5, 4, 3, 5, 4, 4);
    };
    using Params::parameters;
    using Params::evo_float;
    using Params::pop;
};


/****************************************/
/****************************************/
template<>
void sferes::eval::_argos_parallel_envir<phen_t>::LaunchSlave(size_t slave_id)
{
    /* note: needs to be in a cpp */
    /* Initialize ARGoS */
    /* Redirect LOG and argos::LOGERR to dedicated files to prevent clutter on the screen */
    /* append to the back of the file */
    std::ofstream cLOGFile(std::string("ARGoS_LOG_" + argos::ToString(getppid())+ "_" + std::to_string(slave_id)).c_str(), std::ios::app);
    argos::LOG.DisableColoredOutput();
    argos::LOG.GetStream().rdbuf(cLOGFile.rdbuf());
    std::ofstream cLOGERRFile(std::string("ARGoS_LOGERR_" + argos::ToString(getppid())+ "_" + std::to_string(slave_id)).c_str(), std::ios::app);
    argos::LOGERR.DisableColoredOutput();
    argos::LOGERR.GetStream().rdbuf(cLOGERRFile.rdbuf());



    /* The CSimulator class of ARGoS is a singleton. Therefore, to
    * manipulate an ARGoS experiment, it is enough to get its instance */
    argos::CSimulator &cSimulator = argos::CSimulator::GetInstance();
    try
    {
            _generator.seed((unsigned) time(NULL) * getpid());// create a random generator based on process id
            
            size_t envir_no = _distribution(_generator);

            //redirect(jobname,getppid());
            // /* Set the .argos configuration file
            //  * This is a relative path which assumed that you launch the executable
            //  * from argos3-examples (as said also in the README) */
            cSimulator.SetExperimentFileName(jobname+"_"std::to_string(envir_no)+".argos");
            // /* Load it to configure ARGoS */
            cSimulator.LoadExperiment();
            
            /* describe the environment */
            std::vector<float> bd = {(float) envir_no / (float) NUM_ENVIRS};
            static EvolutionLoopFunctions &cLoopFunctions = dynamic_cast<EvolutionLoopFunctions &>(cSimulator.GetLoopFunctions());
            cLoopFunctions.descriptor = new StaticDescriptor(bd);
           
    }
    catch (argos::CARGoSException &ex)
    {
        argos::LOGERR << ex.what() << std::endl;
        argos::LOGERR.Flush();
        ::raise(SIGKILL);
    }
    //argos::LOG << "child starting " << std::endl;
     // initialise the fitness function and the genotype
    _pop[slave_id]->develop();
    assert(i < _pop.size());
    // evaluate the individual
    _pop[slave_id]->fit().eval(*_pop[slave_id]);

    assert(!std::isnan(_pop[slave_id]->fit().objs()[0])); // ASSUMES SINGLE OBJECTIVE
    // write fitness and descriptors to shared memory
    shared_memory[slave_id]->setFitness(_pop[slave_id]->fit().objs()[0]); // ASSUME SINGLE OBJECTIVE
    shared_memory[slave_id]->setDescriptor(_pop[slave_id]->fit().desc());
    shared_memory[slave_id]->setDeath(_pop[slave_id]->fit().dead());
    // argos::LOG << "child fitness " << slave_id << " " << _pop[slave_id]->fit().obj(0) << std::endl;
    // argos::LOG << "child: descriptor for individual " << slave_id << std::endl;
     
    // for (size_t j = 0; j < _pop[slave_id]->fit().desc().size(); ++j)
    // {
    //   argos::LOG << "   " << _pop[slave_id]->fit().desc()[j] << std::endl;
    // }
    // argos::LOG << "child: death " << _pop[slave_id]->fit().dead() << std::endl;
    argos::LOG.Flush();
    argos::LOGERR.Flush();
    cSimulator.Destroy();// difference to the usual argosparallel
    exit(EXIT_SUCCESS);
    
}



int main(int argc, char **argv)
{
    sferes::eval::jobname = argv[1];
    /*
     * Initialize ARGoS
     */
    /* The CSimulator class of ARGoS is a singleton. Therefore, to
    * manipulate an ARGoS experiment, it is enough to get its instance */
    argos::CSimulator &cSimulator = argos::CSimulator::GetInstance();
    // note: at this point LOG seems to be NULL until you actually perform Init; the parent therefore should not be redirected
    /*  deal with argos simulator later */


    init_shared_mem();
    configure_and_run_ea<parallelenvir_ea_t>(argc,argv);

    /*
    * Dispose of ARGoS stuff
    */
    argos::CSimulator::GetInstance().Destroy();
    /* All is OK */
    return 0;
}

/****************************************/
/****************************************/
