
/****************************************/
/****************************************/
/* ARGoS-related headers */
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/loop_functions.h>

#include <src/evolution/evol_loop_functions.h>
#include <src/evolution/descriptors.h>
#include <src/exec_tools.h>

/******************************************************/

struct EnvirParams : EAParams
{
    struct ea
    {
        SFERES_CONST double epsilon = EAParams::ea::epsilon;
        SFERES_CONST size_t behav_dim = 6;
        SFERES_ARRAY(size_t, behav_shape, 4, 4, 4, 4, 4, 4);
    };
    using EAParams::evo_float;
    using EAParams::parameters;
    using EAParams::pop;

    static std::vector<int> options;
};


std::vector<float> sferes::eval::L = {0.,0.,0.,0.,0.,0.};// minimum for each of the environment dimensions
std::vector<float> sferes::eval::U = {0.,0.,0.,0.,0.,0.};// maximum for each of the environment dimensions
std::vector<float> sferes::eval::initial_values = {0.,0.,0.,0.,0.,0.};// normal values for each of the environment dimensions
typedef T<EnvirParams, phen_t,eval::ArgosParallelEnvir<EnvirParams>>::ea_t parallelenvir_ea_t;
typedef T<EnvirTaskAgnostParams, phen_t,eval::ArgosParallelEnvir<EnvirTaskAgnostParams>>::ea_t parallelenvirtaskagnost_ea_t;

/****************************************/
/****************************************/


int main(int argc, char **argv)
{
    sferes::eval::jobname = argv[1];
    sferes::eval::redirect_dir = argv[2];
    std::cout << "configurations " << sferes::eval::jobname << std::endl;
    std::cout << "redirecting argos log to " << sferes::eval::redirect_dir << std::endl;
    /*
     * Initialize ARGoS
     */
    /* The CSimulator class of ARGoS is a singleton. Therefore, to
    * manipulate an ARGoS experiment, it is enough to get its instance */
    argos::CSimulator &cSimulator = argos::CSimulator::GetInstance();
    // note: at this point LOG seems to be NULL until you actually perform Init; the parent therefore should not be redirected
    /*  deal with argos simulator later */

    
    if (strcmp(argv[3],"task_agnost")==0)
    {
        std::cout << "starting task agnostic envir_evolution" << std::endl;
        init_shared_mem<EnvirTaskAgnostParams>();
        configure_and_run_ea<parallelenvirtaskagnost_ea_t>(argc, argv);
    }
    else
    {
        std::cout << "starting taskspecific envir_evolution: argv[3] was " << argv[3] << std::endl;
        init_shared_mem<EnvirParams>();
        configure_and_run_ea<parallelenvir_ea_t>(argc, argv);
    }

    /*
    * Dispose of ARGoS stuff
    */
    argos::CSimulator::GetInstance().Destroy();
    /* All is OK */
    return 0;
}

/****************************************/
/****************************************/
