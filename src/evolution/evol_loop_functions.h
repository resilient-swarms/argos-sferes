

#ifndef EVOL_LOOP_FUNCTIONS
#define EVOL_LOOP_FUNCTIONS

//#define BEHAV_DIM 6

/****************************************/
/****************************************/
/* Sferes related headers */

#ifndef NO_PARALLEL
   #include <unistd.h>
   #include <signal.h>
   #include <sys/wait.h>
   #include <sys/mman.h>

#endif 
//#include <src/evolution/base_classes.h>

#include <src/evolution/nn_controller.h>
#include <src/core/base_loop_functions.h>
/****************************************/
/****************************************/





/****************************************/
/****************************************/

class Descriptor;

class FitFun;

class EvolutionLoopFunctions : public BaseLoopFunctions
{

public:
    EvolutionLoopFunctions();
    virtual ~EvolutionLoopFunctions();
    virtual std::string get_controller_id()
    {
        return "tnn";
    }
    /* get the controller  */
    virtual BaseController *get_controller(size_t robot);
    virtual void Init(TConfigurationNode &t_node);

    //    /* Configures the robot controller from the genome */
    //    void ConfigureFromGenome(robots_nn::nn_t& ctrl)
    //    {
    //        _ctrlrob = ctrl;
    //    }

    virtual void PreStep();
    virtual void PostStep();

    CVector3 get_position(CThymioEntity *robot)
    {

        CVector3 position = robot->GetEmbodiedEntity().GetOriginAnchor().Position;
        // #ifdef PRINTING
        //   std::cout<<"position "<<position<<std::endl;
        // #endif
        return position;
    }

private:

public:
#ifdef CVT
    std::string centroids_folder;
#endif

    std::vector<CThymioNNController *> m_pcvecController;
    bool stop_eval;
    Real stand_still, maxIRSensor;
    Descriptor *descriptor;

    // only used for the checks which are not used (presumably the checks quite expensive) ?; also not suitable for multi-agent ?
    CVector3 centre, max;

    /* config initialisation functions */

    /* Process behavioural descriptor type  */
    void init_descriptors(TConfigurationNode &t_node);
    /* Process initialisation of robots, number of trials, and outputfolder  */
    void init_simulation(TConfigurationNode &t_node);

    /* Process perturbations */
    void init_perturbations();

    /* remove superfluous agents */
    virtual void remove_agents(size_t too_much);

    /* add additional agents */
    virtual void create_new_agents();


    /* Process robots */
    virtual void init_robots(TConfigurationNode &t_node);

    /* next are functions that are done during loop */

    void before_trials(argos::CSimulator &cSimulator);
    void start_trial(CSimulator &cSimulator);
    void end_trial();

    /* following are all functions useful for setting the descriptor */

    std::vector<float> alltrials_descriptor();

    bool check_BD_choice(const std::string choice);

    Real get_Max_Sens(CThymioNNController &controller);
    

    /* get bin for sensory probabilities  */
    size_t get_sensory_bin(size_t i, size_t num_bins) const;
    /* get bin for sensory probabilities  */
    size_t get_actuator_bin(size_t i, size_t num_bins) const;

    /* get activation bin for the activations of each sensory quadrant */
    size_t get_quadrant_bin() const;

    /* get joint activation bin for the actuators */
    size_t get_joint_actuator_bin(size_t num_bins) const;

    /* get activation bin for the activations of each sensory quadrant */
    std::string quadrant_from_bin() const;

    /* get bin for the centre of mass of the swarm */
    size_t get_CM_bin(size_t num_bins, size_t num_SD_bins);


    /* get bin for the movement of he swarm */
    size_t get_swarmmovement_bin(size_t num_bins, size_t num_SD_bins);





    /* get positions of objects of a type indicated by a string */
    std::vector<CVector3> get_object_positions(std::string type);

    /* number of sensors */
    size_t get_num_sensors() const;


};

namespace sferes
{
// ********** Main Class ***********
//SFERES_FITNESS(FitObstacle, sferes::fit::Fitness)

FIT_MAP(FitObstacleMapElites){

    public :
        FitObstacleMapElites(){}

        inline bool dead()
        {
            return false;
        }

        // *************** _eval ************
        //
        // This is the main function to evaluate the individual
        // It runs argos sim
        //
        // **********************************
        template <typename Indiv>
        void eval(Indiv &ind)
        {
#ifdef NO_PARALLEL
            this->_objs.resize(1);

            ind.nn().simplify();
            //ind.nn().init();
            /* The CSimulator class of ARGoS is a singleton. Therefore, to
            * manipulate an ARGoS experiment, it is enough to get its instance.
            * This variable is declared 'static' so it is created
            * once and then reused at each call of this function.
            * This line would work also without 'static', but written this way
            * it is faster. 
            */
           
            // if (generator != NULL)
            // {   
            //     static argos::CSimulator &cSim = argos::CSimulator::GetInstance();
            //     static EvolutionLoopFunctions &cLoopFunctions = dynamic_cast<EvolutionLoopFunctions &>(cSimulator.GetLoopFunctions());

            //     //undo earlier Init and destroy controllers; start new simulation
            //     cSim.Destroy();
            //     static argos::CSimulator &cSim2 = argos::CSimulator::GetInstance();
            //     // do a new Init
            //     generator->generate(cSim2);// problem: could not remove the "tnn" controllers now they are duplicate

            // }
            static argos::CSimulator &cSimulator = argos::CSimulator::GetInstance();
            static EvolutionLoopFunctions &cLoopFunctions = dynamic_cast<EvolutionLoopFunctions &>(cSimulator.GetLoopFunctions());

            cLoopFunctions.generate();
            for (size_t j = 0; j < cLoopFunctions.m_unNumberRobots; ++j)
                cLoopFunctions.m_pcvecController[j]->nn = ind.nn_cpy();
        #ifdef PRINTING
            std::ofstream ofs("nn.dot");
            ind.nn().write(ofs);
        #endif
            float fFitness = cLoopFunctions.run_all_trials(cSimulator);

            this->_objs[0] = fFitness;
            this->_value = fFitness;
        #ifdef RECORD_FIT
            cLoopFunctions.fitness_writer << fFitness << std::endl;
        #endif
            std::vector<float> behavioural_descriptor = cLoopFunctions.alltrials_descriptor();
            this->set_desc(behavioural_descriptor);

        #ifdef PRINTING
            printf("\n\n fFitness = %f", fFitness);
        #endif


#else
	LOG.Flush();//flush beforehand (generation file text may have been written)
	LOGERR.Flush();
        /** Pointer to the shared memory area */
        Real* m_pfSharedMem;
        size_t unShareMemSize = (BEHAV_DIM + 1) * sizeof(Real);
        /* Get pointer to shared memory area */
        // Memory buffer will be readable and writable:
        int protection = PROT_READ | PROT_WRITE;
        // The buffer will be shared (meaning other processes can access it), but
        // anonymous (meaning third-party processes cannot obtain an address for it),
        // so only this process and its children will be able to use it:
        int visibility = MAP_ANONYMOUS | MAP_SHARED;
        m_pfSharedMem = reinterpret_cast<Real*>(::mmap(NULL, unShareMemSize, protection, visibility, -1, 0));

        if(m_pfSharedMem == MAP_FAILED)
        {
            ::perror("shared memory");
            exit(-1);
        }

        this->_objs.resize(1);

        ind.nn().simplify();

#ifndef NDEBUG
        for(size_t i = 0; i < BEHAV_DIM + 1; ++i)
            m_pfSharedMem[i] = -27.0;
#endif

        //std::cout << "In parent " << getpid() << " Fitness is initialized to " << m_pfSharedMem[0] << std::endl;
        pid_t pid = ::fork();

        if(pid == 0)
        { // child process
            //printf("pid in child=%d and parent=%d\n",getpid(),getppid());

            static argos::CSimulator &cSimulator = argos::CSimulator::GetInstance();
            static EvolutionLoopFunctions &cLoopFunctions = dynamic_cast<EvolutionLoopFunctions &>(cSimulator.GetLoopFunctions());

            //cLoopFunctions.generate();
            for (size_t j = 0; j < cLoopFunctions.m_unNumberRobots; ++j)
                cLoopFunctions.m_pcvecController[j]->nn = ind.nn_cpy();

            float fFitness = cLoopFunctions.run_all_trials(cSimulator);
            m_pfSharedMem[0] = fFitness;

            std::vector<float> behavioural_descriptor = cLoopFunctions.alltrials_descriptor();
            for(size_t i = 0; i < BEHAV_DIM; ++i)
                m_pfSharedMem[i+1] = behavioural_descriptor[i];

#ifdef PRINTING
            std::ofstream ofs("nn.dot");
            ind.nn().write(ofs);

            printf("\n\n fFitness = %f", fFitness);
#endif

#ifdef RECORD_FIT
            cLoopFunctions.fitness_writer << this->_objs[0] << std::endl;
#endif

            //std::cout << "Child " << getpid() << " has set fitness to " << m_pfSharedMem[0] << std::endl;
	    //::raise(SIGCHLD);
	     argos::CSimulator::GetInstance().Destroy();
   	    argos::LOG.Flush();
   	     argos::LOGERR.Flush();
            exit(2);
        }
        else if(pid > 0)
        { // parent process
            //printf("pid in parent=%d and childid=%d\n",getpid(),pid);
            //printf("Waiting parent process \n");
            //int nSlaveInfo;
            //pid_t tSlavePID = ::waitpid(pid,&nSlaveInfo, WUNTRACED);
            
           siginfo_t siginfo; 
          ::waitid(P_PID, pid, &siginfo, WEXITED);
	   
	   //:cout << "Resumed parent " << getpid() << " has got fitness value of " << m_pfSharedMem[0] << " from child " << pid << std::endl;

#ifndef NDEBUG
            for(size_t i = 0; i < BEHAV_DIM + 1; ++i)
                if(m_pfSharedMem[i] == -27)
                {
                    std::cout << "Check all values " << i << " " << m_pfSharedMem[i] << " -- should be a different from initialized value " << std::endl;
                    exit(-1);
                }
#endif

            this->_objs[0] = m_pfSharedMem[0];
            this->_value   = m_pfSharedMem[0];

            std::vector<float> behavioural_descriptor;
            for(size_t i = 0; i < BEHAV_DIM; ++i)
                behavioural_descriptor.push_back(m_pfSharedMem[i+1]);

            this->set_desc(behavioural_descriptor);
	     LOG.Flush();
            LOGERR.Flush();
	    //wait(NULL);
	    //sleep(1);
        }
        else
        {
            // fork failed
            std::cerr << "Fork failed";
            exit(-1);
        }
	
        munmap(m_pfSharedMem, (BEHAV_DIM + 1) * sizeof(Real));
#endif
    } // *** end of eval ***
};
}

/****************************************/
/****************************************/

#endif
