

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

#if NN_DIM_TYPE==1 
    #include "foraging_nn_controller.h"
#else
    #include "nn_controller.h"
#endif
#include <src/core/base_loop_functions.h>
#include <src/evolution/serialisation_functions.hpp>
/****************************************/
/****************************************/





/****************************************/
/****************************************/

class Descriptor;

class FitFun;

class BaseEvolutionLoopFunctions : public BaseLoopFunctions
{

public:
#if NN_DIM_TYPE==1   // use foragingthymio_nn_controller
    typedef ForagingThymioNN ControllerType;
#else
    typedef CThymioNNController ControllerType;
#endif

    size_t current_robot;
    BaseEvolutionLoopFunctions();
    virtual ~BaseEvolutionLoopFunctions();
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

public:
#ifdef CVT
    std::string centroids_folder;
#endif

    std::vector<ControllerType *> m_pcvecController;
    bool stop_eval=false;
    Real stand_still, maxIRSensor;
    Descriptor *descriptor;

    // only used for the checks which are not used (presumably the checks quite expensive) ?; also not suitable for multi-agent ?
    CVector3 centre, max;

    /* config initialisation functions */


    /* Process behavioural descriptor type  */
    void init_descriptors(TConfigurationNode &t_node);
    /* create static descriptor; no modifications needed */
    //void new_static_descriptor(std::vector<float> bd);
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

    Real get_Max_Sens(ControllerType &controller);
    

    /* get bin for sensory probabilities  */
    size_t get_sensory_bin(size_t i, size_t num_bins) const;
    /* get bin for sensory probabilities  */
    size_t get_actuator_bin(size_t i, size_t num_bins) const;

    /* get activation bin for the activations of each sensory quadrant, assuming use only proximity sensors */
    size_t get_quadrant_bin() const;

    /* get activation bin for the activations of each sensory quadrant, assuming use proximity+RAB sensors */
    size_t get_quadrant_binRAB() const;

    /* get activation of groups of inputs */
    std::vector<float> get_inputgroup_activations(std::vector<size_t> end_indexes, float thresh, size_t start=0) const;

    /* get activation of groups of inputs */
    std::vector<float> get_inputgroup_activations_smaller(std::vector<size_t> end_indexes, float thresh, size_t start=0) const;

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



    /* analyse an individual */
    void analyse(float fFitness);



};


/****************************************/
/****************************************/

#endif
