

#ifndef EVOL_LOOP_FUNCTIONS
#define EVOL_LOOP_FUNCTIONS

//#define BEHAV_DIM 6

/****************************************/
/****************************************/
/* Sferes related headers */


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

    } // *** end of eval ***
}
;
}

/****************************************/
/****************************************/

#endif
