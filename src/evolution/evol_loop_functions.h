

#ifndef EVOL_LOOP_FUNCTIONS
#define EVOL_LOOP_FUNCTIONS

//#define BEHAV_DIM 6

/****************************************/
/****************************************/
/* Sferes related headers */

#include <sferes/phen/parameters.hpp>

#include <Eigen/Core>

#include <modules/nn2/mlp.hpp>
#include "phen_dnn.hpp"

#include <sferes/run.hpp>
#include <sferes/stc.hpp>
#include <sferes/misc.hpp>

#include <sferes/gen/evo_float.hpp>
//#include <sferes/ea/nsga2.hpp>
#ifdef CVT
#include <modules/cvt_map_elites/cvt_map_elites.hpp>
#include <modules/cvt_map_elites/fit_map.hpp>
#include <modules/cvt_map_elites/stat_map.hpp>
#include <modules/cvt_map_elites/stat_progress.hpp>

#else
#include <modules/map_elites/map_elites.hpp>
#include <modules/map_elites/fit_map.hpp>
#include <modules/map_elites/stat_map.hpp>
#include <modules/map_elites/stat_progress.hpp>

#endif
//#include <sferes/ea/nsga2.hpp>

#include <sferes/fit/fitness.hpp>
#include <sferes/eval/parallel.hpp>
#include <sferes/eval/eval.hpp>
#include <sferes/stat/pareto_front.hpp>
#include <sferes/stat/best_fit.hpp>
#include <sferes/modif/dummy.hpp>

//#include <src/evolution/base_classes.h>

#include <src/core/base_loop_functions.h>

/****************************************/
/****************************************/

using namespace sferes;
using namespace sferes::gen::evo_float;
using namespace sferes::gen::dnn;
using namespace nn;

struct ParamsDnn
{
    /* use only the proximity sensors */
    struct dnn
    {
        static constexpr size_t nb_inputs = 16; // ! 7 ir sensors + 8 RAB sensors bias input at +1
        static constexpr size_t nb_outputs = 2; // 2 motors: left and right wheel

        static constexpr int io_param_evolving = true;
        static constexpr float m_rate_add_conn = 0.15f;
        static constexpr float m_rate_del_conn = 0.15f;
        static constexpr float m_rate_change_conn = 0.15f;
        static constexpr float m_rate_add_neuron = 0.10f;
        static constexpr float m_rate_del_neuron = 0.10f;

        static constexpr init_t init = random_topology; //random_topology or ff (feed-forward)
        //these only count w/ random init, instead of feed forward
        static constexpr size_t min_nb_neurons = 0;  // does not include input and output neurons
        static constexpr size_t max_nb_neurons = 20; // does not include input and output neurons
        static constexpr size_t min_nb_conns = 0;
        static constexpr size_t max_nb_conns = 40;
    };

    struct parameters
    {
        //Min and max weights of MLP?
        static constexpr float min = -2.0f;
        static constexpr float max = 2.0f;
    };

    struct evo_float
    {
        static constexpr mutation_t mutation_type = polynomial;
        //static const cross_over_t cross_over_type = sbx;
        static constexpr cross_over_t cross_over_type = no_cross_over;
        static constexpr float cross_rate = 0.0f;
        static constexpr float mutation_rate = 0.05f;
        static constexpr float eta_m = 15.0f;
        static constexpr float eta_c = 10.0f;
    };
};
struct Params
{
#ifdef CVT
    struct ea
    {
        SFERES_CONST size_t number_of_clusters = 5000;
        SFERES_CONST size_t number_of_dimensions = BEHAV_DIM;
        typedef boost::array<double, number_of_dimensions> point_t;
        static std::vector<point_t> centroids;
    };

#else
    struct ea
    {

        SFERES_CONST double epsilon = 0; //0.05;
        SFERES_CONST size_t behav_dim = BEHAV_DIM;
#if BEHAV_DIM == 1
        SFERES_ARRAY(size_t, behav_shape, 1000);
#elif BEHAV_DIM == 2
        SFERES_ARRAY(size_t, behav_shape, 32, 32);
#elif BEHAV_DIM == 3
        SFERES_ARRAY(size_t, behav_shape, 10, 10, 10);
#elif BEHAV_DIM == 6
        SFERES_ARRAY(size_t, behav_shape, 4, 4, 4, 4, 4, 4);
#else
#error "Unsupported BEHAV_DIM setting (choose 2,3, or 6)"
#endif
    };
#endif

    struct parameters
    {
        //Min and max weights of MLP?
        static constexpr float min = -2.0f;
        static constexpr float max = 2.0f;
    };

    struct evo_float
    {
        static constexpr mutation_t mutation_type = polynomial;
        //static const cross_over_t cross_over_type = sbx;
        static constexpr cross_over_t cross_over_type = no_cross_over;
        static constexpr float cross_rate = 0.0f;
        static constexpr float mutation_rate = 0.05f;
        static constexpr float eta_m = 15.0f;
        static constexpr float eta_c = 10.0f;
    };

    struct pop
    {
        // number of initial random points
        SFERES_CONST size_t init_size = 2000; //1000;
        // size of a batch
        SFERES_CONST size_t size = 200; //1000;
        SFERES_CONST size_t nb_gen = 1000;
        SFERES_CONST size_t dump_period = 2;// computations are expensive so 2 generations is a lot
    };
};

namespace robots_nn
{
typedef phen::Parameters<gen::EvoFloat<1, ParamsDnn>, fit::FitDummy<>, ParamsDnn> weight_t;
typedef phen::Parameters<gen::EvoFloat<1, ParamsDnn>, fit::FitDummy<>, ParamsDnn> bias_t;

typedef PfWSum<weight_t> pf_t;
typedef AfTanh<bias_t> af_t;
typedef Neuron<pf_t, af_t> neuron_t;
typedef Connection<weight_t> connection_t;
typedef sferes::gen::Dnn<neuron_t, connection_t, ParamsDnn> gen_t;
typedef typename gen_t::nn_t nn_t; // not sure if typename should be here?
} // namespace robots_nn

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


    std::vector<robots_nn::nn_t> _vecctrlrob;
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

    /* Process robots */
    virtual void init_robots();

    /* next are functions that are done during loop */

    void before_trials(argos::CSimulator &cSimulator);
    void start_trial(CSimulator &cSimulator);
    void end_trial(Real time);

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

            for (size_t j = 0; j < cLoopFunctions.m_unNumberRobots; ++j)
                cLoopFunctions._vecctrlrob[j] = ind.nn_cpy();
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
