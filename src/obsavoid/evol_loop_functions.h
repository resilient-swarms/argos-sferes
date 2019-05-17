

#ifndef EVOL_LOOP_FUNCTIONS
#define EVOL_LOOP_FUNCTIONS

//#define BEHAV_DIM 6



/****************************************/
/****************************************/
/* ARGoS related headers */
/* The NN controller */
#include <tuple>
#include <cmath>
#include <src/obsavoid/nn_controller.h>

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/thymio/simulator/thymio_entity.h>

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

//#include <src/obsavoid/base_classes.h>

/****************************************/
/****************************************/

using namespace sferes;
using namespace sferes::gen::evo_float;
using namespace sferes::gen::dnn;
using namespace nn;

struct ParamsDnn
{
    struct dnn
    {
        static constexpr size_t nb_inputs = 8;  //! 7 ir sensors + bias input at +1
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
        SFERES_CONST size_t number_of_clusters = 1000;
        SFERES_CONST size_t number_of_dimensions = BEHAV_DIM;
        typedef boost::array<double, number_of_dimensions> point_t;
        static std::vector<point_t> centroids;
    };

#else
    struct ea
    {

        SFERES_CONST double epsilon = 0; //0.05;
        SFERES_CONST size_t behav_dim = BEHAV_DIM;
#if BEHAV_DIM == 2
        SFERES_ARRAY(size_t, behav_shape, 10, 10);
#elif BEHAV_DIM == 3
        SFERES_ARRAY(size_t, behav_shape, 10, 10, 10);
#elif BEHAV_DIM == 6
        SFERES_ARRAY(size_t, behav_shape, 10, 10, 10, 10, 10, 10);
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
        SFERES_CONST size_t nb_gen = 5000;
        SFERES_CONST size_t dump_period = 1;
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

class CObsAvoidEvolLoopFunctions : public CLoopFunctions
{

public:
    CObsAvoidEvolLoopFunctions();
    virtual ~CObsAvoidEvolLoopFunctions();

    virtual void Init(TConfigurationNode &t_node);

    virtual void Reset();

    /* Called by the evolutionary algorithm to set the current trial */
    inline void SetTrial()
    {
        ++m_unCurrentTrial;
    }

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
    /* The initial setup of a trial */
    struct SInitSetup
    {
        CVector3 Position;
        CQuaternion Orientation;
    };

    //CEPuckEntity* m_pcEPuck;
    //CEPuckNNController* m_pcController;

    std::vector<CThymioNNController *> m_pcvecController;

public:
    std::vector<CThymioEntity *> m_pcvecRobot;

    CRandom::CRNG *m_pcRNG;

public:
    std::vector<std::vector<SInitSetup>> m_vecInitSetup;
    size_t m_unNumberTrials, m_unNumberRobots;
    int m_unCurrentTrial; // will start with -1 for convenience

public:
    std::string output_folder;
    //robots_nn::nn_t _ctrlrob;
    std::vector<robots_nn::nn_t> _vecctrlrob;
    std::vector<float> outf, inputs;


    bool stop_eval;
    Real stand_still, maxIRSensor;
    Descriptor *descriptor;
    FitFun *fitfun;

    // only used for the checks which are not used (presumably the checks quite expensive) ?; also not suitable for multi-agent ?
    CVector3 old_pos;
    CRadians old_theta;
    CVector3 curr_pos;
    CRadians curr_theta;
    CVector3 centre, max;

    void before_trials();
    void start_trial(CSimulator &cSimulator);
    void end_trial(Real time);
    float alltrials_fitness();
    std::vector<float> alltrials_descriptor();
    void print_progress();

    bool check_BD_choice(const std::string choice);

    Real get_Max_Sens(CThymioNNController &controller);

    /* get bin for sensory probabilities  */
    size_t get_sensory_bin(size_t i,size_t num_bins) const;
    /* get bin for sensory probabilities  */
    size_t get_actuator_bin(size_t i,size_t num_bins) const;

    /* get activation bin for the activations of each sensory quadrant */
    size_t get_quadrant_bin() const;

    /* get joint activation bin for the actuators */
    size_t get_joint_actuator_bin(size_t num_bins) const;

    /* get activation bin for the activations of each sensory quadrant */
    std::string quadrant_from_bin() const;

    /* get positions of objects of a type indicated by a string */
    std::vector<CVector3> get_object_positions(std::string type);

};

namespace sferes
{
// ********** Main Class ***********
//SFERES_FITNESS(FitObstacle, sferes::fit::Fitness)

FIT_MAP(FitObstacleMapElites){

    public :

        FitObstacleMapElites(){}

    CObsAvoidEvolLoopFunctions &
    getLoopFun(){
        /* The CSimulator class of ARGoS is a singleton. Therefore, to
      * manipulate an ARGoS experiment, it is enough to get its instance.
      * This variable is declared 'static' so it is created
      * once and then reused at each call of this function.
      * This line would work also without 'static', but written this way
      * it is faster. */
        static argos::CSimulator &cSimulator = argos::CSimulator::GetInstance();

/* Get a reference to the loop functions */
static CObsAvoidEvolLoopFunctions &cLoopFunctions = dynamic_cast<CObsAvoidEvolLoopFunctions &>(cSimulator.GetLoopFunctions());
return cLoopFunctions;
} // namespace sferes
inline bool dead()
{
    return false;
} // namespace sferes

template <typename Indiv>
void print_progress(Indiv &ind, CObsAvoidEvolLoopFunctions &cLoopFunctions, Real time)
{

    cLoopFunctions.print_progress();

    if (cLoopFunctions.m_unCurrentTrial == 0)
    {
        std::ofstream ofs("nn.dot");
        ind.nn().write(ofs);
    }
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
    static argos::CSimulator &cSimulator = argos::CSimulator::GetInstance();

    /* Get a reference to the loop functions */
    static CObsAvoidEvolLoopFunctions &cLoopFunctions = dynamic_cast<CObsAvoidEvolLoopFunctions &>(cSimulator.GetLoopFunctions());
    for (size_t j = 0; j < cLoopFunctions.m_unNumberRobots; ++j)
        cLoopFunctions._vecctrlrob[j] = ind.nn_cpy();

    cLoopFunctions.before_trials();

    /*
         * Run x trials and take the worst performance as final value.
        */

    for (size_t i = 0; i < cLoopFunctions.m_unNumberTrials; ++i)
    {
        cLoopFunctions.start_trial(cSimulator);

        /* Run the experiment */
        cSimulator.Execute();
        Real time = (Real)cSimulator.GetMaxSimulationClock();

        cLoopFunctions.end_trial(time);

#ifdef PRINTING

        print_progress(ind, cLoopFunctions, time);
#endif
    }
    /****************************************/
    /****************************************/
    float fFitness = cLoopFunctions.alltrials_fitness();
    this->_objs[0] = fFitness;
    this->_value = fFitness;

    Real time = (Real)cSimulator.GetMaxSimulationClock();
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
