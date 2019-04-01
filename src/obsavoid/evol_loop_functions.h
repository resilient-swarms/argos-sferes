/****************************************/
/****************************************/
/* ARGoS related headers */
/* The NN controller */
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

#include <modules/map_elites/map_elites.hpp>
#include <modules/map_elites/fit_map.hpp>
#include <modules/map_elites/stat_map.hpp>
#include <modules/map_elites/stat_progress.hpp>

#include <sferes/fit/fitness.hpp>
#include <sferes/eval/parallel.hpp>
#include <sferes/eval/eval.hpp>
#include <sferes/stat/pareto_front.hpp>
#include <sferes/stat/best_fit.hpp>
#include <sferes/modif/dummy.hpp>

/****************************************/
/****************************************/

using namespace sferes;
using namespace sferes::gen::evo_float;
using namespace sferes::gen::dnn;
using namespace nn;

using namespace argos;

struct Params
{
    struct ea
    {
        SFERES_CONST size_t behav_dim = 7;
        SFERES_CONST double epsilon = 0;//0.05;
        SFERES_ARRAY(size_t, behav_shape, 10, 10, 10, 10, 10, 10, 10);
    };

    struct parameters
    {
        //Min and max weights of MLP?
        static constexpr float min = -5.0f;
        static constexpr float max = 5.0f;
    };

    struct evo_float
    {
        static constexpr mutation_t mutation_type = polynomial;
        //static const cross_over_t cross_over_type = sbx;
        static constexpr cross_over_t cross_over_type = no_cross_over;
        static constexpr float cross_rate = 0.0f;
        static constexpr float mutation_rate = 0.1f;
        static constexpr float eta_m = 15.0f;
        static constexpr float eta_c = 10.0f;
    };

    struct pop
    {
        // number of initial random points
        SFERES_CONST size_t init_size = 200;//1000;
        // size of a batch
        SFERES_CONST size_t size = 200; //1000;
        SFERES_CONST size_t nb_gen = 10001;
        SFERES_CONST size_t dump_period = 100;
    };
};

struct ParamsDnn
{
    struct dnn
    {
        static constexpr size_t nb_inputs       = 8; //! 7 ir sensors + bias input at +1
        static constexpr size_t nb_outputs      = 2; // 2 motors: left and right wheel

        static constexpr int io_param_evolving = true;
        static constexpr float m_rate_add_conn	= 0.05f;
        static constexpr float m_rate_del_conn	= 0.1f;
        static constexpr float m_rate_change_conn = 0.05f;
        static constexpr float m_rate_add_neuron  = 0.05f;
        static constexpr float m_rate_del_neuron  = 0.1f;

        static constexpr init_t init = random_topology; //random_topology or ff (feed-forward)
        //these only count w/ random init, instead of feed forward
        static constexpr size_t min_nb_neurons  = 4; // does not include input and output neurons
        static constexpr size_t max_nb_neurons  = 10; // does not include input and output neurons
        static constexpr size_t min_nb_conns    = nb_inputs*nb_outputs;
        static constexpr size_t max_nb_conns    = 40;
    };

    struct parameters
    {
        //Min and max weights of MLP?
        static constexpr float min = -5.0f;
        static constexpr float max = 5.0f;
    };

    struct evo_float
    {
        static constexpr mutation_t mutation_type = polynomial;
        //static const cross_over_t cross_over_type = sbx;
        static constexpr cross_over_t cross_over_type = no_cross_over;
        static constexpr float cross_rate = 0.0f;
        static constexpr float mutation_rate = 0.1f;
        static constexpr float eta_m = 15.0f;
        static constexpr float eta_c = 10.0f;
    };
};

namespace robots_nn
{
typedef phen::Parameters<gen::EvoFloat<1, ParamsDnn>, fit::FitDummy<>, ParamsDnn> weight_t;
typedef phen::Parameters<gen::EvoFloat<1, ParamsDnn>, fit::FitDummy<>, ParamsDnn> bias_t;

typedef PfWSum<weight_t> pf_t;
typedef AfTanh<bias_t> af_t;
typedef Neuron<pf_t, af_t>  neuron_t;
typedef Connection <weight_t> connection_t;
typedef sferes::gen::Dnn<neuron_t, connection_t, ParamsDnn> gen_t;
typedef typename gen_t::nn_t nn_t; // not sure if typename should be here?
}

/****************************************/
/****************************************/

class CObsAvoidEvolLoopFunctions : public CLoopFunctions
{

public:

    CObsAvoidEvolLoopFunctions();
    virtual ~CObsAvoidEvolLoopFunctions();

    virtual void Init(TConfigurationNode& t_node);

    virtual void Reset();

    /* Called by the evolutionary algorithm to set the current trial */
    inline void SetTrial(size_t un_trial)
    {
        m_unCurrentTrial = un_trial;
    }

//    /* Configures the robot controller from the genome */
//    void ConfigureFromGenome(robots_nn::nn_t& ctrl)
//    {
//        _ctrlrob = ctrl;
//    }

    virtual void  PreStep();
    virtual void  PostStep();

private:

    /* The initial setup of a trial */
    struct SInitSetup
    {
        CVector3 Position;
        CQuaternion Orientation;
    };

    //CEPuckEntity* m_pcEPuck;
    //CEPuckNNController* m_pcController;
    std::vector<CThymioEntity*>  m_pcvecRobot;
    std::vector<CThymioNNController*>  m_pcvecController;


public:
    CRandom::CRNG* m_pcRNG;

public:
    std::vector< std::vector<SInitSetup> > m_vecInitSetup;
    size_t m_unNumberTrials, m_unCurrentTrial, m_unNumberRobots;


public:
    //robots_nn::nn_t _ctrlrob;
    std::vector<robots_nn::nn_t> _vecctrlrob;
    std::vector<float> outf, inputs;

    Real nb_coll, stand_still;
    bool stop_eval;
    float speed, lin_speed;

    // Behavior descriptors
    float num_ds;
    std::vector<float> num_senact;

    CVector3 old_pos;  CRadians old_theta;
    CVector3 curr_pos; CRadians curr_theta;
};


/****************************************/
/****************************************/

namespace  sferes
{
// ********** Main Class ***********
//SFERES_FITNESS(FitObstacle, sferes::fit::Fitness)
FIT_MAP(FitObstacleMapElites)
{
    public:
    FitObstacleMapElites() { }

    // *************** _eval ************
    //
    // This is the main function to evaluate the individual
    // It runs argos sim
    //
    // **********************************

    bool dead()
    {
        return false;
    }


    template<typename Indiv>
    void eval(Indiv& ind)
    {
        this->_objs.resize(1);

        ind.nn().simplify();
        //ind.nn().init();


        /****************************************/
        /****************************************/
        /* The CSimulator class of ARGoS is a singleton. Therefore, to
      * manipulate an ARGoS experiment, it is enough to get its instance.
      * This variable is declared 'static' so it is created
      * once and then reused at each call of this function.
      * This line would work also without 'static', but written this way
      * it is faster. */
        static argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();

        /* Get a reference to the loop functions */
        static CObsAvoidEvolLoopFunctions& cLoopFunctions = dynamic_cast<CObsAvoidEvolLoopFunctions&>(cSimulator.GetLoopFunctions());
        for(size_t j = 0; j < cLoopFunctions.m_unNumberRobots; ++j)
            cLoopFunctions._vecctrlrob[j] = ind.nn_cpy();

        /*
         * Run x trials and take the worst performance as final value.
        */
        Real fFitness = INFINITY;
        for(size_t i = 0; i < cLoopFunctions.m_unNumberTrials; ++i)
        {
            cLoopFunctions.nb_coll=0;
            cLoopFunctions.stop_eval=false;
            cLoopFunctions.speed=0.0f; cLoopFunctions.lin_speed=0.0f;
            cLoopFunctions.stand_still = 0;
            cLoopFunctions.old_pos   = CVector3(0.0f, 0.0f, 0.0f);
            cLoopFunctions.old_theta = CRadians(0.0f);
            cLoopFunctions.num_ds = 0.0;
            cLoopFunctions.num_senact.resize(0); cLoopFunctions.num_senact.resize(ParamsDnn::dnn::nb_inputs - 1, 0.0f); // Params::dnn::nb_inputs includes sensors and a bias input

            /* Tell the loop functions to get ready for the i-th trial */
            cLoopFunctions.SetTrial(i);

            /* Reset the experiment. This internally calls also cLoopFunctions::Reset(). */
            cSimulator.Reset();

            /* Configure the controller with the indiv gen */
            //cLoopFunctions.ConfigureFromGenome(ind.nn());
            //cLoopFunctions._ctrlrob = ind.nn_cpy();
            //cLoopFunctions._ctrlrob.init(); // a copied nn object needs to be init before use


            for(size_t j = 0; j < cLoopFunctions.m_unNumberRobots; ++j)
                cLoopFunctions._vecctrlrob[j].init(); // a copied nn object needs to be init before use


            /* Run the experiment */
            cSimulator.Execute();

            if (this->mode() == fit::mode::view)
            {
                printf("\n\n lin_speed = %f", cLoopFunctions.lin_speed);
                printf("\n\n nb_coll = %f", cLoopFunctions.nb_coll);
                printf("\n\n nb_coll = %f", cLoopFunctions.nb_coll);
                printf("\n\n fitness in trial %lu is %f", i,
                       cLoopFunctions.lin_speed / (Real)cSimulator.GetMaxSimulationClock() * (Real)cLoopFunctions.nb_coll / (Real)cSimulator.GetMaxSimulationClock());

                if(i==0)
                {
                    std::ofstream ofs("nn.dot");
                    ind.nn().write(ofs);
                }
            }

            //Floreano & Mondada '94
            fFitness = Min(fFitness,(cLoopFunctions.lin_speed/(Real)cSimulator.GetMaxSimulationClock())*(Real)cLoopFunctions.nb_coll/(Real)cSimulator.GetMaxSimulationClock());
        }
        /****************************************/
        /****************************************/

        this->_objs[0] = fFitness;
        this->_value   = fFitness;

        std::vector<float> data;

        // BD1 -- characterizes the number of times the robot turns.
        //data.push_back(cLoopFunctions.num_ds / (Real)cSimulator.GetMaxSimulationClock());
        //assert(cLoopFunctions.num_ds / (Real)cSimulator.GetMaxSimulationClock() >= 0.0 && cLoopFunctions.num_ds / (Real)cSimulator.GetMaxSimulationClock() <= 1.0);

        // BD1 - BD7 -- characterizes the number of times the different IR proximity sensors on the robot return a high value
        for(size_t i = 0; i < cLoopFunctions.num_senact.size(); ++i)
            data.push_back(cLoopFunctions.num_senact[i] / (Real)cSimulator.GetMaxSimulationClock());

	
        this->set_desc(data);


        if (this->mode() == fit::mode::view)
            printf("\n\n fFitness = %f", fFitness);

    } // *** end of eval ***
};
}

/****************************************/
/****************************************/



