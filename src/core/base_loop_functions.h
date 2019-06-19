
/****************************************/
/****************************************/
/* ARGoS related headers */
/* The NN controller */
#include <tuple>
#include <cmath>
#include <src/evolution/nn_controller.h>

#include <argos3/core/utility/math/angles.h>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/thymio/simulator/thymio_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

/*******************/

class FitFun;

class BaseLoopFunctions : public CLoopFunctions
{

public:

    /* The initial setup of a trial */
    struct SInitSetup
    {
        CVector3 Position;
        CQuaternion Orientation;
    };

    BaseLoopFunctions();
    virtual ~BaseLoopFunctions(){};

    virtual std::string get_controller_id()=0;

    /* embodied entity  */
    virtual CEmbodiedEntity& get_embodied_entity(size_t robot);

    /* get the controller  */
    virtual BaseController* get_controller(size_t robot);

    /* place robots on initial positions */
    void place_robots();

    /* initialise the robot vector */
    virtual void init_robots();
    
    virtual void Init(TConfigurationNode &t_node);
    void init_fitfuns(TConfigurationNode &t_node);

    virtual void Reset();

    virtual void PreStep() = 0;
    virtual void PostStep() = 0;

    virtual void before_trials();
    /* set the current trial */
    inline void SetTrial()
    {
        ++m_unCurrentTrial;
    }
    virtual void start_trial(argos::CSimulator& cSimulator);
    virtual void end_trial(Real time);
    virtual void print_progress();
    float run_all_trials(argos::CSimulator& cSimulator);



    /* these methods are not be overridden */

    void perform_trial(argos::CSimulator &cSimulator);
    float alltrials_fitness();


    


    

public:

    CRandom::CRNG *m_pcRNG;
    float tick_time;
    std::vector<float> outf;
    std::vector<float> inputs;
    FitFun *fitfun;
    std::vector<CVector3> curr_pos, old_pos;
    std::vector<CRadians> curr_theta, old_theta;
    size_t m_unNumberTrials;
    int m_unCurrentTrial; // will start with -1 for convenience
    std::vector<std::vector<SInitSetup>> m_vecInitSetup;
    size_t m_unNumberRobots;
    std::string output_folder;
#ifdef RECORD_FIT
    std::ofstream fitness_writer;
#endif

    /* robot vectors */
    std::vector<CThymioEntity *> m_pcvecRobot;
};
