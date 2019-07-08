
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
class EnvironmentGenerator;

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

    virtual void Destroy()
    {
        m_pcvecRobot.clear();
    }
    virtual ~BaseLoopFunctions(){

        
    };

    virtual std::string get_controller_id()=0;

    /* embodied entity  */
    virtual CEmbodiedEntity* get_embodied_entity(size_t robot);

    /* get RAB range */
    Real get_RAB_range(size_t robot);

    /* get the controller  */
    virtual BaseController* get_controller(size_t robot);

    /* place robots on initial positions */
    void place_robots();

    /* initialise the robot vector */
    virtual void init_robots(TConfigurationNode &t_node);
    
    virtual void Init(TConfigurationNode &t_node);
    void init_fitfuns(TConfigurationNode &t_node);

    virtual void Reset();

    virtual void PreStep() = 0;
    virtual void PostStep();

    virtual void before_trials(argos::CSimulator& cSimulator);
    /* set the current trial */
    inline void SetTrial()
    {
        ++m_unCurrentTrial;
    }

    /* add additional agents */
    virtual void create_new_agents();

    /* remove superfluous agents */
    virtual void remove_agents(size_t too_much);

    /* adjust the number of agents */
    void adjust_number_agents();

    void reset_agent_positions();
    virtual void start_trial(argos::CSimulator& cSimulator);
    virtual void end_trial();
    virtual void print_progress();
    float run_all_trials(argos::CSimulator& cSimulator);



    /* these methods are not be overridden */

    void perform_trial(argos::CSimulator &cSimulator);
    float alltrials_fitness();

    /* helper functions */
    

    /*get the orientation of the robot */
    CRadians get_orientation(size_t robot_index);

    /* linear speed normalised to [0,1], based on the actual movement rather than wheel speed */
    float actual_linear_velocity_01(size_t robot_index);

    /* turn velocity normalised to [0,1], based on the actual orientations rather than wheel speed*/
    float actual_turn_velocity_01(size_t robot_index);
    /* linear velocity normalised to [-1,1]*/
    float actual_linear_velocity_signed(size_t robot_index);

    /* get the mass */
    float get_mass(CThymioEntity *robot);

    /* get the centre of mass */
    argos::CVector3 centre_of_mass(const std::vector<CVector3>& positions);


public:

    CRandom::CRNG *m_pcRNG;
    float tick_time;
    std::vector<float> outf;
    std::vector<float> inputs;
    FitFun *fitfun;
    // EnvironmentGenerator* generator = NULL;
    std::vector<CVector3> curr_pos, old_pos;
    std::vector<CRadians> curr_theta, old_theta;
    size_t m_unNumberTrials;
    size_t  seed;// seed part corresponding to the one in the config
    int m_unCurrentTrial; // will start with -1 for convenience
    std::vector<std::vector<SInitSetup>> m_vecInitSetup,m_vecInitSetupCylinders;

    size_t m_unNumberCylinders;
    size_t m_unNumberRobots;
    EnvironmentGenerator* generator = NULL;
    std::string output_folder;
#ifdef RECORD_FIT
    std::ofstream fitness_writer;
#endif

    /* robot vectors */
    std::vector<CThymioEntity *> m_pcvecRobot;

    /* help to create robot vectors */
    std::string robot_id;
    size_t rab_data_size;
    size_t rab_range;
    size_t max_num_robots;
};
