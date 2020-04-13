
/****************************************/
/****************************************/
/* ARGoS related headers */
/* The NN controller */
#include <tuple>
#include <cmath>
#include <src/core/base_controller.h>

#include <argos3/core/utility/math/angles.h>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/thymio/simulator/thymio_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <argos3/core/simulator/space/space.h>

#define WALL_THICKNESS 1.0 // note: configs should always take a 1 meter thickness of the walls

/*******************/
//class EnvironmentGenerator;

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

    virtual std::string get_controller_id() = 0;

    /* embodied entity  */
    virtual CEmbodiedEntity *get_embodied_entity(size_t robot);

    CEmbodiedEntity *get_embodied_cylinder(size_t robot);

    /* get RAB range */
    Real get_RAB_range(size_t robot);

    /* get the controller  */
    virtual BaseController *get_controller(size_t robot);

    /* place robots on initial positions */
    void place_robots();
    virtual bool try_robot_position(CVector3 &Position, CQuaternion &Orientation, const CRange<Real> x_range, const CRange<Real> y_range, const size_t m_unRobot, size_t &num_tries);
    virtual std::vector<size_t> priority_robotplacement();
    void robot_trial_setup(size_t m_unTrial, const CRange<Real> x_range, const CRange<Real> y_range,size_t& num_tries);

    /* place cylinders on initial positions */
    void place_cylinders();

    /* initialise data for custom environment generator */
    //void init_generator(TConfigurationNode &t_node);
    /* initialise the robot vector */
    virtual void init_robots(TConfigurationNode &t_node);

    virtual void Init(TConfigurationNode &t_node);
    void init_fitfuns(TConfigurationNode &t_node);

    /* before the trials start and Reset happens check 
    whether some settings of the config must be changed */
    void generate();

    virtual void Reset();

    virtual void PreStep() = 0;
    virtual void PostStep();

    virtual void before_trials(argos::CSimulator &cSimulator);
    /* set the current trial */
    inline void SetTrial()
    {
        ++m_unCurrentTrial;
    }
    /* get wall outer position, assuming centered evenly in arena */
    CVector3 get_wall_pos(std::string type)
    {
        CSpace::TMapPerTypePerId &entity_map = GetSpace().GetEntityMapPerTypePerId();
        CVector3 pos = any_cast<CBoxEntity *>(entity_map["box"][type])->GetEmbodiedEntity().GetOriginAnchor().Position;
        if (type == "wall_north")
        {
            // remove WALL_THICKNESS/2 from the Y component
            pos.SetY(pos.GetY() - WALL_THICKNESS / 2.0f);
        }
        else if (type == "wall_south")
        {
            // add WALL_THICKNESS/2 to the Y component
            pos.SetY(pos.GetY() + WALL_THICKNESS / 2.0f);
        }
        else if (type == "wall_east")
        {
            // remove WALL_THICKNESS/2 from the X component
            pos.SetX(pos.GetX() - WALL_THICKNESS / 2.0f);
        }
        else if (type == "wall_west")
        {
            // add WALL_THICKNESS/2 to the X component
            pos.SetX(pos.GetX() + WALL_THICKNESS / 2.0f);
        }
        else
        {
            throw std::runtime_error("walltype not supported");
        }
        return pos;
    }
    CVector3 get_arenasize()
    {

        CVector3 wall_N = get_wall_pos("wall_north");
        CVector3 wall_S = get_wall_pos("wall_south");
        CVector3 wall_W = get_wall_pos("wall_west");
        ;
        CVector3 wall_E = get_wall_pos("wall_east");

        Real y = wall_N.GetY() - wall_S.GetY();
        Real x = wall_E.GetX() - wall_W.GetX();
        CVector3 size = CVector3(x, y, (Real)0.0);
        return size;
    }

    inline CVector3 get_arenacenter()
    {
        CVector3 centre = GetSpace().GetArenaCenter();

        return CVector3(centre.GetX(), centre.GetY(), (Real)0.0);
    }

    /* add additional agents */
    virtual void create_new_agents();

    /* remove superfluous agents */
    virtual void remove_agents(size_t too_much);

    /* adjust the number of agents */
    void adjust_number_agents();

    /* add additional cylinders */
    virtual void create_new_cylinders();

    /* remove superfluous cylinders */
    virtual void remove_cylinders(size_t too_much);

    /* adjust the number of cylinders */
    void adjust_number_cylinders();

    /* put the agents on the initvecs */
    virtual void reset_agent_positions(bool force=false);

    /* put the cylinders on the initvecs */
    void reset_cylinder_positions();

    virtual void start_trial(argos::CSimulator &cSimulator);
    virtual void end_trial();
    virtual void print_progress();
    float run_all_trials(argos::CSimulator &cSimulator);

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
    argos::CVector3 centre_of_mass(const std::vector<CVector3> &positions);
#ifdef RECORD_FIT
    /* write fitness to file */
    virtual void write_fitness(float fFitness)
    {
        fitness_writer << fFitness << std::endl;
    }
#endif

public:
    bool stop_eval;
    CRandom::CRNG *m_pcRNG;
    float tick_time;
    std::vector<float> outf;
    std::vector<float> inputs;
    FitFun *fitfun;
    std::vector<CVector3> curr_pos, old_pos;
    std::vector<CRadians> curr_theta, old_theta;
    size_t m_unNumberTrials;
    size_t seed;          // seed part corresponding to the one in the config
    int m_unCurrentTrial; // will start with -1 for convenience
    std::vector<std::vector<SInitSetup>> m_vecInitSetup, m_vecInitSetupCylinders;

    size_t m_unNumberCylinders = 0; //note: this may also be zero when using distribute exclusively
    size_t m_unNumberRobots;
    //EnvironmentGenerator* generator = NULL;
    std::string output_folder;
#ifdef RECORD_FIT
    std::ofstream fitness_writer;
#endif

    /* robot vectors */
    std::vector<CThymioEntity *> m_pcvecRobot;
    /* cylinder vectors */
    std::vector<CCylinderEntity *> m_pcvecCylinder;

    /* help to create robot vectors */
    std::string robot_id;
    size_t rab_data_size;
    size_t rab_range;
    size_t max_num_robots;
};
