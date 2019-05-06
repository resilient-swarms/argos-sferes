

// #ifndef DESCRIPTORS
// #define DESCRIPTORS
/* ARGoS related headers */
/* The NN controller */
#include <tuple>
#include <cmath>

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/thymio/simulator/thymio_entity.h>

#include <src/obsavoid/arena_utils.h>
/****************************************/
/****************************************/
/****************************************/

class CObsAvoidEvolLoopFunctions;
class RunningStat;



class Descriptor
{
  public:
    Descriptor();
    size_t num_updates, current_trial;
    static const size_t behav_dim; // for now all the same

    /* final value of bd*/
    std::vector<std::vector<float>> bd;

    /* prepare for trials*/
    virtual void before_trials(CObsAvoidEvolLoopFunctions &cLoopFunctions);
    /*reset BD at the start of a trial*/
    virtual void start_trial();
    /*after getting inputs, can update the descriptor if needed*/
    virtual void set_input_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions){};

    /*after getting outputs, can update the descriptor if needed*/
    virtual void set_output_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions){};

    /*after the looping over robots*/
    virtual void after_robotloop(CObsAvoidEvolLoopFunctions &cLoopFunctions)
    {
    }

    /*end the trial*/
    virtual void end_trial(CObsAvoidEvolLoopFunctions &cLoopFunctions)
    {
    }

    /*summarise BD at the end of trials*/
    virtual std::vector<float> after_trials(CObsAvoidEvolLoopFunctions &cLoopFunctions);

};

class AverageDescriptor : public Descriptor
{
    /* Get the average sensory readings averaged within and between trial
     */
  public:
    AverageDescriptor()
    {
    }

    /*after getting inputs, can update the descriptor if needed*/
    virtual void set_input_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions);

    /*end the trial*/
    virtual void end_trial(CObsAvoidEvolLoopFunctions &cLoopFunctions);
};

class IntuitiveHistoryDescriptor : public Descriptor
{
    /* 
    *  track observation-action or state-action pairs over time 
    *  after all trials, gather statistics of the observed history
    */
  public:
    IntuitiveHistoryDescriptor(CLoopFunctions *cLoopFunctions);
    CoverageCalc coverageCalc;

    argos::CVector3 center;
    const float max_velocitysd = 0.50; // with min,max=0,1 -> at most 0.5 deviation on average
    float max_deviation, deviation;

    /*reset BD at the start of a trial*/
    virtual void start_trial();

    /*after getting inputs, can update the descriptor if needed*/
    virtual void set_input_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions);
    /*after getting outputs, can update the descriptor if needed*/
    virtual void set_output_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions);
    /*end the trial*/
    virtual void end_trial(CObsAvoidEvolLoopFunctions &cLoopFunctions);



};

struct Entity
{
    std::vector<float> attributes;
    CVector3 position;
    Entity() {}
    float &operator[](size_t idx) { return attributes[idx]; }
    static float distance(const Entity e1, const Entity e2);
    void set_attributes(const std::vector<float> new_vec, CVector3 pos)
    {
        attributes = new_vec;
        position = pos;
    }
};

struct Entity_Group
{

    size_t max_size, min_size;
    size_t kappa;                 // feature_size
    std::vector<Entity> entities; // the attributes of the entities, can be obtained by a key (e.g. location or agent id), makes identifying entities in a group easier

    Entity_Group()
    {
    }

    Entity_Group(size_t k, size_t M, size_t m, std::vector<Entity> entity_vec) : kappa(k), max_size(M), min_size(m), entities(entity_vec)
    {
    }
    size_t get_absolute_size()
    {
        return entities.size();
    }
    float get_size()
    {
        return ((float)get_absolute_size() - (float)min_size) / ((float)max_size - (float)min_size);
    }

    float mean_state_vec(size_t feature_index)
    {
        float mean_state = 0;
        for (auto &entity : entities)
        {

            mean_state += entity[feature_index];
        }
        mean_state /= (float)entities.size();
        return mean_state;
    }
    float sd_state_vec(size_t feature_index, float mean_state)
    {
        float sd = 0;
        for (auto &entity : entities)
        {

            sd += std::pow(entity[feature_index] - mean_state, 2);
        }
        sd /= (float)entities.size();
        return std::pow(sd, 0.5);
    }
    void add_entity(Entity e)
    {
        entities.push_back(e);
    }

    Entity &operator[](size_t idx) { return entities[idx]; }
};

class SDBC : public Descriptor
{
    /* 
    *  Systematically Derived Behavioral Characterisation
    */
  public:
    bool include_std;
    size_t bd_index, num_groups;
    float maxdist, maxX, maxY;
    std::map<std::string, Entity_Group> entity_groups;
    std::vector<std::string> within_comparison_groups, between_comparison_groups; //
    std::vector<std::string> variable_groups;                                     // contains the keys of groups with variable sizes
    SDBC(CLoopFunctions *cLoopFunctions, std::string init_type);
    void init_walls_and_robots(CLoopFunctions *cLoopFunctions);
    void init_robots(CLoopFunctions *cLoopFunctions);

    /* group sizes are the first BD dimensions*/
    void add_group_sizes();

    /* group mean attribute vectors, the second set of BD dimensions*/
    void add_group_meanstates();

    /* avg pair-wise distance within groups, the third set of  BD dimensions*/
    void add_within_group_dispersion();

    /* avg pair-wise distance between groups, the final BD dimensions*/
    void add_between_group_dispersion();

    /* prepare for trials*/
    virtual void before_trials(argos::CSimulator &cSimulator);
    /*reset BD at the start of a trial*/
    virtual void start_trial();

    /*after getting inputs, can update the descriptor if needed*/
    virtual void set_input_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions);

    /*after getting outputs, can update the descriptor if needed*/
    virtual void set_output_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions);
    /*after the looping over robots*/
    virtual void after_robotloop(CObsAvoidEvolLoopFunctions &cLoopFunctions);
    /*end the trial*/
    virtual void end_trial(CObsAvoidEvolLoopFunctions &cLoopFunctions);
};

// class RNNHistoryDescriptor: public HistoryDescriptor{
//     /*
//     *
//     *  use RNN to process observed oa-history
//     */
// public:
//     RNNHistoryDescriptor(){

//     }
//     void set_RNN_from_config()
//     {

//     }
//     static const size_t behav_dim=7;
//     /*reset BD at the start of a trial*/
//     virtual void start_trial()
//     {
//         //bd.resize(ParamsDnn::dnn::nb_inputs + ParamsDnn::dnn::nb_outputs - 1, 0.0f);
//         bd.resize(ParamsDnn::dnn::nb_inputs - 1, 0.0f);
//     }
//     /*end the trial*/
//     virtual void end_trial()
//     {
//         rnn->forward();
//     }
//     /*summarise BD at the end of trials*/
//     virtual std::vector<float> after_trials(Real time);
// };

class AutoDescriptor : public Descriptor
{
  public:
    AutoDescriptor() {}

    /*after getting inputs, can update the descriptor if needed*/
    virtual void set_input_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions){};

    /*after getting outputs, can update the descriptor if needed*/
    virtual void set_output_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions){};

    /*summarise BD at the end of trial*/
    virtual std::vector<float> after_trials(Real time, CObsAvoidEvolLoopFunctions &cLoopFunctions){};
};
/****************************************/
/****************************************/
/* typedef function pointer, useful for fitness functions */
//typedef void (*functionPtr)();

/* given a group of sensory activations, average their input activation, and get the correspinding bin
*/
static size_t get_group_inputactivation(size_t num_bins, std::vector<size_t> group, CObsAvoidEvolLoopFunctions &cLoopFunctions);

class CVT_MutualInfo : public Descriptor
{
  public:
    /* number of bins used for the probability distribution*/
    const size_t num_bins = 5;
    /* number of sensors */
    const size_t num_sensors=ParamsDnn::dnn::nb_inputs - 1;
    /* track the frequencies of the different bins for all groups*/
    std::vector<std::vector<float>> freqs;// for each sensor
    /* track the joint frequencies of the different bins for all groups*/
    std::vector<std::vector<std::vector<float>>> joint_freqs;//for each sensory combination a 2-D matrix
    CVT_MutualInfo() 
    {
    }
    /*  *
    float marginal_prob(size_t sensor_index)
    {

    }
    /* get bin for sensory probabilities  */
    size_t get_sensory_bin(float activation) const;
    /* prepare for trials*/
    virtual void before_trials(CObsAvoidEvolLoopFunctions &cLoopFunctions);
    /*reset BD at the start of a trial*/
    virtual void start_trial();
    /*after getting inputs, can update the descriptor if needed*/
    virtual void set_input_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions);

    /*after the looping over robots*/
    virtual void after_robotloop(CObsAvoidEvolLoopFunctions &cLoopFunctions);

    /*end the trial*/
    virtual void end_trial(CObsAvoidEvolLoopFunctions &cLoopFunctions);

    /*summarise BD at the end of trials*/
    virtual std::vector<float> after_trials(CObsAvoidEvolLoopFunctions &cLoopFunctions);
};

class CompressionDescriptor : public Descriptor
{
  public:
    CompressionDescriptor()
    {
    }
    /* given a group of sensory activations, average their activation*/
    float get_group_activation(std::vector<size_t> group, CObsAvoidEvolLoopFunctions &cLoopFunctions);
    /* prepare for trials*/
    virtual void before_trials(CObsAvoidEvolLoopFunctions &cLoopFunctions);
    /*reset BD at the start of a trial*/
    virtual void start_trial();
    /*after getting inputs, can update the descriptor if needed*/
    virtual void set_input_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions);

    /*after getting outputs, can update the descriptor if needed*/
    virtual void set_output_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions);

    /*after the looping over robots*/
    virtual void after_robotloop(CObsAvoidEvolLoopFunctions &cLoopFunctions);

    /*end the trial*/
    virtual void end_trial(CObsAvoidEvolLoopFunctions &cLoopFunctions);

    /*summarise BD at the end of trials*/
    virtual std::vector<float> after_trials(CObsAvoidEvolLoopFunctions &cLoopFunctions);
};

class NonMarkovianStochasticPolicyInduction : public Descriptor
{
    /* Obtain non-Markovian estimates of the stochastic policy.
    *  Rather than utilising the 
    */
  public:
    NonMarkovianStochasticPolicyInduction()
    {
        bd.resize(behav_dim);
    }

    /* prepare for trials*/
    virtual void before_trials(CObsAvoidEvolLoopFunctions &cLoopFunctions);
    /*reset BD at the start of a trial*/
    virtual void start_trial();
    /*after getting inputs, can update the descriptor if needed*/
    virtual void set_input_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions){};

    /*after getting outputs, can update the descriptor if needed*/
    virtual void set_output_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions){};

    /*after the looping over robots*/
    virtual void after_robotloop(CObsAvoidEvolLoopFunctions &cLoopFunctions)
    {
    }

    /*end the trial*/
    virtual void end_trial(CObsAvoidEvolLoopFunctions &cLoopFunctions)
    {
    }

    /*summarise BD at the end of trials*/
    virtual std::vector<float> after_trials(CObsAvoidEvolLoopFunctions &cLoopFunctions);
};

class CVT_Trajectory : public Descriptor
{
    /* 
    *  Divide the trajectory in N chunks, and record the (x,y) at each endpoint
    *  Total dimensionality of the descriptor = N*2*trials
    */
  public:
    size_t num_chunks;
    size_t periodicity;
    float maxX, maxY;
    /* final behavioural descriptor  */
    std::vector<float> final_bd;
    CVT_Trajectory(CObsAvoidEvolLoopFunctions &cLoopFunctions, size_t num_steps);

    inline bool end_chunk() const
    {
        return (num_updates + 1) % periodicity == 0;
    }

    virtual void before_trials(CObsAvoidEvolLoopFunctions &cLoopFunctions);

    /*after getting outputs, can update the descriptor if needed*/
    virtual void set_output_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions);
    /*summarise BD at the end of trials*/
    virtual std::vector<float> after_trials(CObsAvoidEvolLoopFunctions &cLoopFunctions);
};
// #endif
