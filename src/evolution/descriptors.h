

// #ifndef DESCRIPTORS
// #define DESCRIPTORS
/* ARGoS related headers */
/* The NN controller */
#include <tuple>
#include <cmath>

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/thymio/simulator/thymio_entity.h>

#include <src/core/arena_utils.h>
/****************************************/
/****************************************/
/****************************************/

class EvolutionLoopFunctions;
class RunningStat;

class Descriptor
{
public:
  Descriptor();
  bool geometric_median = false;
  size_t num_updates, current_trial;
  static const size_t behav_dim = BEHAV_DIM;

  /* final value of bd*/
  std::vector<std::vector<float>> bd;

  /* prepare for trials*/
  virtual void before_trials(EvolutionLoopFunctions &cLoopFunctions);
  /*reset BD at the start of a trial*/
  virtual void start_trial();
  /*after getting inputs, can update the descriptor if needed*/
  virtual void set_input_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions){};

  /*after getting outputs, can update the descriptor if needed*/
  virtual void set_output_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions){};

  /*after the looping over robots*/
  virtual void after_robotloop(EvolutionLoopFunctions &cLoopFunctions)
  {
  }

  /*end the trial*/
  virtual void end_trial(EvolutionLoopFunctions &cLoopFunctions)
  {
  }

  /*summarise BD at the end of trials*/
  virtual std::vector<float> after_trials(EvolutionLoopFunctions &cLoopFunctions);
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
  virtual void set_input_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions);

  /*end the trial*/
  virtual void end_trial(EvolutionLoopFunctions &cLoopFunctions);
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
  virtual void set_input_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions);
  /*after getting outputs, can update the descriptor if needed*/
  virtual void set_output_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions);
  /*end the trial*/
  virtual void end_trial(EvolutionLoopFunctions &cLoopFunctions);
};

struct RobotAttributeSetter
{
  float maxX,maxY;
  RobotAttributeSetter(CLoopFunctions* cLoopFunctions)
  {
    argos::CVector3 max = cLoopFunctions->GetSpace().GetArenaSize();
	  maxX = max.GetX();
	  maxY = max.GetY();
  }

  virtual std::vector<float> get_attributes(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions)=0;

};
struct NormalAttributeSetter : public RobotAttributeSetter
{
  NormalAttributeSetter(CLoopFunctions* cLoopFunctions) : RobotAttributeSetter(cLoopFunctions)
  {

  }
  virtual  std::vector<float> get_attributes(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions)
  {
    // here just set the attributes of robot at index; let end
    CVector3 pos = cLoopFunctions.curr_pos[robot_index];

    float x = pos.GetX() / maxX;
    float y = pos.GetY() / maxY;
    float theta = cLoopFunctions.curr_theta[robot_index].UnsignedNormalize() / CRadians::TWO_PI; // normalise radians to [0,1]
    float wheel1 = cLoopFunctions.get_controller(robot_index)->left_wheel_velocity_01();
    float wheel2 = cLoopFunctions.get_controller(robot_index)->right_wheel_velocity_01();

    std::vector<float> new_vec = {x, y, theta, wheel1, wheel2};
#ifdef PRINTING
    std::cout << "x,y,theta,w1,w2=" << x << "," << y << "," << theta << "," << wheel1 << "," << wheel2 << std::endl;
#endif
    return new_vec;
  }
};

struct SpeedAttributeSetter : public RobotAttributeSetter
{
  SpeedAttributeSetter(CLoopFunctions* cLoopFunctions) : RobotAttributeSetter(cLoopFunctions)
  {

  }
  virtual  std::vector<float> get_attributes(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions)
  {
    float v_lin = cLoopFunctions.actual_linear_velocity_01(robot_index);
    float v_turn= cLoopFunctions.actual_turn_velocity_01(robot_index);

    std::vector<float> new_vec = {v_lin, v_turn};
#ifdef PRINTING
    std::cout << "v_lin, v_turn =" << v_lin << "," << v_turn << std::endl;
    if (!StatFuns::in_range(v_lin,0.0f,1.0f))
    {
      throw std::runtime_error("v_lin not in [0,1]");
    }
    if (!StatFuns::in_range(v_turn,0.0f,1.0f))
    {
      throw std::runtime_error("v_turn not in [0,1]");
    }
#endif
    return new_vec;
  }
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
  bool is_static; // are entities static or can they move/be moved
  size_t max_size, min_size;
  size_t kappa;                 // feature_size
  std::vector<Entity> entities; // the attributes of the entities, can be obtained by a key (e.g. location or agent id), makes identifying entities in a group easier

  Entity_Group()
  {
  }

  Entity_Group(size_t k, size_t M, size_t m, std::vector<Entity> entity_vec, bool stat) : kappa(k), max_size(M), min_size(m), entities(entity_vec), is_static(stat)
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
  bool include_std, include_closest_robot,geometric_median;
  RobotAttributeSetter* attribute_setter;
  size_t bd_index, num_groups, num_features;
  float maxdist, maxX, maxY;
  std::map<std::string, Entity_Group> entity_groups;
  std::vector<std::string> within_comparison_groups, between_comparison_groups; //
  std::vector<std::string> variable_groups;                                     // contains the keys of groups with variable sizes
  SDBC(CLoopFunctions *cLoopFunctions, std::string init_type);
  void init_walls(CLoopFunctions *cLoopFunctions);
  void init_robots(size_t num_features,CLoopFunctions *cLoopFunctions);
  void init_cylindric_obstacles(CLoopFunctions *cLoopFunctions);

  /* group sizes are the first BD dimensions*/
  void add_group_sizes();

  /* group mean attribute vectors, the second set of BD dimensions*/
  void add_group_meanstates();

  /* avg pair-wise distance within groups, the third set of  BD dimensions*/
  void add_within_group_dispersion();

  /* avg pair-wise distance between groups, the final BD dimensions*/
  void add_between_group_dispersion();

  /* distance to closest robot */
  void add_closest_robot_dist(EvolutionLoopFunctions &cLoopFunctions);

  /* prepare for trials*/
  virtual void before_trials(EvolutionLoopFunctions &cLoopFunctions);
  /*reset BD at the start of a trial*/
  virtual void start_trial();

  /*after getting inputs, can update the descriptor if needed*/
  virtual void set_input_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions);

  /*after getting outputs, can update the descriptor if needed*/
  virtual void set_output_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions);
  /*after the looping over robots*/
  virtual void after_robotloop(EvolutionLoopFunctions &cLoopFunctions);
  /*summarise BD at the end of trials*/
  virtual std::vector<float> after_trials(EvolutionLoopFunctions &cLoopFunctions);
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
  virtual void set_input_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions){};

  /*after getting outputs, can update the descriptor if needed*/
  virtual void set_output_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions){};

  /*summarise BD at the end of trial*/
  virtual std::vector<float> after_trials(Real time, EvolutionLoopFunctions &cLoopFunctions){};
};
/****************************************/
/****************************************/
/* typedef function pointer, useful for fitness functions */
//typedef void (*functionPtr)();

/* given a group of sensory activations, average their input activation, and get the correspinding bin
*/
static size_t get_group_inputactivation(size_t num_bins, std::vector<size_t> group, EvolutionLoopFunctions &cLoopFunctions);

class CVT_MutualInfo : public Descriptor
{
public:
  /* number of bins used for the probability distribution*/
  const size_t num_bins = 2;
  /* number of sensors */
  const size_t num_sensors = ParamsDnn::dnn::nb_inputs - 1;

  /* track the frequencies of the different bins for all groups*/
  std::vector<std::vector<float>> freqs; // for each sensor

  /* track the joint frequencies of the different bins for all groups*/
  std::vector<std::vector<std::vector<float>>> joint_freqs; //for each sensory combination a 2-D matrix

  CVT_MutualInfo();
  /*  *
    float marginal_prob(size_t sensor_index)
    {

    }

  /* prepare for trials*/
  virtual void before_trials(EvolutionLoopFunctions &cLoopFunctions);
  /*reset BD at the start of a trial*/
  virtual void start_trial();

  /*after getting inputs, can update the descriptor if needed*/
  virtual void set_input_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions);

  /*after the looping over robots*/
  virtual void after_robotloop(EvolutionLoopFunctions &cLoopFunctions);

  /*end the trial*/
  virtual void end_trial(EvolutionLoopFunctions &cLoopFunctions);

  /*summarise BD at the end of trials*/
  virtual std::vector<float> after_trials(EvolutionLoopFunctions &cLoopFunctions);

  /* normalise frequencies */
  virtual void normalise();

  /* get the descriptor */
  virtual std::vector<float> get_bd();

  /* calculate MI and check */
  float calc_and_check(size_t i, size_t j);
};
class CVT_MutualInfoAct : public CVT_MutualInfo
{
public:
  /* number of actuators */
  const size_t num_act = ParamsDnn::dnn::nb_outputs;

  /* number of bins used for the probability distribution*/
  const size_t num_act_bins = 5; // for now keep it the same as the sensory bins

  /* track the frequencies of the different bins for all groups*/
  std::vector<std::vector<float>> act_freqs; // for each actuator

  CVT_MutualInfoAct();
  /* calculate entropies */
  virtual float calc_and_check(size_t i, size_t j);
  /* normalise frequencies */
  virtual void normalise();

  /* prepare for trials*/
  virtual void before_trials(EvolutionLoopFunctions &cLoopFunctions);
  /*after getting inputs, can update the descriptor if needed*/
  virtual void set_input_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions){}; //don't do anything yet

  /*after getting outputs, can update the descriptor if needed*/
  virtual void set_output_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions);

  /* get the descriptor */
  virtual std::vector<float> get_bd();
};

class CVT_Spirit : public Descriptor
{
public:
  CVT_Spirit();

  /* smoothing factor */
  const float alpha_smooth = 1.0;

  /* number of joint sensory bins */
  const size_t num_joint_sensory_bins = 16;

  /* number of actuator bins */
  const size_t num_actuator_bins = 5;

  /* number of joint actuator bins */
  const size_t num_joint_actuator_bins = std::pow(num_actuator_bins, 2); // assuming two actuators

  /* track the frequencies of the different bins for all groups*/
  std::vector<std::vector<float>> freqs; // for each sensory state the action probabilities

  /* prepare for trials*/
  virtual void before_trials(EvolutionLoopFunctions &cLoopFunctions);
  /*reset BD at the start of a trial*/
  virtual void start_trial();

  /*after getting outputs, can update the descriptor if needed*/
  virtual void set_output_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions);

  /*after the looping over robots*/
  virtual void after_robotloop(EvolutionLoopFunctions &cLoopFunctions);

  /*end the trial*/
  virtual void end_trial(EvolutionLoopFunctions &cLoopFunctions);

  /*summarise BD at the end of trials*/
  virtual std::vector<float> after_trials(EvolutionLoopFunctions &cLoopFunctions);

  /* normalise frequencies */
  void normalise();

  /* get the descriptor */
  virtual std::vector<float> get_bd();
};
class CompressionDescriptor : public Descriptor
{
public:
  CompressionDescriptor()
  {
  }
  /* given a group of sensory activations, average their activation*/
  float get_group_activation(std::vector<size_t> group, EvolutionLoopFunctions &cLoopFunctions);
  /* prepare for trials*/
  virtual void before_trials(EvolutionLoopFunctions &cLoopFunctions);
  /*reset BD at the start of a trial*/
  virtual void start_trial();
  /*after getting inputs, can update the descriptor if needed*/
  virtual void set_input_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions);

  /*after getting outputs, can update the descriptor if needed*/
  virtual void set_output_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions);

  /*after the looping over robots*/
  virtual void after_robotloop(EvolutionLoopFunctions &cLoopFunctions);

  /*end the trial*/
  virtual void end_trial(EvolutionLoopFunctions &cLoopFunctions);

  /*summarise BD at the end of trials*/
  virtual std::vector<float> after_trials(EvolutionLoopFunctions &cLoopFunctions);
};

// class NonMarkovianStochasticPolicyInduction : public Descriptor
// {
//     /* Obtain non-Markovian estimates of the stochastic policy.
//     *  Rather than utilising the
//     */
//   public:
//     NonMarkovianStochasticPolicyInduction();

//     /* prepare for trials*/
//     virtual void before_trials(EvolutionLoopFunctions &cLoopFunctions){};
//     /*reset BD at the start of a trial*/
//     virtual void start_trial(){};
//     /*after getting inputs, can update the descriptor if needed*/
//     virtual void set_input_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions){};

//     /*after getting outputs, can update the descriptor if needed*/
//     virtual void set_output_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions){};

//     /*after the looping over robots*/
//     virtual void after_robotloop(EvolutionLoopFunctions &cLoopFunctions)
//     {
//     }

//     /*end the trial*/
//     virtual void end_trial(EvolutionLoopFunctions &cLoopFunctions)
//     {
//     }

//     /*summarise BD at the end of trials*/
//     virtual std::vector<float> after_trials(EvolutionLoopFunctions &cLoopFunctions){};
// };

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
  CVT_Trajectory(EvolutionLoopFunctions &cLoopFunctions, size_t num_steps);

  inline bool end_chunk() const
  {
    return (num_updates + 1) % periodicity == 0;
  }

  virtual void before_trials(EvolutionLoopFunctions &cLoopFunctions);

  /*after getting outputs, can update the descriptor if needed*/
  virtual void set_output_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions);
  /*summarise BD at the end of trials*/
  virtual std::vector<float> after_trials(EvolutionLoopFunctions &cLoopFunctions);
};
// #endif

// class CVT_Damage : public Descriptor
// {
//     /*
//     *
//     */
//   public:
//     size_t num_chunks;
//     size_t periodicity;
//     float maxX, maxY;
//     /* final behavioural descriptor  */
//     std::vector<float> final_bd;
//     CVT_Trajectory(EvolutionLoopFunctions &cLoopFunctions, size_t num_steps);

//     inline bool end_chunk() const
//     {
//         return (num_updates + 1) % periodicity == 0;
//     }

//     virtual void before_trials(EvolutionLoopFunctions &cLoopFunctions);

//     /*after getting outputs, can update the descriptor if needed*/
//     virtual void set_output_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions);
//     /*summarise BD at the end of trials*/
//     virtual std::vector<float> after_trials(EvolutionLoopFunctions &cLoopFunctions);
// };
