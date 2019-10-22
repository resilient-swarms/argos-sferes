

// #ifndef FITNESS_FUNCTIONS
// #define FITNESS_FUNCTIONS

#include <vector>
#include <src/core/arena_utils.h>

class FitFun
{
public:
  size_t num_updates;
  std::vector<float> fitness_per_trial;
  FitFun(){

  };
  virtual inline bool quit_on_collision() const
  {
    return false;
  }
  /* before trial */
  virtual void before_trial(BaseLoopFunctions &cLoopFunctions)
  {
    //empty by default
  }
  /*after a single step of a single agent */
  virtual void after_step(size_t robot_index, BaseLoopFunctions &cLoopFunctions){};
  /*after a single step of all agents */
  virtual void after_robotloop(BaseLoopFunctions &cLoopFunctions){};
  /*after completing trial, calc fitness*/
  virtual void apply(BaseLoopFunctions &cLoopFunctions) = 0;

  /*after completing all trials, combine fitness*/
  virtual float after_trials() = 0;

  /*after completing a trial, print some statistics (if desired)*/
  virtual void print_progress(size_t trial)
  {
    printf("\n\n fitness in trial %zu is %f", trial, fitness_per_trial[trial]);
  }
};

class FloreanoMondada : public FitFun
{
  //Floreano & Mondada '94; minimum of the linear speed, controlling for collision
public:
  Real nb_coll = 0;
  float speed = 0.0f;
  float lin_speed = 0.0f;
  float num_ds = 0.0f;
  FloreanoMondada();
  /*after a single step of all agents */
  virtual void after_robotloop(BaseLoopFunctions &cLoopFunctions);
  /*after completing trial, calc fitness*/
  virtual void apply(BaseLoopFunctions &cLoopFunctions);

  /*after completing all trials, combine fitness*/
  virtual float after_trials();

  /*after completing a trial, print some statistics (if desired)*/
  virtual void print_progress(size_t trial);
};

class MeanSpeed : public FitFun
{
  // same as FloreanoMondada but take mean instead of minimum
public:
  Real nb_coll = 0;
  float speed = 0.0f;
  float lin_speed = 0.0f;
  float num_ds = 0.0f;
  MeanSpeed();
  /*after a single step of all agents */
  virtual void after_robotloop(BaseLoopFunctions &cLoopFunctions);
  /*after completing trial, calc fitness*/
  virtual void apply(BaseLoopFunctions &cLoopFunctions);

  /*after completing all trials, combine fitness*/
  virtual float after_trials();

  /*after completing a trial, print some statistics (if desired)*/
  virtual void print_progress(size_t trial);
};

class Coverage : public FitFun
{
  // Coverage as the number of visited cells divided by total cells
public:
  size_t num_updates = 0;
  CoverageCalc *coverageCalc;
  Coverage(std::string init_type, BaseLoopFunctions *cLoopFunctions);
  virtual inline bool quit_on_collision() const
  {
    return true;
  }
  /*after a single step of all agents */
  virtual void after_robotloop(BaseLoopFunctions &cLoopFunctions);
  /*after completing trial, calc fitness*/
  virtual void apply(BaseLoopFunctions &cLoopFunctions);

  /*after completing all trials, combine fitness*/
  virtual float after_trials();

  /*after completing a trial, print some statistics (if desired)*/
  virtual void print_progress(size_t trial);
};

class TrialCoverage : public Coverage
{
  // Coverage but account for different trials
public:
  TrialCoverage(std::string init_type, BaseLoopFunctions *cLoopFunctions);
  virtual inline bool quit_on_collision() const
  {
    return true;
  }
  virtual void before_trial(BaseLoopFunctions &cLoopFunctions);
};

class DecayCoverage : public FitFun
{
  // Gomes & Christensen 2018
  // The arena is discretised into a grid of 10 × 10,
  // and every time cell is visited by a robot, the value of the cell goes to 1, and then
  // decays constantly at a rate of 0.005/s.
  // The fitness is the average value of all cells over the entire simulation.
public:
  size_t num_updates = 0;
  DecayCoverageCalc *coverageCalc;
  DecayCoverage(std::string init_type, BaseLoopFunctions *cLoopFunctions);

  /*after a single step of all agents */
  virtual void after_robotloop(BaseLoopFunctions &cLoopFunctions);
  /*after completing trial, calc fitness*/
  virtual void apply(BaseLoopFunctions &cLoopFunctions);

  /*after completing all trials, combine fitness*/
  virtual float after_trials();
};

class Aggregation : public FitFun
{
  // Gomes & Christensen 2018
  // The fitness function is inversely
  //proportional to the average distance to the centre of mass over the entire simulation
public:
  /* keep track of the number of updates */
  size_t num_updates = 0;
  /* keep track if the distance during the trial */
  float trial_dist = 0.0f;
  /* the maximal theoretical distance between agents */
  float maxdist;
  Aggregation(BaseLoopFunctions *cLoopFunctions);

  /*after completing trial, calc fitness*/
  virtual void apply(BaseLoopFunctions &cLoopFunctions);
  /*after completing all trials, combine fitness*/
  virtual float after_trials();
  /*after a single step of all agents */
  virtual void after_robotloop(BaseLoopFunctions &cLoopFunctions);
  float get_mass(CThymioEntity *robot);

  std::pair<std::vector<argos::CVector3>, argos::CVector3> centre_of_mass(BaseLoopFunctions &cLoopFunctions);
};

class Dispersion : public FitFun
{
  /* Gomes & Christensen 2018
  The fitness is proportional to the average distance
  to the nearest neighbour, averaged over the entire simulation.*/
public:
  size_t num_updates = 0;
  float maxdist;
  float trial_dist = 0.0f;
  Dispersion(BaseLoopFunctions *cLoopFunctions);

  /*after completing trial, calc fitness*/
  virtual void apply(BaseLoopFunctions &cLoopFunctions);
  /*after completing all trials, combine fitness*/
  virtual float after_trials();
  /*after a single step of all agents */
  virtual void after_robotloop(BaseLoopFunctions &cLoopFunctions);

  float avg_min_dist(BaseLoopFunctions &cLoopFunctions);
};

class Flocking : public FitFun
{
  /* Gomes & Christensen 2018
  The fitness function rewards robots for having an orientation 
  similar to the other robots within a radius of 25 cm (half the robot sensing range), 
  and for moving as fast as possible.*/
public:
  /* range of neighbour in which to optimise similar orientation and high speed */
  float flocking_range;
  float accumulator;
  size_t num_updates = 0;
  Flocking(BaseLoopFunctions *cLoopFunctions);

  /*after completing trial, calc fitness*/
  virtual void apply(BaseLoopFunctions &cLoopFunctions);
  /*after completing all trials, combine fitness*/
  virtual float after_trials();
  /*after a single step of all agents */
  virtual void after_robotloop(BaseLoopFunctions &cLoopFunctions);
};

/***********************************/

class Chaining : public FitFun
{
  /* The fitness function is inversely proportional to
     * the average distance between the closest robots connected through RAB to the
     * source and destination by a chain (or network) of robots.
     *
     * The number of robots used in this fitness function is n-2 as two robots are set aside
     * by this function to act as the source and destination.
     * TODO: The source and destination robots can be stationary or moving.
     * */
public:
  Chaining();

  /* source */
  const size_t src_robot_id = 0;
  /* destination */
  const size_t dest_robot_id = 1;
  /* keep track of the number of updates */
  size_t num_updates = 0;
  /* keep track if the distance during the trial */
  float trial_dist = 0.0f;

  virtual void before_trial(BaseLoopFunctions &cLoopFunctions);
  /*after completing trial, calc fitness*/
  virtual void apply(BaseLoopFunctions &cLoopFunctions);
  /*after completing all trials, combine fitness*/
  virtual float after_trials();
  /*after a single step of all agents */
  virtual void after_robotloop(BaseLoopFunctions &cLoopFunctions);
  /* calculate the smallest distance between networked robots connected to the source and destination */
  float min_connected_dist(BaseLoopFunctions &cLoopFunctions, float maxdist);
  /* make sure the source and destination are the maximally distant robots */
  void swap_robots(BaseLoopFunctions &cLoopFunctions, size_t robot_a, size_t robot_b);
};

// #endif

class Foraging : public FitFun
{
  /*
     *
     */
public:

  float reward;
  Foraging(){};
  const float nest_x = -1;
  const float m_fFoodSquareRadius = 0.25;
  const size_t num_food = 10;
  /*

     */
  const std::vector<CVector3> m_cFoodPos = {
      CVector3(-1.5,-1.5,0.0),
      CVector3(-1.5,-1.0,0.0),
      CVector3(-1.5,1.0,0.0),
      CVector3(-0.5,1.5,0.0),
      CVector3(1.5,1.5,0.0),
      CVector3(1.5,-1.,0.0),
      CVector3(1.5,-1.5,0.0),
      CVector3(1.0,1.0,0.0),
      CVector3(1.0,-1.5,0.0),
      CVector3(1.5,-0.5,0.0)};

  size_t num_updates = 0;
  std::vector<bool> m_cVisitedFood = {};
  std::vector<bool> m_bRobotsHoldingFood = {};
  size_t numfoodCollected = 0;
  float trial_performance = 0;

  virtual void before_trial(BaseLoopFunctions &cLoopFunctions);
  /*after completing trial, calc fitness*/
  virtual void apply(BaseLoopFunctions &cLoopFunctions);
  /*after completing all trials, combine fitness*/
  virtual float after_trials();
  /*after a single step of all agents */
  virtual void after_robotloop(BaseLoopFunctions &cLoopFunctions);
};
