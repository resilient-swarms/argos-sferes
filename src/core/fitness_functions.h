

// #ifndef FITNESS_FUNCTIONS
// #define FITNESS_FUNCTIONS

#include <vector>
#include <src/core/arena_utils.h>


class FitFun
{
public:
  std::vector<float> fitness_per_trial;
  FitFun(){};
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
  virtual void apply(BaseLoopFunctions &cLoopFunctions, Real time) = 0;

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
  virtual void apply(BaseLoopFunctions &cLoopFunctions, Real time);

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
  virtual void apply(BaseLoopFunctions &cLoopFunctions, Real time);

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
  CoverageCalc* coverageCalc;
  Coverage(std::string init_type, BaseLoopFunctions *cLoopFunctions);
  virtual inline bool quit_on_collision() const
  {
    return true;
  }
  /*after a single step of all agents */
  virtual void after_robotloop(BaseLoopFunctions &cLoopFunctions);
  /*after completing trial, calc fitness*/
  virtual void apply(BaseLoopFunctions &cLoopFunctions, Real time);

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
  DecayCoverageCalc* coverageCalc;
  DecayCoverage(std::string init_type, BaseLoopFunctions *cLoopFunctions);

  /*after a single step of all agents */
  virtual void after_robotloop(BaseLoopFunctions &cLoopFunctions);
  /*after completing trial, calc fitness*/
  virtual void apply(BaseLoopFunctions &cLoopFunctions, Real time);

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
  virtual void apply(BaseLoopFunctions &cLoopFunctions, Real time);
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
  virtual void apply(BaseLoopFunctions &cLoopFunctions, Real time);
  /*after completing all trials, combine fitness*/
  virtual float after_trials();
  /*after a single step of all agents */
  virtual void after_robotloop(BaseLoopFunctions &cLoopFunctions);

  float avg_min_dist(BaseLoopFunctions &cLoopFunctions);
};

// class Flocking : public FitFun
// {
//   /* Gomes & Christensen 2018
//   The fitness function rewards robots for having an orientation 
//   similar to the other robots within a radius of 25 cm (half the robot sensing range), 
//   and for moving as fast as possible.*/
// public:
//   Flocking(BaseLoopFunctions *cLoopFunctions);

//   /*after completing trial, calc fitness*/
//   virtual void apply(BaseLoopFunctions &cLoopFunctions, Real time);
//   /*after completing all trials, combine fitness*/
//   virtual float after_trials();
//   /*after a single step of all agents */
//   virtual void after_robotloop(BaseLoopFunctions &cLoopFunctions);
// };

// #endif