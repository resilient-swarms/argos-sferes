
#include <argos3/core/simulator/simulator.h>

#include <random>


class BaseLoopFunctions;


struct EnvironmentGenerator
{
  // define all the possible values of the environment (5000 in total)
  // choices based on those that lead to the most variety in behaviour


  /* number of robots */
  const std::vector<size_t> num_robots = {5,10};//5 options

  /* arena size */
  //const std::vector<size_t> arena_size = {2,4,6};//3 options

  /* cylindric obstacles */
  const std::vector<size_t> num_cylinders = {3,6};//3 options

  /* wheel */
  // random number generator
  std::mt19937 rng;
  std::uniform_int_distribution<int> robot_dist;
  //const std::uniform_int_distribution<int> arena_dist(0,2);
  std::uniform_int_distribution<int> cylinder_dist;
  EnvironmentGenerator(int seed)
  {
      rng.seed(seed);
      robot_dist = std::uniform_int_distribution<int>(0,num_robots.size()-1);
      cylinder_dist = std::uniform_int_distribution<int>(0,num_cylinders.size()-1);
  }

  void generate(BaseLoopFunctions* cLoopFunctions);

};