
#include <argos3/core/simulator/simulator.h>

#include <random>


class BaseLoopFunctions;
// class EnvironmentGenerator
// {
//   public:
//     EnvironmentGenerator()
//     {
//     }
//     virtual void generate(argos::CSimulator& cSimulator);
// };


// class ConfigurationBasedGenerator : public EnvironmentGenerator
// {
// private:
//   std::string filename;// has its own associated configuration file
// public:
//   ConfigurationBasedGenerator(std::string file_name)
//   {
//     filename = file_name;
//   }
//   virtual void generate(argos::CSimulator& cSimulator);
// };




struct EnvironmentGenerator
{
  // define all the possible values of the environment (5000 in total)
  // choices based on those that lead to the most variety in behaviour


  /* number of robots */
  const std::vector<size_t> num_robots = {3,4,5,6,7};//5 options

  /* arena size */
  //const std::vector<size_t> arena_size = {2,4,6};//3 options

  /* cylindric obstacles */
  //const std::vector<size_t> num_cylinders = {0,2,4,8,16};//3 options
  // random number generator
  std::mt19937 rng;
  std::uniform_int_distribution<int> robot_dist;
  //const std::uniform_int_distribution<int> arena_dist(0,2);
  //std::uniform_int_distribution<int> cylinder_dist(0,4);
  EnvironmentGenerator(int seed)
  {
      rng.seed(seed);
      robot_dist = std::uniform_int_distribution<int>(0,4);
  }

  void generate(BaseLoopFunctions* cLoopFunctions);

};