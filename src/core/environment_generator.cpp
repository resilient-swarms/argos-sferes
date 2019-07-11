
#include <src/core/environment_generator.h>
#include <src/core/base_loop_functions.h>

// void EnvironmentGenerator::generate(argos::CSimulator& cSimulator)
// {

// }

// void ConfigurationBasedGenerator::generate(argos::CSimulator& cSimulator)
// {



// }




/* generates the configuration parameters just before the reset */
void EnvironmentGenerator::generate(BaseLoopFunctions* cLoopFunctions)
{

    size_t rob_index = robot_dist(rng);
    cLoopFunctions->m_unNumberRobots = num_robots[rob_index];
    cLoopFunctions->adjust_number_agents();
    //size_t cyl_index = cylinder_dist(e1);
    //cLoopFunctions->m_unNumberCylinders = num_c
}