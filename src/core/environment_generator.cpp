
#include <src/core/environment_generator.h>
#include <src/core/base_loop_functions.h>


/* generates the configuration parameters just before the reset */
void EnvironmentGenerator::generate(BaseLoopFunctions* cLoopFunctions)
{

    size_t rob_index = robot_dist(rng);
    cLoopFunctions->m_unNumberRobots = num_robots[rob_index];
    //std::cout<<cLoopFunctions->m_unNumberRobots<<std::endl;
    //std::cout<<cLoopFunctions->m_pcvecRobot.size()<<std::endl;
    size_t cyl_index = cylinder_dist(rng);
    cLoopFunctions->m_unNumberCylinders = num_cylinders[cyl_index];
    //std::cout<<cLoopFunctions->m_unNumberCylinders<<std::endl;
    //std::cout<<cLoopFunctions->m_pcvecCylinder.size()<<std::endl;
    cLoopFunctions->adjust_number_agents();// do it here rather than reset because more efficient 
    cLoopFunctions->adjust_number_cylinders();// but also because need vec_ctrllob
}