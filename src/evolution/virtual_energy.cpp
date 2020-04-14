#include <src/evolution/virtual_energy.h>
#include <iostream>

/* main step function, increasing or decreasing energy depending on subgoal attainment */
bool VirtualEnergy::step(size_t j, bool collide, VirtualState state)
{
    float reward;
    if (state == DEFAULT)
    {
        reward = -1.0;
        //std::cout << "Agent " << j << "default :" << reward << std::endl;
    }
    // g2: positive reward r = 10 when a robot is holding food
    else if (state == HOLDING_FOOD && previous_state[j] != HOLDING_FOOD)
    {
        reward = food_reward;
        //std::cout << "Agent " << j << "holding_food :" << reward << std::endl;
    }
    // g3:  positive reward r = 100 when a robot was holding food and is now in the nest
    else if (state == NEST && previous_state[j] == HOLDING_FOOD)
    {
        reward = nest_reward;
        //std::cout << "Agent " << j << "nest_reward :" << reward << std::endl;
    }
    else
    {
        // count as default as well
        reward = -1.0;
        //std::cout << "Agent " << j << "default :" << reward << std::endl;
    }
    //penalty for collisions
    if (collide)
    {
        reward -= 1.0; // small penalty for collision, on top of other rewar
    }
    //finally, update the energy
    E += reward;
    previous_state[j] = state;
#ifdef PRINTING
    std::cout << "E = " << E << std::endl;
#endif
}

