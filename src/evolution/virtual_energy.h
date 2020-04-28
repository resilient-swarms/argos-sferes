/* implementation of virtual energy based on three types of subgoals:

– g 1 : this goal gives a negative reward r = −2, twice the default reward of
r = −1, for colliding with other robots or the wall.
– g 2 : this goal gives a positive reward r = 10 when a robot has a visitation
pattern "food | ∗ nest food"
– g 3 : this goal gives a positive reward r = d when a robot has a visitation
pattern "∗ food nest", where d is the distance of the nest to the food.

*/

#include <cstddef>
#include <vector>
#include <iostream>

enum VirtualState
{
    DEFAULT,
    NEST,
    HOLDING_FOOD
};
struct VirtualEnergy
{

    float E;
    float init_reward, default_reward, food_reward, nest_reward;
    std::vector<VirtualState> previous_state;
    VirtualEnergy(float num_agents, float steps_to_1m)
    {
        // initialise
        init_reward = 4.0 * steps_to_1m * num_agents;
        food_reward = steps_to_1m;
        nest_reward = 5.0 * steps_to_1m;
        for (size_t i = 0; i < num_agents; ++i)
        {
            previous_state.push_back(DEFAULT);
        }

        std::cout << "init reward = "  << init_reward << std::endl;
        std::cout << "food reward = "  << food_reward << std::endl;
        std::cout << "nest reward = "  << nest_reward << std::endl;
        E = init_reward;
    }
    void reset()
    {
        for (size_t i = 0; i < previous_state.size(); ++i)
        {
            previous_state[i] = DEFAULT;
        }
        E = init_reward;
    }
    /* main step function, increasing or decreasing energy depending on subgoal attainment */
    bool step(size_t  j, bool collide, VirtualState state);
    /* whether or not energy is depleted */
    inline bool depleted()
    {
        return this->E <= 0;
    }
};
