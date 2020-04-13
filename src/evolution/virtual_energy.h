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

enum VirtualState
{
    DEFAULT,
    NEST,
    HOLDING_FOOD
};
struct VirtualEnergy
{

    float E;
    float default_reward, food_reward, nest_reward;
    size_t counter;
    std::vector<VirtualState> previous_state;
    VirtualEnergy(float num_agents, float steps_to_1m)
    {
        // initialise
        E = 4.0 * steps_to_1m * num_agents;
        food_reward = steps_to_1m / 2.0;
        nest_reward = 2.0 * steps_to_1m;
        for(size_t i=0; i < num_agents; ++i)
        {
            previous_state.push_back(DEFAULT);
        }
    }
    /* main step function, increasing or decreasing energy depending on subgoal attainment */
    bool step(bool collide, VirtualState state);
    /* whether or not energy is depleted */
    inline bool depleted()
    {
        
        counter = 0; //loop is finished, set counter back to 0
        return this->E <= 0;
    }
};