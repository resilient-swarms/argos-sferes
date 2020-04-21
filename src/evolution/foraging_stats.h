

#include <src/core/statistics.h>

struct ForagingStats
{

    /* time on food source without harvesting = lost time */
    float not_harvesting_times = 0.0f;
    /* time steps without holding food*/
    float steps_without_food = 0.0f;

    /* nest visitiations without holding food */
    float nest_without_food = 0.0f;

    float num_agents;

    /* number of trials */
    size_t num_trials;

    /* write to file */
    std::ofstream stat_writer;
    ForagingStats(std::string outputdir,float agents) : num_agents(agents), stat_writer(outputdir+"/foraging_stats.txt")
    {
    }

    void count_notharvesting()
    {
        ++not_harvesting_times;
    }

    void count_stepswithoutholdingfood()
    {
        ++steps_without_food;
    }

    void count_nestvisitwithoutholdingfood()
    {
        ++nest_without_food;
    }

    void write()
    {
        stat_writer <<  not_harvesting_times/num_agents  << " " << steps_without_food/num_agents 
            << " " << nest_without_food/num_agents <<std::endl;
        not_harvesting_times = 0.0f;
        steps_without_food = 0.0f;
        nest_without_food = 0.0f;
    }
};