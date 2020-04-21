

#include <src/core/statistics.h>

struct ForagingStats
{

    /* time on food source without harvesting = lost time */
    float not_harvesting_times = 0.0f;
    /* time steps without holding food*/
    float steps_without_food = 0.0f;

    /* nest visitiations without holding food */
    float nest_without_food = 0.0f;

    /* normalisation after all trials finished*/
    float norm;

    /* write to file */
    std::ofstream stat_writer;
    ForagingStats(std::string outputdir,size_t agents, size_t trials) : norm(trials*agents), stat_writer(outputdir+"/foraging_stats_archive.txt",std::ios::app)
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
        stat_writer <<  not_harvesting_times/norm << " " << steps_without_food/norm
            << " " << nest_without_food/norm <<std::endl;
    }
};