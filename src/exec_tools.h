/* this header includes files and functions needed in the executables */

/* ARGoS-related headers */
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/loop_functions.h>

#include <src/evolution/evol_loop_functions.h>
#include <src/parallel/argosparallelenviron_eval.h> 

#include <sys/types.h>
#include <unistd.h>

#ifdef CVT

typedef Params::ea::point_t point_t;

std::vector<point_t> load_centroids(const std::string &centroids_filename)
{
    std::vector<point_t> centroids;

    std::ifstream fin(centroids_filename.c_str());

    if (!fin)
    {
        std::cout << centroids_filename << std::endl;
        std::cerr << "Error: Could not load the centroids." << std::endl;
        exit(1);
    }

    std::vector<std::string> lines;

    std::string line;
    while (std::getline(fin, line))
    {
        if (!line.empty())
        {
            lines.push_back(line);
        }
    }

    fin.close();

    if (lines.size() != Params::ea::number_of_clusters)
    {
        std::cerr << "Error: The number of clusters "
                  << Params::ea::number_of_clusters
                  << " is not equal to the number of loaded elements "
                  << lines.size() << ".\n";
        exit(1);
    }

    for (size_t i = 0; i < lines.size(); ++i)
    {
        std::vector<std::string> cols;

        std::string temp;
        std::istringstream stringStream;
        stringStream.str(lines[i]);

        while (stringStream >> temp)
            cols.push_back(temp);

        if (cols.size() != Params::ea::number_of_dimensions)
        {
            std::cerr << "Error: The number of dimensions "
                      << Params::ea::number_of_dimensions
                      << " is not equal to the dimensionality (" << cols.size()
                      << ") of loaded element with index " << i << ".\n";
            exit(1);
        }

        point_t p;
        for (size_t j = 0; j < cols.size(); ++j)
            p[j] = atof(cols[j].c_str());

        centroids.push_back(p);
    }

    std::cout << "\nLoaded " << centroids.size() << " centroids.\n";

    return centroids;
}
std::vector<point_t> Params::ea::centroids;

#endif


typedef FitObstacleMapElites<Params> fit_t;
typedef phen::Dnn<robots_nn::gen_t, fit_t, ParamsDnn> phen_t;
//MODIFIER
typedef modif::Dummy<> modifier_t;

template<typename eval_t>
struct T
{

#ifdef CVT
    typedef boost::fusion::vector<stat::Map<phen_t, Params>, stat::MapProgress<phen_t, Params>> stat_t;
    typedef ea::CVTMapElites<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;
#else
    typedef boost::fusion::vector<
        sferes::stat::Map<phen_t, Params>,
        sferes::stat::MapProgress<phen_t, Params>>
        stat_t;
    typedef ea::MapElites<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;
#endif

};
typedef T<eval::Eval<Params>>::ea_t serial_ea_t;
typedef T<eval::ArgosParallel<Params>>::ea_t parallel_ea_t;
typedef T<eval::ArgosParallelEnvir<Params>>::ea_t parallelenvir_ea_t;

template<typename ea_t>
void configure_and_run_ea(int argc, char** argv)
{
    ea_t ea;
    run_ea(argc, argv, ea);
}