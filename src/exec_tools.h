/* this header includes files and functions needed in the executables */

/* ARGoS-related headers */
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/loop_functions.h>

#include <src/evolution/loopfunctions_include.h>
#include <src/parallel/argosparallelenviron_eval.h>

#include <sys/types.h>
#include <unistd.h>

#ifdef CVT

typedef EAParams::ea::point_t point_t;

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
    
    if (lines.size() != EAParams::ea::number_of_clusters)
    {
        std::cerr << "Error: The number of clusters "
                  << EAParams::ea::number_of_clusters
                  << " is not equal to the number of loaded elements "
                  << lines.size() << ".\n";
        exit(1);
    }
    //size_t num_cols;
    for (size_t i = 0; i < lines.size(); ++i)
    {
        std::vector<std::string> cols;

        std::string temp;
        std::istringstream stringStream;
        stringStream.str(lines[i]);

        while (stringStream >> temp)
            cols.push_back(temp);
        //num_cols = cols.size();
        //std::cout << "line "<< i << "num cols = "<< num_cols;
        if (cols.size() != EAParams::ea::number_of_dimensions)
        {
            std::cerr << "Error: The number of dimensions "
                      << EAParams::ea::number_of_dimensions
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
std::vector<point_t> EAParams::ea::centroids;

#endif

typedef FitObstacleMapElites<EAParams> fit_t;
typedef phen::Dnn<robots_nn::gen_t, fit_t, ParamsDnn> phen_t;
//MODIFIER
typedef modif::Dummy<> modifier_t;

template <typename Params_t, typename Phen_t, typename eval_t>
struct T
{

#ifdef CVT
    typedef boost::fusion::vector<stat::Map<Phen_t, Params_t>, stat::MapProgress<Phen_t, Params_t>> stat_t;
    typedef ea::CVTMapElites<Phen_t, eval_t, stat_t, modifier_t,Params_t> ea_t;
#else
    typedef boost::fusion::vector<
        sferes::stat::Map<Phen_t, Params_t>,
        sferes::stat::MapProgress<Phen_t, Params_t>>
        stat_t;
    typedef ea::MapElites<Phen_t, eval_t, stat_t, modifier_t, Params_t> ea_t;
#endif
};
typedef T<EAParams,phen_t,eval::Eval<EAParams>>::ea_t serial_ea_t;
typedef T<EAParams,phen_t,eval::ArgosParallel<EAParams>>::ea_t parallel_ea_t;


template <typename ea_t>
void configure_and_run_ea(int argc, char **argv)
{
    ea_t ea;
    run_ea(argc, argv, ea);
}


template<typename Param_t>
void init_shared_mem()
{
    // times 2 because two individuals generated per reproduction
    sferes::eval::num_memory = 2 * Param_t::pop::size;// for the initial population, allocate on the spot
    sferes::eval::shared_memory.clear();
    for (size_t i = 0; i < sferes::eval::num_memory; ++i)
    {
        sferes::eval::shared_memory.push_back(new sferes::eval::CSharedMem(BEHAV_DIM));
    }
}