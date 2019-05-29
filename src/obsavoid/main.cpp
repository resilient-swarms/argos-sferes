
/****************************************/
/****************************************/
/* ARGoS-related headers */
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/loop_functions.h>

#include <src/obsavoid/evol_loop_functions.h>
#include <src/obsavoid/statistics.h>
#include <src/obsavoid/fitness_functions.h>
#include <src/obsavoid/descriptors.h>


/****************************************/
/****************************************/
#ifdef CVT

typedef Params::ea::point_t point_t;

std::vector<point_t> load_centroids(const std::string& centroids_filename)
{
    std::vector<point_t> centroids;

    std::ifstream fin(centroids_filename.c_str());

    if (!fin) {
        std::cout << centroids_filename <<std::endl;
        std::cerr << "Error: Could not load the centroids." << std::endl;
        exit(1);
    }

    std::vector<std::string> lines;

    std::string line;
    while (std::getline(fin, line)) {
        if (!line.empty()) {
            lines.push_back(line);
        }
    }

    fin.close();

    if (lines.size() != Params::ea::number_of_clusters) {
        std::cerr << "Error: The number of clusters "
                  << Params::ea::number_of_clusters
                  << " is not equal to the number of loaded elements "
                  << lines.size() << ".\n";
        exit(1);
    }

    for (size_t i = 0; i < lines.size(); ++i) {
        std::vector<std::string> cols;

        std::string temp;
        std::istringstream stringStream;
        stringStream.str(lines[i]);

        while (stringStream >> temp)
            cols.push_back(temp);

        if (cols.size() != Params::ea::number_of_dimensions) {
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
int main(int argc, char **argv)
{
    //std::cout<<"tag="+std::string(TAG)<<std::endl;
    /*
     * Initialize ARGoS
     */
    /* The CSimulator class of ARGoS is a singleton. Therefore, to
     * manipulate an ARGoS experiment, it is enough to get its instance */
    argos::CSimulator &cSimulator = argos::CSimulator::GetInstance();
    /* Set the .argos configuration file
     * This is a relative path which assumed that you launch the executable
     * from argos3-examples (as said also in the README) */
    cSimulator.SetExperimentFileName(argv[1]);

    /* Load it to configure ARGoS */
    cSimulator.LoadExperiment();

    static CObsAvoidEvolLoopFunctions &cLoopFunctions = dynamic_cast<CObsAvoidEvolLoopFunctions &>(cSimulator.GetLoopFunctions());

#ifdef CVT
    Params::ea::centroids = load_centroids(cLoopFunctions.centroids_folder+"/centroids_1000_"+std::to_string(Params::ea::number_of_dimensions)+".dat");
#endif
    //typedef FitObstacle<Params> fit_t;
    typedef FitObstacleMapElites<Params> fit_t;
    typedef phen::Dnn<robots_nn::gen_t, fit_t, ParamsDnn> phen_t;
    typedef eval::Eval<Params> eval_t; //eval::Parallel gives cryptic seg fault for nn. Unrelated but make sure visualization is disabled when parallelizing
    //typedef boost::fusion::vector<sferes::stat::ParetoFront<phen_t, Params> >  stat_t;

    

    //MODIFIER
    typedef modif::Dummy<> modifier_t;

    #ifdef CVT
        
        typedef boost::fusion::vector<stat::Map<phen_t, Params>, stat::MapProgress<phen_t, Params>> stat_t;
    #else
        typedef boost::fusion::vector<
        sferes::stat::Map<phen_t, Params>,
        sferes::stat::MapProgress<phen_t, Params>>
        stat_t;
    #endif
    #ifdef CVT
        //typedef ea::Nsga2<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;
        typedef ea::CVTMapElites<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;
        //typedef ea::Nsga2<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;
    #else
        //typedef ea::Nsga2<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;
        typedef ea::MapElites<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;
        //typedef ea::Nsga2<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;
    #endif
    
    ea_t ea;

    run_ea(argc, argv, ea);

    /*
    * Dispose of ARGoS stuff
    */
    argos::CSimulator::GetInstance().Destroy();

    /* All is OK */
    return 0;
}

/****************************************/
/****************************************/


