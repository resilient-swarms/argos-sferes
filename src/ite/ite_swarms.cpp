#include <iostream>
#include <sstream>
#include <string>

#include <limbo/limbo.hpp>

#include <exhaustive_search_archive.hpp>
#include <mean_archive.hpp>


using namespace limbo;

namespace global
{
    std::string results_path;
    std::string argossim_bin_name;
    std::string argossim_config_name;
    std::string archive_path;   
    float original_max = - std::numeric_limits<float>::infinity(); 
    unsigned gen_to_load;
    unsigned behav_dim = BEHAV_DIM; // number of dimensions of MAP
} // namespace global


template <typename Params>
struct PercentageMax{
    PercentageMax() {}

    template <typename BO, typename AggregatorFunction>
    bool operator()(const BO& bo, const AggregatorFunction& afun)
    {
        return afun(bo.best_observation(afun)) > 0.90*global::original_max;
    }
};


struct Params
{

    

    struct bayes_opt_boptimizer : public defaults::bayes_opt_boptimizer
    {};

    struct bayes_opt_bobase : public defaults::bayes_opt_bobase
    {
        BO_PARAM(int, stats_enabled, true);
    };

    // no noise
    struct kernel : public defaults::kernel
    {
        BO_PARAM(double, noise, 1e-10);
    };

    struct kernel_maternfivehalves : public defaults::kernel_maternfivehalves
    {
        BO_PARAM(double, l, 0.4);
    };

    struct stop_maxiterations
    {
        //BO_DYN_PARAM(int, iterations);
        BO_PARAM(int, iterations, 20);
    };


    struct stop_maxpredictedvalue
    {
        //BO_DYN_PARAM(int, iterations);
        BO_PARAM(double, ratio, 0.9);
    };

    struct acqui_ucb : public defaults::acqui_ucb
    {
        BO_PARAM(double, alpha, 0.2);
    };

    struct archiveparams
    {
        struct elem_archive
        {
            std::vector<double> behav_descriptor; // the first entry of elem_archive should be the behaviour descriptor (see ln 19 in exhaustive_search_archive.hpp)
            float fit;
            unsigned controller;
        };

        struct classcomp
        {
            /* to sort the std::map */
            bool operator()(const std::vector<double>& lhs, const std::vector<double>& rhs) const
            {
                assert(lhs.size() == global::behav_dim && rhs.size() == global::behav_dim);
                size_t i = 0;
                while (i < (global::behav_dim-1) && std::round(lhs[i] * 100.0)/100.0 == std::round(rhs[i] * 100.0)/100.0) //lhs[i]==rhs[i])
                    i++;
                return std::round(lhs[i] * 100.0)/100.0 < std::round(rhs[i] * 100.0)/100.0; //lhs[i]<rhs[i];
            }
        };
        typedef std::map<std::vector<double>, elem_archive, classcomp> archive_t;
        static std::map<std::vector<double>, elem_archive, classcomp> archive;
    };
};

Params::archiveparams::archive_t load_archive(std::string archive_name);

struct Eval
{
    BO_PARAM(size_t, dim_in, BEHAV_DIM); //global::behav_dim
    BO_PARAM(size_t, dim_out, 1);

    // the function to be optimized
    Eigen::VectorXd operator()(const Eigen::VectorXd& x) const
    {
        std::cout << "In Eval " << std::endl;

        //assert(global::behav_dim == 2);

        std::vector<double> key(x.size(), 0);
        Eigen::VectorXd::Map(key.data(), key.size()) = x;

        unsigned ctrl_index = Params::archiveparams::archive.at(key).controller;

        // ./bin/behaviour_evolBO2D experiments/history_BO.argos experiments/fitness${INDIVIDUAL} --load ${DATA}/history/results1/gen_2000 -n ${INDIVIDUAL} -o experiments/nn${INDIVIDUAL}
        std::string sim_cmd = global::argossim_bin_name + " " +
                              global::argossim_config_name + " " +
                              global::results_path + "/fitness" + std::to_string(ctrl_index) + ".dat "
                              "--load " + global::archive_path + "/gen_" + std::to_string(global::gen_to_load) + " " +
                              "-n " + std::to_string(ctrl_index) + " " +
                              "-o " + global::results_path + "/nn" + std::to_string(ctrl_index) + ".dot " +
                              "-d " + global::results_path;

        if(system(sim_cmd.c_str())!=0)
        {
            std::cerr << "Error executing simulation " << std::endl << sim_cmd << std::endl;
            exit(-1);
        }

        std::cout << "Loading fitness" << ctrl_index << " file " << std::endl;
        std::ifstream monFlux((global::results_path + "/fitness" + std::to_string(ctrl_index)+ ".dat").c_str(), std::ios::out);
        double fitness;
        if (monFlux)
        {
            std::string line; unsigned line_count = 0;
            while (std::getline(monFlux, line))
            {
                std::istringstream iss(line);
                std::vector<double> numbers;
                double num;
                while (iss >> num)
                    numbers.push_back(num);

                if(numbers.size() > 1)
                    std::cerr << "Warning ... we were expecting a single number in the fitness file " << " and not " << numbers.size();

                fitness = numbers[0];
                if (fitness > global::original_max)
                    global::original_max = fitness;

                line_count++;
            }
            if(line_count > 1)
                std::cerr << "Warning ... we were expecting a single line in the fitness file " << " and not " << line_count;
        }
        else
        {
            std::cerr << "ERROR: Could not load the fitness" << ctrl_index << "file " << std::endl;
            exit(-1);
        }

        std::cout << "Fitness = " << fitness << std::endl;

        /*std::vector<double> ctrl = Params::archiveparams::archive.at(key).controller;
        hexapod_dart::HexapodDARTSimu<> simu(ctrl, global::global_robot->clone());
        simu.run(5);

        return tools::make_vector(simu.covered_distance());*/
        return tools::make_vector(fitness);
    }
};


std::map<std::vector<double>, Params::archiveparams::elem_archive, Params::archiveparams::classcomp> load_archive(std::string archive_name)
{

    std::map<std::vector<double>, Params::archiveparams::elem_archive, Params::archiveparams::classcomp> archive;

    /*size_t lastindex = archive_name.find_last_of(".");
    std::string extension = archive_name.substr(lastindex + 1);*/

    // Assuming order <ind index> <behav_descriptor> <fitness>
    std::cout << "Loading archive file "  << archive_name << std::endl;
    std::ifstream monFlux(archive_name.c_str());
    if (monFlux)
    {
        std::string line;
        while (std::getline(monFlux, line))
        {
            std::istringstream iss(line);
            std::vector<double> numbers;
            double num;
            while (iss >> num)
            {
                numbers.push_back(num);
            }

            if (numbers.size() < (global::behav_dim + 1 + 1))
                continue;

            int init_i = 0;
            if (numbers.size() > (global::behav_dim + 1 + 1))
                init_i = 1;

            Params::archiveparams::elem_archive elem;
            std::vector<double> candidate(global::behav_dim);
            for (size_t i = 0; i < (global::behav_dim + 1 + 1); i++)
            {
                double data = numbers[init_i + i];
                if (i==0)
                    elem.controller = (size_t) data;

                else if (i >=1 && i <= global::behav_dim)
                {
                    candidate[i-1] = data;
                    elem.behav_descriptor.push_back(data);
                }
                else if (i == (global::behav_dim+1))
                {
                    elem.fit = data;
                }
                else{
                    throw std::runtime_error("not possible value of i");
                }
            }
            archive[candidate] = elem;
        }
    }
    else
    {
        std::cerr << "ERROR: Could not load the archive " << global::archive_path+"/archive_"+std::to_string(global::gen_to_load)+".dat" << std::endl;
        exit(-1);
    }

    std::cout << archive.size() << " elements loaded" << std::endl;
    return archive;
}

void print_individual_to_network(std::vector<double> bd,
            std::map<std::vector<double>, Params::archiveparams::elem_archive, Params::archiveparams::classcomp> archive
            )
{

    /*size_t lastindex = archive_name.find_last_of(".");
    std::string extension = archive_name.substr(lastindex + 1);*/

    Params::archiveparams::elem_archive elem = archive[bd];// get the element in the archive
    // get the controller id
    size_t ctrl_index = elem.controller;
    std::string sim_cmd = global::argossim_bin_name + " " +
                              global::argossim_config_name + " " +
                              global::results_path + "/fitness" + std::to_string(ctrl_index) + ".dat "+
                              "--load " + global::archive_path + "/gen_" + std::to_string(global::gen_to_load) + " " +
                              "-n " + std::to_string(ctrl_index) + " " +
                              "-o " + global::results_path + "/nn" + std::to_string(ctrl_index) + ".dot " +
                              "-d " + global::results_path;

    if(system(sim_cmd.c_str())!=0)
    {
        std::cerr << "Error executing simulation " << std::endl << sim_cmd << std::endl;
        exit(-1);
    }
}



Params::archiveparams::archive_t Params::archiveparams::archive;
//BO_DECLARE_DYN_PARAM(int, Params::stop_maxiterations, iterations);

/* ite_swarms requires arguments in the following order:
 * a. -m MAP-ELITES path of MAP
 * b.    Generation to load
 * c. -e Binary file to execute the argos simulator <with path>
 * d.    ARGoS configuration file for c. <with path>
*/
int main(int argc, char** argv)
{
    std::vector<std::string> cmd_args;
    for (int i = 0; i < argc; i++)
        cmd_args.push_back(std::string(argv[i]));

    std::vector<std::string>::iterator map_it = std::find(cmd_args.begin(), cmd_args.end(), "-m");
    std::vector<std::string>::iterator eval_it = std::find(cmd_args.begin(), cmd_args.end(), "-e");

    if(map_it == cmd_args.end())
    {
        std::cerr << "Argument -m map_path generation_to_load is missing. Exiting ..." << std::endl;
        exit(-1);
    }

    // Reading MAP arguments
    if((map_it+2 > cmd_args.end()) || (map_it+1 == eval_it) || (map_it+2 == eval_it))
    {
        std::cerr << "Argument -m is to be followed by map_path and generation_to_load. Exiting ..." << std::endl;
        exit(-1);
    }
    else
    {
        global::archive_path = *(map_it+1);
        global::gen_to_load= atoi((*(map_it+2)).c_str());
        Params::archiveparams::archive = load_archive(global::archive_path+"/archive_"+std::to_string(global::gen_to_load)+".dat");
    }

    if(eval_it == cmd_args.end())
    {
        std::cerr << "Argument -e argos_simulator_binary argos_config_file is missing. Exiting ...";
        exit(-1);
    }

    // Reading argos simulator arguments
    if((eval_it+2 > cmd_args.end()) || (eval_it+1 == map_it) || (eval_it+2 == map_it))
    {
        std::cerr << "Argument -e is to be followed by argos_simulator_binary argos_config_file. Exiting ..." << std::endl;
        exit(-1);
    }
    else
    {
        global::argossim_bin_name = *(eval_it+1);
        global::argossim_config_name = *(eval_it+2);
    }

    typedef kernel::MaternFiveHalves<Params> Kernel_t;
    typedef opt::ExhaustiveSearchArchive<Params> InnerOpt_t;

    // note: MaxPredictedValue just stops immediately (seems like maximal predicted value is set to initial value)
    // ,PercentageMax<Params>
    typedef boost::fusion::vector<stop::MaxIterations<Params>> Stop_t;
    typedef mean::MeanArchive<Params> Mean_t;
    typedef boost::fusion::vector<stat::Samples<Params>, stat::BestObservations<Params>, stat::ConsoleSummary<Params>> Stat_t;

    typedef init::NoInit<Params> Init_t;
    typedef model::GP<Params, Kernel_t, Mean_t> GP_t;
    typedef acqui::UCB<Params, GP_t> Acqui_t;

    bayes_opt::BOptimizer<Params, modelfun<GP_t>, initfun<Init_t>, acquifun<Acqui_t>, acquiopt<InnerOpt_t>, statsfun<Stat_t>, stopcrit<Stop_t>> opt;
    global::results_path = opt.res_dir();

    opt.optimize(Eval());


    auto val = opt.best_observation();
    Eigen::VectorXd result = opt.best_sample().transpose();

    std::cout << val << " res  " << result.transpose() << std::endl;


    std::vector<double> bd(result.data(), result.data() + result.rows() * result.cols());


    // now look up the behaviour descriptor in the archive file 
    // and save to BOOST_SERIALISATION_NVP
    print_individual_to_network(bd,Params::archiveparams::archive);

    return 0;
}
