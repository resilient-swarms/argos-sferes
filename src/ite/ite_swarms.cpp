#include <iostream>
#include <sstream>
#include <string>

#include <limbo/limbo.hpp>

#include <exhaustive_search_archive.hpp>
#include <mean_archive.hpp>


using namespace limbo;

namespace global
{
    std::string argossim_bin_name;
    std::string argossim_config_name;
    std::string archive_path;
    unsigned gen_to_load;
    unsigned behav_dim; // number of dimensions of MAP
} // namespace global

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
        BO_PARAM(int, iterations, 10);
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
    BO_PARAM(size_t, dim_in, 2); //global::behav_dim
    BO_PARAM(size_t, dim_out, 1);

    // the function to be optimized
    Eigen::VectorXd operator()(const Eigen::VectorXd& x) const
    {
        std::cout << "In Eval " << std::endl;

        assert(global::behav_dim == 2);

        std::vector<double> key(x.size(), 0);
        Eigen::VectorXd::Map(key.data(), key.size()) = x;

        unsigned ctrl_index = Params::archiveparams::archive.at(key).controller;

        // ./bin/behaviour_evolBO2D experiments/history_BO.argos experiments/fitness${INDIVIDUAL} --load ${DATA}/history/results1/gen_2000 -n ${INDIVIDUAL} -o experiments/nn${INDIVIDUAL}
        std::string sim_cmd = global::argossim_bin_name + " " +
                              global::argossim_config_name + " " +
                              "experiments/fitness" + std::to_string(ctrl_index) + " "
                              "--load " + global::archive_path + "/gen_" + std::to_string(global::gen_to_load) + " " +
                              "-n " + std::to_string(ctrl_index) + " " +
                              "-o " + "experiments/nn" + std::to_string(ctrl_index);

        if(system(sim_cmd.c_str())!=0)
        {
            std::cerr << "Error executing simulation " << std::endl << sim_cmd << std::endl;
            exit(-1);
        }

        std::cout << "Loading fitness" << ctrl_index << "file " << std::endl;
        std::ifstream monFlux(("experiments/fitness" + std::to_string(ctrl_index)).c_str());
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

                if (i >=1 && i <= global::behav_dim)
                {
                    candidate[i] = data;
                    elem.behav_descriptor.push_back(data);
                }
                if (i == (global::behav_dim+1))
                {
                    elem.fit = data;
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

Params::archiveparams::archive_t Params::archiveparams::archive;
//BO_DECLARE_DYN_PARAM(int, Params::stop_maxiterations, iterations);

/* ite_swarms requires arguments in the following order:
 * a. -a MAP-ELITES path of MAP
 * b.    Number of dimensions of MAP
 * c.    Generation to load
 * d. -s Binary file to execute the argos simulator <with path>
 * e.    ARGoS configuration file for d. <with path>
 * f. -i Number of iterations (optional)
*/
int main(int argc, char** argv)
{
    std::vector<std::string> cmd_args;
    for (int i = 0; i < argc; i++)
        cmd_args.push_back(std::string(argv[i]));

    /*std::vector<std::string>::iterator legs_it = std::find(cmd_args.begin(), cmd_args.end(), "-l");
    std::vector<std::string>::iterator ctrl_it = std::find(cmd_args.begin(), cmd_args.end(), "-c");
    std::vector<std::string>::iterator n_it = std::find(cmd_args.begin(), cmd_args.end(), "-n");

    std::vector<int> brokenleg;
    if (legs_it != cmd_args.end()) {
        std::vector<std::string>::iterator end_it = (legs_it < ctrl_it) ? ctrl_it : cmd_args.end();
        end_it = (end_it < n_it || n_it < legs_it) ? end_it : n_it;

        for (std::vector<std::string>::iterator ii = legs_it + 1; ii != end_it; ii++) {
            brokenleg.push_back(atoi((*ii).c_str()));
        }
        if (brokenleg.size() >= 6) {
            std::cerr << "The robot should at least have one leg!" << std::endl;
            if (global::global_robot)
                global::global_robot.reset();
            return -1;
        }
    }
    global::brokenLegs = brokenleg;

    init_simu(std::string(std::getenv("RESIBOTS_DIR")) + "/share/hexapod_models/URDF/pexod.urdf", global::brokenLegs);

    if (ctrl_it != cmd_args.end()) {
        std::vector<std::string>::iterator end_it = ctrl_it + 37;

        std::vector<double> ctrl;
        for (std::vector<std::string>::iterator ii = ctrl_it + 1; ii != end_it; ii++) {
            ctrl.push_back(atof((*ii).c_str()));
        }
        if (ctrl.size() != 36) {
            std::cerr << "You have to provide 36 controller parameters!" << std::endl;
            if (global::global_robot)
                global::global_robot.reset();
            return -1;
        }
        lecture(ctrl);

        if (global::global_robot)
            global::global_robot.reset();
        return 1;
    }*/

    // you need a map archive with number of dimensions, as well a argos-sim binary and configuration files in order to run ite-swarms
    if (argc < 6)
    {
        std::cerr << "Please provide the path to the map archive along with the number of dimensions of the MAP and the generation to load, as well as "
                     "the argos-sim binary and configuration files" << std::endl;
        return -1;
    }

    global::archive_path = argv[1];
    global::behav_dim = atoi(argv[2]);
    global::gen_to_load= atoi(argv[3]);
    global::argossim_bin_name = argv[4];
    global::argossim_config_name = argv[5];

    //std::cerr << "Setting number of iterations if argc =  " << argc << " for argv[6] " << argv[6];

    Params::archiveparams::archive = load_archive(std::string(argv[1])+"/archive_"+std::to_string(global::gen_to_load)+".dat");

    /*std::cerr << "Setting number of iterations if argc =  " << argc;

    if (argc == 7)
    {
        std::cout << "Setting number of iterations to " << atoi(argv[6]);
        Params::stop_maxiterations::set_iterations(atoi(argv[5]));
    }
    else
        Params::stop_maxiterations::set_iterations(10);*/

    typedef kernel::MaternFiveHalves<Params> Kernel_t;
    typedef opt::ExhaustiveSearchArchive<Params> InnerOpt_t;
    typedef boost::fusion::vector<stop::MaxIterations<Params>> Stop_t;
    typedef mean::MeanArchive<Params> Mean_t;
    typedef boost::fusion::vector<stat::Samples<Params>, stat::BestObservations<Params>, stat::ConsoleSummary<Params>> Stat_t;

    typedef init::NoInit<Params> Init_t;
    typedef model::GP<Params, Kernel_t, Mean_t> GP_t;
    typedef acqui::UCB<Params, GP_t> Acqui_t;

    bayes_opt::BOptimizer<Params, modelfun<GP_t>, initfun<Init_t>, acquifun<Acqui_t>, acquiopt<InnerOpt_t>, statsfun<Stat_t>, stopcrit<Stop_t>> opt;
    opt.optimize(Eval());

    auto val = opt.best_observation();
    Eigen::VectorXd result = opt.best_sample().transpose();

    std::cout << val << " res  " << result.transpose() << std::endl;

    /*if (global::global_robot)
        global::global_robot.reset();*/

    return 0;
}
