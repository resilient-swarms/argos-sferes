#include <iostream>
#include <sstream>
#include <string>

#include <limbo/limbo.hpp>

#include <exhaustive_search_archive.hpp>
#include <mean_archive.hpp>
#include <stdio.h>

using namespace limbo;


const size_t max_trials = 10;

namespace global
{
std::string results_path;
std::string argossim_bin_name;
std::vector<std::string> argossim_config_name;
std::string archive_path;
size_t num_trials;
float original_max = -std::numeric_limits<float>::infinity();
unsigned gen_to_load;
unsigned behav_dim = BEHAV_DIM; // number of dimensions of MAP
} // namespace global

struct Params
{

    struct bayes_opt_boptimizer : public defaults::bayes_opt_boptimizer
    {
    };

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

    struct stop_maxiterations : public defaults::stop_maxiterations
    {
        //BO_DYN_PARAM(int, iterations);
        BO_PARAM(int, iterations, max_trials);
    };

    struct stop_maxpredictedvalue : public defaults::stop_maxpredictedvalue
    {
        //BO_DYN_PARAM(int, iterations);
        BO_PARAM(double, ratio, 0.90);
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
            bool operator()(const std::vector<double> &lhs, const std::vector<double> &rhs) const
            {
                assert(lhs.size() == global::behav_dim && rhs.size() == global::behav_dim);
                size_t i = 0;
                while (i < (global::behav_dim - 1) && std::round(lhs[i] * 100.0) / 100.0 == std::round(rhs[i] * 100.0) / 100.0) //lhs[i]==rhs[i])
                    i++;
                return std::round(lhs[i] * 100.0) / 100.0 < std::round(rhs[i] * 100.0) / 100.0; //lhs[i]<rhs[i];
            }
        };
        typedef std::map<std::vector<double>, elem_archive, classcomp> archive_t;
        static std::map<std::vector<double>, elem_archive, classcomp> archive;
    };
};

Params::archiveparams::archive_t load_archive(std::string archive_name);

double get_fitness(size_t ctrl_index)
{
    std::ifstream monFlux((global::results_path + "/fitness" + std::to_string(ctrl_index) + ".dat").c_str(), std::ios::out);
    std::cout << "Loading fitness" << ctrl_index << " file " << std::endl;
    double fitness;
    if (monFlux)
    {
        std::string line;
        unsigned line_count = 0;
        while (std::getline(monFlux, line))
        {
            std::istringstream iss(line);
            std::vector<double> numbers;
            double num;
            while (iss >> num)
                numbers.push_back(num);

            if (numbers.size() > 1)
                std::cerr << "Warning ... we were expecting a single number in the fitness file "
                          << " and not " << numbers.size();

            fitness = numbers[0];
            if (fitness > global::original_max)
                global::original_max = fitness;

            line_count++;
        }
        if (line_count > 1)
            std::cerr << "Warning ... we were expecting a single line in the fitness file "
                      << " and not " << line_count;
    }
    else
    {
        std::cerr << "ERROR: Could not load the fitness" << ctrl_index << "file " << std::endl;
        exit(-1);
    }
    std::cout << "Fitness = " << fitness << std::endl;
    return fitness;
}

double perform_command(size_t ctrl_index, size_t config_index)
{

    std::string sim_cmd = global::argossim_bin_name + " " +
                          global::argossim_config_name[config_index] + " " +
                          global::results_path + "/fitness" + std::to_string(ctrl_index) + ".dat "
                                                                                           "--load " +
                          global::archive_path + "/gen_" + std::to_string(global::gen_to_load) + " " +
                          "-n " + std::to_string(ctrl_index) + " " +
                          "-o " + global::results_path + "/nn" + std::to_string(ctrl_index) + ".dot " +
                          "-d " + global::results_path;
    if (system(sim_cmd.c_str()) != 0)
    {
        std::cerr << "Error executing simulation " << std::endl
                  << sim_cmd << std::endl;
        exit(-1);
    }

    return get_fitness(ctrl_index);
}

struct ControllerEval
{
    BO_PARAM(size_t, dim_in, BEHAV_DIM); //global::behav_dim
    BO_PARAM(size_t, dim_out, 1);

    // the function to be optimized
    Eigen::VectorXd operator()(const Eigen::VectorXd &x) const
    {
        std::cout << "In Eval " << std::endl;
        ++global::num_trials;
        //assert(global::behav_dim == 2);

        std::vector<double> key(x.size(), 0);
        Eigen::VectorXd::Map(key.data(), key.size()) = x;

        unsigned ctrl_index = Params::archiveparams::archive.at(key).controller;

        // ./bin/behaviour_evolBO2D experiments/history_BO.argos experiments/fitness${INDIVIDUAL} --load ${DATA}/history/results1/gen_2000 -n ${INDIVIDUAL} -o experiments/nn${INDIVIDUAL}

        float sum_fitness = 0.0f;
        for (size_t i = 0; i < global::argossim_config_name.size(); ++i)
        {
            sum_fitness += perform_command(ctrl_index, i);
        }
        sum_fitness /= (float)global::argossim_config_name.size();

        std::cout << "fit was : " << Params::archiveparams::archive[key].fit << std::endl;
        Params::archiveparams::archive[key].fit = sum_fitness; //update the value
        std::cout << "fit is now : " << Params::archiveparams::archive[key].fit << std::endl;
        /*std::vector<double> ctrl = Params::archiveparams::archive.at(key).controller;
        hexapod_dart::HexapodDARTSimu<> simu(ctrl, global::global_robot->clone());
        simu.run(5);

        return tools::make_vector(simu.covered_distance());*/
        return tools::make_vector(sum_fitness);
    }
};

std::map<std::vector<double>, Params::archiveparams::elem_archive, Params::archiveparams::classcomp> load_archive(std::string archive_name)
{

    std::map<std::vector<double>, Params::archiveparams::elem_archive, Params::archiveparams::classcomp> archive;

    /*size_t lastindex = archive_name.find_last_of(".");
    std::string extension = archive_name.substr(lastindex + 1);*/

    // Assuming order <ind index> <behav_descriptor> <fitness>
    std::cout << "Loading archive file " << archive_name << std::endl;
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
                if (i == 0)
                    elem.controller = (size_t)data;

                else if (i >= 1 && i <= global::behav_dim)
                {
                    candidate[i - 1] = data;
                    elem.behav_descriptor.push_back(data);
                }
                else if (i == (global::behav_dim + 1))
                {
                    elem.fit = data;
                }
                else
                {
                    throw std::runtime_error("not possible value of i");
                }
            }
            archive[candidate] = elem;
        }
    }
    else
    {
        std::cerr << "ERROR: Could not load the archive " << global::archive_path + "/archive_" + std::to_string(global::gen_to_load) + ".dat" << std::endl;
        exit(-1);
    }

    std::cout << archive.size() << " elements loaded" << std::endl;
    return archive;
}

void print_individual_to_network(std::vector<double> bd,
                                 std::map<std::vector<double>, Params::archiveparams::elem_archive, Params::archiveparams::classcomp> archive)
{

    /*size_t lastindex = archive_name.find_last_of(".");
    std::string extension = archive_name.substr(lastindex + 1);*/

    Params::archiveparams::elem_archive elem = archive[bd]; // get the element in the archive
    // get the controller id
    size_t ctrl_index = elem.controller;
    std::cout << "will now evaluate individual=" + std::to_string(ctrl_index) << std::endl;
    std::string sim_cmd = global::argossim_bin_name + " " +
                          global::argossim_config_name[0] + " " +
                          global::results_path + "/fitness" + std::to_string(ctrl_index) + ".dat " +
                          "--load " + global::archive_path + "/gen_" + std::to_string(global::gen_to_load) + " " +
                          "-n " + std::to_string(ctrl_index) + " " +
                          "-o " + global::results_path + "/nn" + std::to_string(ctrl_index) + ".dot " +
                          "-d " + global::results_path;

    if (system(sim_cmd.c_str()) != 0)
    {
        std::cerr << "Error executing simulation " << std::endl
                  << sim_cmd << std::endl;
        exit(-1);
    }
}

void rename_folder(std::string oldname, std::string newname)
{
    int result;
    result = rename(oldname.c_str(), newname.c_str());
    if (result == 0)
        puts("File successfully renamed");
    else
        perror("Error renaming file");
}

Params::archiveparams::archive_t Params::archiveparams::archive;



typedef kernel::MaternFiveHalves<Params> Kernel_t;
typedef opt::ExhaustiveSearchArchive<Params> InnerOpt_t;

// note: MaxPredictedValue just stops immediately (seems like maximal predicted value is set to initial value)
// ,PercentageMax<Params>
//typedef boost::fusion::vector<stop_maxiterations> Stop_t;
typedef mean::MeanArchive<Params> Mean_t;
typedef boost::fusion::vector<stat::Samples<Params>, stat::BestObservations<Params>, stat::ConsoleSummary<Params>> Stat_t;

typedef init::NoInit<Params> Init_t;
typedef model::GP<Params, Kernel_t, Mean_t> GP_t;
typedef acqui::UCB<Params, GP_t> Acqui_t;
//typedef limbo::stop::MaxPredictedValue<Params> Stop_t;// seems to not be updated at all --> modify default values
