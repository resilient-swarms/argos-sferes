#include <iostream>
#include <sstream>
#include <string>

#include <limbo/limbo.hpp>

#include <src/ite/exhaustive_search_archive.hpp>
#include <src/ite/mean_archive.hpp>
#include <stdio.h>
#include <map>

using namespace limbo;

const size_t max_evals = 100;

#ifdef REAL_EXP
size_t num_trials = 3;
#else
size_t num_trials = 1; //trials done internally
#endif

namespace global
{
#if HETEROGENEOUS
    const size_t num_ID_features = 6;
    std::vector<double> normalID;
#endif
    std::string results_path;
    std::string hyper_results_path;
    std::ofstream hyper_log("hyperlog.txt");
    std::string argossim_bin_name;
    std::vector<std::string> argossim_config_name;
    std::string current_config;
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

        BO_PARAM(double, noise, 0.0); // 0.001 is used in IT&E
    };

    // // using a default 90% above all other predicted gait performances in the map
    // struct stop_maxpredictedvalue : public defaults::stop_maxpredictedvalue
    // {
    //     BO_PARAM(double, ratio, 0.99);
    // };

    struct stop_maxiterations
    {
        BO_PARAM(int, iterations, max_evals);
    };
#ifdef TUNING
    struct kernel_maternfivehalves : public defaults::kernel_maternfivehalves
    {
        BO_DYN_PARAM(double, l);
    };

    struct acqui_ucb : public defaults::acqui_ucb
    {
        BO_DYN_PARAM(double, alpha);
    };

#else

    // take tuned parameters 0.903197	0.274235
    struct kernel_maternfivehalves : public defaults::kernel_maternfivehalves
    {
#ifdef VE
        BO_PARAM(double, l, 0.0591898); // smoothness of the function;
#else
        BO_PARAM(double, l, 0.121697); // smoothness of the function;
#endif
        // 0.4 is used in IT&E; but here this affects all the behaviours it seems
        //1.5 is a setting used scikit learn https://scikit-learn.org/stable/modules/generated/sklearn.gaussian_process.kernels.Matern.html
    };

    struct acqui_ucb : public defaults::acqui_ucb
    {
#ifdef VE
        BO_PARAM(double, alpha, 0.407532);
#else
        BO_PARAM(double, alpha, 0.926734);
#endif
    };

#endif

    struct archiveparams
    {
        struct elem_archive
        {
            std::vector<double> behav_descriptor; // the first entry of elem_archive should be the behaviour descriptor (see ln 19 in exhaustive_search_archive.hpp)
            float fit;
            unsigned controller;
            bool checked = false;
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

Params::archiveparams::archive_t load_archive(std::string archive_name, std::string VE_file, bool uniform);

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
#ifdef VE
            fitness = numbers[1]; // only number usually; otherwise use first
#else
            fitness = numbers[0];
#endif
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

double perform_command(size_t ctrl_index, std::string config_file)
{
    std::cout << "will evaluate config : " << std::endl;
    std::cout << config_file << std::endl;
    std::string sim_cmd = global::argossim_bin_name + " " +
                          config_file + " " +
                          global::results_path + "/fitness" + std::to_string(ctrl_index) + ".dat " +
                          "--load " +
                          global::archive_path + "/gen_" + std::to_string(global::gen_to_load) + " " +
                          "-n " + std::to_string(ctrl_index) + " " +
                          "-o " + global::results_path + "/nn" + std::to_string(ctrl_index) + ".dot " +
                          "-d " + global::results_path + " >> BOlog.txt";
    if (system(sim_cmd.c_str()) != 0)
    {
        std::cerr << "Error executing simulation " << std::endl
                  << sim_cmd << std::endl;
        std::cout << "please check whether the fitness has been written properly, and press a key";
        char key;
        std::cin >> key;
    }

    return get_fitness(ctrl_index);
}
struct RealEval
{
    BO_PARAM(size_t, dim_in, BEHAV_DIM); //global::behav_dim
    BO_PARAM(size_t, dim_out, 1);

    // the function to be optimized
    Eigen::VectorXd operator()(const Eigen::VectorXd &x) const
    {
        std::cout << "In Eval " << std::endl;

        //assert(global::behav_dim == 2);

        std::vector<double> key(x.size(), 0);
        Eigen::VectorXd::Map(key.data(), key.size()) = x;

        unsigned ctrl_index = Params::archiveparams::archive.at(key).controller;

        // ./bin/behaviour_evolBO2D experiments/history_BO.argos experiments/fitness${INDIVIDUAL} --load ${DATA}/history/results1/gen_2000 -n ${INDIVIDUAL} -o experiments/nn${INDIVIDUAL}
        char ready;
        double sum = 0.0;
        double fitness;
        std::vector<double> numbers;
        for (size_t trial = 0; trial < num_trials; ++trial)
        {
            std::string fitfile = global::results_path + "/fitness" + std::to_string(ctrl_index) + "t" + std::to_string(trial) + ".dat";
            std::string sim_cmd = global::argossim_bin_name + " " +
                                  global::argossim_config_name[0] + " " +
                                  fitfile + " --load " +
                                  global::archive_path + "/gen_" + std::to_string(global::gen_to_load) + " " +
                                  "-n " + std::to_string(ctrl_index) + " " +
                                  "-o " + global::results_path + "/nn" + std::to_string(ctrl_index) + ".dot " +
                                  "-d " + global::results_path;
            std::cout << "Will execute individual " << std::to_string(ctrl_index) << std::endl;
            std::cout << "Please run the following command manually " << std::endl;
            std::cout << sim_cmd << std::endl;
            std::cout << "After that press any key to continue" << std::endl;
            std::cin >> ready;

            std::ifstream infile(fitfile.c_str(), std::ios::out);
            int i = 0;
            std::string line;
            numbers.clear();
            while (std::getline(infile, line))
            {
                float value;
                std::stringstream ss(line);

                while (ss >> value)
                {
                    numbers.push_back(value);
                }
                ++i;
            }
            if (i == 0)
            {
                std::cout << "No line" << std::endl;
            }
            if (i > 1)
            {
                std::cout << "warning : more than one line" << std::endl;
            }
            std::cout << "Fitness = " << numbers[0] << std::endl;
            sum += numbers[0];
        }

        fitness = sum /= (float)num_trials;
        if (fitness > global::original_max)
            global::original_max = fitness;

        std::cout << "Average fitness = " << fitness << std::endl;

        /*std::vector<double> ctrl = Params::archiveparams::archive.at(key).controller;
        hexapod_dart::HexapodDARTSimu<> simu(ctrl, global::global_robot->clone());
        simu.run(5);

        return tools::make_vector(simu.covered_distance());*/
        return tools::make_vector(fitness);
    }
};
struct ControllerEval
{
#if HETEROGENEOUS & !PRINT_NETWORK
    BO_PARAM(size_t, dim_in, BEHAV_DIM + global::num_ID_features); //global::behav_dim
#else
    BO_PARAM(size_t, dim_in, BEHAV_DIM); //global::behav_dim
#endif
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

        float fitness = perform_command(ctrl_index, global::current_config);

        std::cout << "fit was : " << Params::archiveparams::archive[key].fit << std::endl;
        //Params::archiveparams::archive[key].fit = sum_fitness;//only way our Mean function is updated
        std::cout << "fit is now : " << fitness << std::endl;
        /*std::vector<double> ctrl = Params::archiveparams::archive.at(key).controller;
        hexapod_dart::HexapodDARTSimu<> simu(ctrl, global::global_robot->clone());
        simu.run(5);

        return tools::make_vector(simu.covered_distance());*/
        return tools::make_vector(fitness);
    }
};

double get_VE(size_t line_no, std::string VE_file)
{
    std::string line;
    size_t temp_line_no = 0;
    std::ifstream monFlux(VE_file.c_str());
    while (std::getline(monFlux, line))
    {
        std::istringstream iss(line);
        std::vector<double> numbers;
        if (temp_line_no == line_no)
        {

            double num;
            while (iss >> num)
            {
                numbers.push_back(num);
            }

            if (numbers.size() != 3)
            {
                throw std::runtime_error("lines in VE file should have three numbers");
            }

            return numbers[1];
        }

        ++temp_line_no;
    }
    throw std::runtime_error("line_no not reached !");
    return 0.0f;
}
#if HETEROGENEOUS & !PRINT_NETWORK
Params::archiveparams::archive_t load_archive(std::string archive_name, std::vector<double> normal_ID = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5}, std::string VE_file = "", bool uniform = false)
{

    global::normalID = normal_ID;
#else
Params::archiveparams::archive_t load_archive(std::string archive_name, std::string VE_file = "", bool uniform = false)
{
#endif

    Params::archiveparams::archive_t archive;

    bool do_VE = false;
    if (VE_file != "")
    {
        do_VE = true;
    }

    float avg = 0.0f;
    /*size_t lastindex = archive_name.find_last_of(".");
    std::string extension = archive_name.substr(lastindex + 1);*/

    // Assuming order <ind index> <behav_descriptor> <fitness>
    std::cout << "Loading archive file " << archive_name << std::endl;
    std::ifstream monFlux(archive_name.c_str());

    if (monFlux)
    {
        size_t line_no = 0;
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
            {
                throw std::runtime_error("lower than expected dimension");
            }
            else if (numbers.size() > (global::behav_dim + 3))
            {
                throw std::runtime_error("higher than expected dimension");
            }

            int init_i = 0;
            if (numbers.size() > (global::behav_dim + 1 + 1)) // additional index added at start (also ignore)
                init_i = 1;

            Params::archiveparams::elem_archive elem;
#if HETEROGENEOUS & !PRINT_NETWORK
            std::vector<double> candidate(global::behav_dim + global::num_ID_features);
#else
            std::vector<double> candidate(global::behav_dim);
#endif
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

                    if (do_VE)
                    {
                        std::cout << "Line no " << line_no << std::endl;
                        if (uniform)
                        {
                            // do nothing yet except increment the average
                            avg += get_VE(line_no, VE_file);
                        }
                        else
                        {
                            elem.fit = get_VE(line_no, VE_file);
                        }
                    }
                    else
                    {
                        if (uniform)
                        {
                            // do nothing yet except increment the average
                            avg += data;
                        }
                        else
                        {
                            elem.fit = data;
                        }
                    }
                }
                else
                {
                    throw std::runtime_error("not possible value of i");
                }
            }
#if HETEROGENEOUS & !PRINT_NETWORK
            for (size_t c = 0; c < normal_ID.size(); ++c)
            {
                candidate[global::behav_dim + c] = normal_ID[c];
            }
#endif
            archive[candidate] = elem;

            ++line_no;
        }
    }
    else
    {
        std::cerr << "ERROR: Could not load the archive " << archive_name << std::endl;
        exit(-1);
    }

    std::cout << archive.size() << " elements loaded" << std::endl;
    if (uniform)
    {
        avg /= (float)archive.size();
        std::cout << "uniform prior with avg = " << avg << std::endl;
        for (Params::archiveparams::archive_t::iterator iter = archive.begin();
             iter != archive.end(); ++iter)
        {
            iter->second.fit = avg; // set to the average across the archive
        }
    }
    return archive;
}

std::string print_individual_to_network(std::vector<double> bd,
                                        Params::archiveparams::archive_t &archive)
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
    std::cout << "Will execute command " << sim_cmd << std::endl;
    if (system(sim_cmd.c_str()) != 0)
    {
        std::cerr << "Error executing simulation " << std::endl
                  << sim_cmd << std::endl;
        exit(-1);
    }
    return std::to_string(ctrl_index);
}

void rename_folder(std::string oldname, std::string newname)
{

    std::cout << "renaming: " << oldname << " " << newname << std::endl;
    std::string mv_cmd = "mv " + oldname + " " + newname;
    if (system(mv_cmd.c_str()) != 0)
    {
        std::cerr << "Error executing simulation " << std::endl
                  << mv_cmd << std::endl;
        exit(-1);
    }
    // int result;
    //ult = rename(oldname.c_str(), newname.c_str());
    //if (result == 0)
    //    puts("File successfully renamed");
    //else
    //    perror("Error renaming file");
}

Params::archiveparams::archive_t Params::archiveparams::archive;

#ifdef HETEROGENEOUS
typedef kernel::MaternFiveHalves<Params> Kernel_t;
typedef opt::ExhaustiveSearchArchive<Params> InnerOpt_t;
//typedef boost::fusion::vector<stop::MaxPredictedValue<Params>> Stop_t;
typedef mean::MeanArchive<Params> Mean_t;
// here, GPArchive, a custom module, writes the maps after each iteration
//    typedef boost::fusion::vector<stat::Samples<Params>, stat::BestObservations<Params>,
//            stat::ConsoleSummary<Params>, stat::AggregatedObservations<Params>, stat::BestAggregatedObservations<Params>,
//            stat::Observations<Params>, stat::BestSamples<Params>, stat::GPArchive<Params>> Stat_t;

// without the gparchive stats module in case you have not installed it.
typedef boost::fusion::vector<>
    Stat_t;

typedef init::NoInit<Params> Init_t;
typedef model::GP<Params, Kernel_t, Mean_t> GP_t;
typedef acqui::UCB<Params, GP_t> Acqui_t;
typedef bayes_opt::BOptimizerAsync<Params, modelfun<GP_t>, initfun<Init_t>, acquifun<Acqui_t>, acquiopt<InnerOpt_t>, statsfun<Stat_t>> Opt_t;

#else
typedef kernel::MaternFiveHalves<Params> Kernel_t;
typedef opt::ExhaustiveSearchArchive<Params> InnerOpt_t;
//typedef boost::fusion::vector<stop::MaxPredictedValue<Params>> Stop_t;
typedef mean::MeanArchive<Params> Mean_t;
// here, GPArchive, a custom module, writes the maps after each iteration
//    typedef boost::fusion::vector<stat::Samples<Params>, stat::BestObservations<Params>,
//            stat::ConsoleSummary<Params>, stat::AggregatedObservations<Params>, stat::BestAggregatedObservations<Params>,
//            stat::Observations<Params>, stat::BestSamples<Params>, stat::GPArchive<Params>> Stat_t;

// without the gparchive stats module in case you have not installed it.
typedef boost::fusion::vector<stat::Samples<Params>, stat::BestObservations<Params>,
                              stat::ConsoleSummary<Params>, stat::AggregatedObservations<Params>, stat::BestAggregatedObservations<Params>,
                              stat::Observations<Params>, stat::BestSamples<Params>>
    Stat_t;

typedef init::NoInit<Params> Init_t;
typedef model::GP<Params, Kernel_t, Mean_t> GP_t;
typedef acqui::UCB<Params, GP_t> Acqui_t;
typedef bayes_opt::BOptimizer<Params, modelfun<GP_t>, initfun<Init_t>, acquifun<Acqui_t>, acquiopt<InnerOpt_t>, statsfun<Stat_t>> Opt_t;

void run_ite(const std::string &newname)
{
    Opt_t opt;
    global::results_path = opt.res_dir();
    global::current_config = global::argossim_config_name[0];
#ifdef REAL_EXP
    opt.optimize(RealEval());
#else
    opt.optimize(ControllerEval());
#endif

    auto val = opt.best_observation();
    Eigen::VectorXd result = opt.best_sample().transpose();

    std::cout << val << " res  " << result.transpose() << std::endl;

    std::vector<double> bd(result.data(), result.data() + result.rows() * result.cols());

    // now look up the behaviour descriptor in the archive file
    // and save to BOOST_SERIALISATION_NVP
    print_individual_to_network(bd, Params::archiveparams::archive);
    rename_folder(global::results_path, newname);
}
#endif