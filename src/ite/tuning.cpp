

#define TUNING

#include "src/ite/ite_swarms.hpp"

/******************************/
/* HYPER PARAM TUNING HERE */

// parameters to tune the experiments alpha and length values
// parameters to tune the experiments alpha and length values

//BO_DECLARE_DYN_PARAM(int, Params::stop_maxiterations, iterations);
BO_DECLARE_DYN_PARAM(double, Params::acqui_ucb, alpha);
BO_DECLARE_DYN_PARAM(double, Params::kernel_maternfivehalves, l);

// parameters to tune the experiments alpha and length values
struct HPParams
{
    struct bayes_opt_boptimizer : public defaults::bayes_opt_boptimizer
    {
    };

// depending on which internal optimizer we use, we need to import different parameters
#ifdef USE_NLOPT
    struct opt_nloptnograd : public defaults::opt_nloptnograd
    {
    };
#elif defined(USE_LIBCMAES)
    struct opt_cmaes : public defaults::opt_cmaes
    {
    };
#else
    struct opt_gridsearch : public defaults::opt_gridsearch
    {
    };
#endif

    // enable / disable the writing of the result files
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
    };

    // we use 10 random samples to initialize the algorithm
    struct init_randomsampling
    {
        BO_PARAM(int, samples, 250);
    };

    // we stop after 40 iterations
    struct stop_maxiterations
    {
        BO_PARAM(int, iterations, 0);
    };

    struct stop_maxpredictedvalue
    {
        BO_PARAM(double, ratio, 100.0f); //take something absurd
    };

    // we use the default parameters for acqui_ucb
    struct acqui_ucb : public defaults::acqui_ucb
    {
    };
};

// evaluate hyperparameter selection
// for ucb see http://www.resibots.eu/limbo/defaults.html#acqui-ucb
// for matern-5/2 kernel see http://www.resibots.eu/limbo/defaults.html#kernel-maternfivehalves
//                          https://github.com/resibots/limbo/blob/master/src/limbo/kernel/matern_five_halves.hpp

struct EvalHP
{
    BO_PARAM(size_t, dim_in, 3);  // 3 hyperparameters  (alpha for acquisition function and l and sigma_sq for matern kernel)
    BO_PARAM(size_t, dim_out, 2); // fitness and number of trials

    // the function to be optimized
    Eigen::VectorXd operator()(const Eigen::VectorXd &x) const
    {
        
        // be able to adjust alpha
        double opt_alpha = x[0];
        double opt_l = x[1];
        Params::acqui_ucb::set_alpha(opt_alpha);
        Params::kernel_maternfivehalves::set_l(opt_l);

        Opt_t opt;



        Eigen::VectorXd vec = Eigen::VectorXd::Zero(2); // tuning on logarithmic scale
        std::cout << "------------------------------------" << std::endl;
            std::cout << "Start evaluating alpha=" << opt_alpha << " length-scale="<<opt_l << std::endl;
            std::cout << "------------------------------------" << std::endl;
        for (size_t i = 0; i < global::argossim_config_name.size(); ++i)
        {
            std::cout << "------------------------------------" << std::endl;
            std::cout << "Do IT&E for config " << global::argossim_config_name[i] << std::endl;
            std::cout << "------------------------------------" << std::endl;
            global::current_config = global::argossim_config_name[i];
            global::results_path = opt.res_dir();
            global::num_trials = 0;
            opt.optimize(ControllerEval());
            auto val = opt.best_observation();
            Eigen::VectorXd result = opt.best_sample().transpose();

            double trials = max_evals - global::num_trials; // minimise the number of trials

            vec[0] += val[0];
            vec[1] += trials;
        }
        vec[0] /= (float)global::argossim_config_name.size();
        vec[1] /= (float)global::argossim_config_name.size();
        std::cout << "------------------------------------" << std::endl;
        std::cout << "evaluated one point for HP tuning" << std::endl;
        std::cout << "Point " << x << " evaluated" << std::endl;
        std::cout << "Avg Fitness="<< vec[0] << std::endl;
        std::cout << "Avg Number of trials="<< max_evals - vec[1] << std::endl;
        std::cout << "------------------------------------" << std::endl;
        std::cout << "------------------------------------" << std::endl;
        global::hyper_log << x[0] << "\t" << x[1] << "\t" << vec[0] << "\t" << vec[1] << std::endl;
        return vec;
    }
};

//Params::archiveparams::archive_t Params::archiveparams::archive;
// BO_DECLARE_DYN_PARAM(int, Params::stop_maxiterations, iterations);
// BO_DECLARE_DYN_PARAM(double, Params::acqui_ucb, alpha);
// BO_DECLARE_DYN_PARAM(double, Params::kernel_maternfivehalves, l);

//BO_DECLARE_DYN_PARAM(int, Params::stop_maxiterations, iterations);

/* ite_swarms requires arguments in the following order:
 * 0. -f outputfolder
 * a. -m MAP-ELITES path of MAP
 * b.    Generation to load
 * c. -e Binary file to execute the argos simulator <with path>
 * d.    ARGoS configuration file for c. <with path>  (variable length, corresponding to different configurations)

*/
//BO_DECLARE_DYN_PARAM(double, Params::acqui_ucb, alpha);
//BO_DECLARE_DYN_PARAM(Eigen::VectorXd, Params::kernel_maternfivehalves, h_params);

int main(int argc, char **argv)
{
    // parse arguments
    std::vector<std::string> cmd_args;
    for (int i = 0; i < argc; i++)
        cmd_args.push_back(std::string(argv[i]));

    std::vector<std::string>::iterator map_it = std::find(cmd_args.begin(), cmd_args.end(), "-m");
    std::vector<std::string>::iterator eval_it = std::find(cmd_args.begin(), cmd_args.end(), "-e");
    std::vector<std::string>::iterator folder_it = std::find(cmd_args.begin(), cmd_args.end(), "-f");

    if (map_it == cmd_args.end())
    {
        std::cerr << "Argument -m map_path generation_to_load is missing. Exiting ..." << std::endl;
        exit(-1);
    }

    // Reading MAP arguments
    if ((map_it + 2 > cmd_args.end()) || (map_it + 1 == eval_it) || (map_it + 2 == eval_it))
    {
        std::cerr << "Argument -m is to be followed by map_path and generation_to_load. Exiting ..." << std::endl;
        exit(-1);
    }
    else
    {
        global::archive_path = *(map_it + 1);
        global::gen_to_load = atoi((*(map_it + 2)).c_str());
        Params::archiveparams::archive = load_archive(global::archive_path + "/archive_" + std::to_string(global::gen_to_load) + ".dat");
    }

    if (eval_it == cmd_args.end())
    {
        std::cerr << "Argument -e argos_simulator_binary argos_config_file is missing. Exiting ...";
        exit(-1);
    }

    // Reading argos simulator arguments
    size_t num_configs = cmd_args.end() - (eval_it + 2);
    if ((eval_it + 2 > cmd_args.end()) || (eval_it + 1 == map_it) || (eval_it + 2 == map_it))
    {
        std::cerr << "Argument -e is to be followed by argos_simulator_binary argos_config_file. Exiting ..." << std::endl;
        exit(-1);
    }
    else
    {
        global::argossim_bin_name = *(eval_it + 1);
        for (size_t c = 0; c < num_configs; ++c)
        {
            global::argossim_config_name.push_back(*(eval_it + 2 + c));
        }
    }

    std::string newname;
    // Results directory
    if ((folder_it + 1 > cmd_args.end()) || (folder_it + 1 == map_it))
    {
        std::cerr << "Argument -f is to be followed by the outputfolder of the bayesian optimisation. Exiting ..." << std::endl;
        exit(-1);
    }
    else
    {
        newname = *(folder_it + 1);
    }

    global::num_trials = 0;

    bayes_opt::BOptimizer<HPParams> HPopt;
    global::hyper_results_path = HPopt.res_dir();
    
    HPopt.optimize(EvalHP());
    auto HPval = HPopt.best_observation();
    Eigen::VectorXd HPresult = HPopt.best_sample().transpose();

    std::cout << "max value: " << HPval << " with best sample:  " << HPresult.transpose() << std::endl;

    rename_folder(global::hyper_results_path, newname);
}