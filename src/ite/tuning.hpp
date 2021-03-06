

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
        BO_PARAM(int, samples, 10);
    };

    // we stop after 40 iterations
    struct stop_maxiterations
    {
        BO_PARAM(int, iterations, 40);
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
    BO_PARAM(size_t, dim_in, 2);  // 2 hyperparameters  (alpha for acquisition function and l for matern kernel)
    BO_PARAM(size_t, dim_out, 1); // fitness and number of trials

    // the function to be optimized
    Eigen::VectorXd operator()(const Eigen::VectorXd &x) const
    {
        
        // be able to adjust alpha
        double opt_alpha = x[0];
        double opt_l = x[1];
        Params::acqui_ucb::set_alpha(opt_alpha);
        Params::kernel_maternfivehalves::set_l(opt_l);

        Opt_t opt;


        float sum_trials=0.0f;
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
            std::string str = opt.res_dir();
            str.copy(global::results_path,str.size()+1);
            global::num_trials = 0;
            opt.optimize(ControllerEval());
            auto val = opt.best_observation();
            Eigen::VectorXd result = opt.best_sample().transpose();

            //double trials = max_evals - global::num_trials; // minimise the number of trials
            std::vector<Eigen::VectorXd> observations = opt.observations();
            float performance_loss=0.0f;
            for (auto &obs : observations)
            {
                performance_loss += (val[0] - obs[0])/(float) max_evals;//
            }


            vec[0] += val[0] - 0.10*performance_loss;// performance
            

        }
        vec[0] /= (float)global::argossim_config_name.size();
        std::cout << "------------------------------------" << std::endl;
        std::cout << "evaluated one point for HP tuning" << std::endl;
        std::cout << "Point " << x << " evaluated" << std::endl;
        std::cout << "Avg Fitness="<< vec[0] << std::endl;
        std::cout << "Avg Number of trials="<< max_evals - vec[1] << std::endl;
        std::cout << "------------------------------------" << std::endl;
        std::cout << "------------------------------------" << std::endl;
        //global::hyper_log << x[0] << "\t" << x[1] << "\t" << vec[0] << std::endl;
        return vec;
    }
};


void run_tuning(const std::string &newname)
{
    global::num_trials = 0;

    bayes_opt::BOptimizer<HPParams> HPopt;
    global::hyper_results_path = HPopt.res_dir();

    HPopt.optimize(EvalHP());
    auto HPval = HPopt.best_observation();
    Eigen::VectorXd HPresult = HPopt.best_sample().transpose();

    std::cout << "max value: " << HPval << " with best sample:  " << HPresult.transpose() << std::endl;

    rename_folder(global::hyper_results_path, newname);
}
