
#include "src/ite/ite_swarms.hpp"

//BO_DECLARE_DYN_PARAM(int, Params::stop_maxiterations, iterations);

/* ite_swarms requires arguments in the following order:
 * 0. -f outputfolder
 * a. -m MAP-ELITES path of MAP
 * b.    Generation to load
 * c. -e Binary file to execute the argos simulator <with path>
 * d.    ARGoS configuration file for c. <with path>

*/
int main(int argc, char **argv)
{
    std::vector<std::string> cmd_args;
    for (int i = 0; i < argc; i++)
        cmd_args.push_back(std::string(argv[i]));

    std::vector<std::string>::iterator map_it = std::find(cmd_args.begin(), cmd_args.end(), "-m");
    std::vector<std::string>::iterator eval_it = std::find(cmd_args.begin(), cmd_args.end(), "-e");
    std::vector<std::string>::iterator folder_it = std::find(cmd_args.begin(), cmd_args.end(), "-f");
    std::vector<std::string>::iterator hyper_it = std::find(cmd_args.begin(), cmd_args.end(), "-h");

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
            global::argossim_config_name.push_back(*(eval_it + 2));
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

    bayes_opt::BOptimizer<Params, modelfun<GP_t>, initfun<Init_t>, acquifun<Acqui_t>, acquiopt<InnerOpt_t>, statsfun<Stat_t>> opt;
    global::results_path = opt.res_dir();

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
    return 0;
}
