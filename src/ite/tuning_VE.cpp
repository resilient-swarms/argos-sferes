

#include <src/ite/tuning.hpp>

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
        std::string str = *(map_it + 1);
        str.copy(global::archive_path, str.size() + 1 );
        global::gen_to_load = atoi((*(map_it + 2)).c_str());
        load_archive(std::string(global::archive_path) + "/archive_" + std::to_string(global::gen_to_load) + ".dat",
                                                        std::string(global::archive_path) + "/virtual_energy_exp/fitness");
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

    run_tuning(newname);
}