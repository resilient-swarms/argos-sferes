
#include "src/ite/baselines.hpp"

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
    std::vector<std::string>::iterator baseline_it = std::find(cmd_args.begin(), cmd_args.end(), "-b");

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
        size_t index = (map_it + 1) - cmd_args.begin();
        global::archive_path = argv[index];
        global::gen_to_load = atoi((*(map_it + 2)).c_str());
        Params::archiveparams::archive = load_archive(std::string(global::archive_path) + "/archive_" + std::to_string(global::gen_to_load) + ".dat");
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
        global::current_config = *(eval_it + 2);
    }

    char *newname;
    // Results directory
    if ((folder_it + 1 > cmd_args.end()) || (folder_it + 1 == map_it))
    {
        std::cerr << "Argument -f is to be followed by the outputfolder of the baseline optimiser. Exiting ..." << std::endl;
        exit(-1);
    }
    else
    {
        size_t index = (folder_it + 1) - cmd_args.begin();
        newname = argv[index];
    }

    // baseline
    std::string choice;
    if ((baseline_it + 1 > cmd_args.end()))
    {
        std::cerr << "Argument -b is to be followed by the type of baseline optimiser. Exiting ..." << std::endl;
        exit(-1);
    }
    else
    {
        choice = *(baseline_it + 1);
    }

    /* run baseline */
    run_baseline(choice, newname);
    return 0;
}
