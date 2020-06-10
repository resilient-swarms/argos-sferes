
#ifndef ITE_GLOBAL_HPP
#define ITE_GLOBAL_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <vector>


namespace global
{
#if HETEROGENEOUS
    std::vector<std::string> best_stats_file;
#if RECORD_FIT
    const size_t num_ID_features = 0;
#else
    const size_t num_ID_features = 6;
#endif
    std::vector<double> normalID;
#endif
    char* results_path;
    std::string hyper_results_path;
    std::ofstream hyper_log("hyperlog.txt");
    std::string argossim_bin_name;
    std::vector<std::string> argossim_config_name;
    std::string current_config;
    char* archive_path;
    size_t num_trials;
    float original_max = -std::numeric_limits<float>::infinity();
    unsigned gen_to_load;
    const unsigned behav_dim = BEHAV_DIM; // number of dimensions of MAP
} // namespace global


#endif