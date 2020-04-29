#ifndef RANDOM_SEARCH_HPP_
#define RANDOM_SEARCH_HPP_

#include <src/ite/ite_swarms.hpp>
//modifies the stat-map to calculate averaged performance over all individuals

// #define MAP_WRITE_PARENTS

struct Proposal
{
    std::vector<std::vector<double>> keys_left;
    size_t current_index;
    std::string tag = "random_search";
    std::random_device rd; //Will be used to obtain a seed for the random number engine
    std::mt19937 gen;      //Standard mersenne_twister_engine seeded with rd()
    void init()
    {
        for (Params::archiveparams::archive_t::iterator iter = Params::archiveparams::archive.begin();
             iter != Params::archiveparams::archive.end(); ++iter)
        {
            std::vector<double> key = iter->first;
            keys_left.push_back(key);
        }
        gen = std::mt19937(rd());
    }
    /* generate proposed behaviour */
    std::vector<double> generate()
    {
        std::uniform_int_distribution<> dis(0, keys_left.size() - 1);
        current_index = dis(gen);
        std::vector<double> key = keys_left[current_index];
        return key;
    }
    /* update the proposal mechanism based on the recent sample */
    void update()
    {
        // remove index
        keys_left.erase(keys_left.begin() + current_index);
    }
};

template <typename Proposal_t>
class Baseline
{
public:
    Proposal_t proposal;
    Baseline()
    {
    }

    void run(std::ofstream &os)
    {
        proposal.init();
        float best_so_far = -std::numeric_limits<float>::infinity();
        while (proposal.keys_left.size() > 0)
        {
            // choose new sample
            std::vector<double> sample = proposal.generate();
            Params::archiveparams::elem_archive el = Params::archiveparams::archive.at(sample);
            // run new sample
            float fitness = perform_command(el.controller, global::current_config);
            if (fitness > best_so_far)
            {
                best_so_far = fitness;
            }
            // update
            proposal.update();

            // write the statistic to file
            for (size_t i = 0; i < sample.size(); ++i)
            {
                os << sample[i] << " ";
            }
            os << fitness << std::endl;
        }
    }
};

void run_baseline(const std::string &choice, const std::string &prefix)
{
    global::results_path =  prefix;
    std::string filename = prefix + "/" + choice;
    std::ofstream writer(filename.c_str());
    if (choice == "random")
    {
        Baseline<Proposal> baseline;
        baseline.run(writer);
    }
    else
    {
        throw std::runtime_error("not yet implemented");
    }
}

#endif