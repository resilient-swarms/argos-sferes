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
    Proposal()
    {
        for (Params::archiveparams::archive_t::iterator iter = Params::archiveparams::archive.begin();
             iter != Params::archiveparams::archive.end(); ++iter)
        {
            std::vector<double> key = iter->first;
            keys_left.push_back(key);
        }
    }
    /* generate proposed behaviour */
    std::vector<double> key generate()
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
        ids_left.erase(ids_left.begin() + current_index);
    }
};

template <typename Proposal_t>
class Baseline
{
public:
    Proposal_t proposal;
    Baseline(Proposal_t &prop) : proposal(prop)
    {
    }

    void run(std::ofstream& os)
    {
        float best_so_far = -std::numeric_limits<float>::infinity();
        while (ids_left.size() > 0)
        {
            // choose new sample
            std::vector<double> sample = proposal.generate();
            size_t ctrl_index = Params::archiveparams::archive.at(key);
            // run new sample
            float fitness = perform_command(ctrl_index, global::current_config);
            if (fitness > best_so_far)
            {
                best_so_far = fitness;
            }
            // update
            proposal.update();

            // write the statistic to file
            for (size_t i 0; i < sample.size(); ++i)
            {
                os << sample[i] << " ";
            }
            os << fitness << std::endl;
            
        }
    }
};

template <typename Proposal_t>
void construct_and_run_baseline(const std::string &choice)
{
    if (choice == "random")
    {
        Proposal random_proposal = Proposal();
        Baseline<Proposal> baseline = Baseline<Proposal>(random_proposal);
        baseline.run();
    }
    else
    {
        throw std::runtime_error("not yet implemented");
    }
}

#endif