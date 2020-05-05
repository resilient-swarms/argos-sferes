#ifndef RANDOM_SEARCH_HPP_
#define RANDOM_SEARCH_HPP_

#include <Eigen/Eigen>
#include <src/ite/ite_swarms.hpp>
//modifies the stat-map to calculate averaged performance over all individuals

// #define MAP_WRITE_PARENTS
#define GRADIENT_LOG
struct Proposal
{
    double current_fitness;
    std::vector<std::vector<double>> keys_left;
    size_t current_index;
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

struct GradientAscent
{
    std::random_device rd; //Will be used to obtain a seed for the random number engine
    std::mt19937 gen;      //Standard mersenne_twister_engine seeded with rd()
    std::vector<std::vector<double>> keys_left;
    Params::archiveparams::archive_t keys_sampled;
    Eigen::Matrix<double, BEHAV_DIM, 1> current_sample, previous_sample, delta;
    std::vector<double> current_key;
    size_t current_index;
    double current_fitness, previous_fitness; // current fitness
    double max_fitness;                       //max fitness used for normalisation
    double alpha = 0.05;                      // learning rate
    //double step = 0.0625;                     // minmal step between two behaviours
    size_t iteration = 0;
#ifdef GRADIENT_LOG
    std::ofstream gradlog;
#endif
    void init()
    {
        max_fitness = -std::numeric_limits<double>::infinity();
        for (Params::archiveparams::archive_t::iterator iter = Params::archiveparams::archive.begin();
             iter != Params::archiveparams::archive.end(); ++iter)
        {
            std::vector<double> key = iter->first;
            keys_left.push_back(key);
            double fit = iter->second.fit;
            if (fit > max_fitness)
            {
                max_fitness = fit;
            }
        }
        gen = std::mt19937(rd());
#ifdef GRADIENT_LOG
        gradlog = std::ofstream("/home/david/argos-sferes/gradient_log.txt", std::ios::app);
#endif
    }
    void find_closest_match()
    {
        double mindist = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < keys_left.size(); ++i)
        {
            std::vector<double> key = keys_left[i];

            double sum_sq = 0.0;
            for (size_t i = 0; i < BEHAV_DIM; ++i)
            {
                sum_sq += (key[i] - current_sample[i]) * (key[i] - current_sample[i]);
            }
            if (sum_sq < mindist)
            {
                mindist = sum_sq;
                current_key = key;
                current_index = i;
            }
        }
        // finally, set the current sample to the current key
        set_sample(current_key);
    }
    Params::archiveparams::archive_t::iterator sample_closest()
    {
        double mindist = std::numeric_limits<double>::infinity();
        Params::archiveparams::archive_t::iterator closest;
        for (Params::archiveparams::archive_t::iterator iter = keys_sampled.begin();
             iter != keys_sampled.end(); ++iter)
        {
            std::vector<double> key = iter->first;

            double sum_sq = 0.0;
            for (size_t i = 0; i < BEHAV_DIM; ++i)
            {
                sum_sq += (key[i] - current_sample[i]) * (key[i] - current_sample[i]);
            }
            if (sum_sq < mindist)
            {
                mindist = sum_sq;
                closest = iter;
            }
        }
        return closest;
    }
    void set_sample(const std::vector<double> &current_key)
    {
        // finally, set the current sample to the current key
        for (size_t i = 0; i < BEHAV_DIM; ++i)
        {
            current_sample[i] = current_key[i];
        }
    }
    /* generate proposed behaviour */
    std::vector<double> generate()
    {
        if (iteration <= 1) // generate two random initial points
        {
            std::uniform_int_distribution<> dis(0, keys_left.size() - 1);
            current_index = dis(gen);
            current_key = keys_left[current_index];
            set_sample(current_key);
            return current_key;
        }
        else
        {
            find_closest_match();
            return current_key;
        }
    }

    void new_sample_naive()
    {

        // follow gradient
        Eigen::Matrix<double, BEHAV_DIM, 1> delta = (current_sample - previous_sample);
        double delta_F = (current_fitness - previous_fitness) / max_fitness;

        for (size_t i = 0; i < BEHAV_DIM; ++i)
        {
            if (delta[i] != 0)
            {
                delta[i] = delta_F / delta[i]; // otherwise keep delta 0, because undefined
            }
        }
        current_sample = current_sample + alpha * delta;
    }

    void new_sample()
    {
        // follow gradient
        Params::archiveparams::archive_t::iterator close_sample = sample_closest();
        double closest_delta_F = (current_fitness - close_sample->second.fit) / max_fitness;
        Eigen::Matrix<double, BEHAV_DIM, 1> delta;
        for (size_t i = 0; i < BEHAV_DIM; ++i)
        {
            delta[i] = (current_sample[i] - close_sample->first[i]);
            if (delta[i] != 0)
            {
                delta[i] = closest_delta_F / delta[i]; // otherwise keep delta 0, because undefined
            }
#ifdef GRADIENT_LOG
            gradlog << delta[i] << "\t";
            gradlog.flush();
#endif
        }

        current_sample = current_sample + alpha * delta;
#ifdef GRADIENT_LOG
        gradlog << "\n";
#endif
    }
    /* update the proposal mechanism based on the recent sample */
    void update()
    {

        // remove index
        keys_left.erase(keys_left.begin() + current_index);
#ifdef GRADIENT_LOG
        for (size_t i = 0; i < BEHAV_DIM; ++i)
        {
            gradlog << current_key[i] << "\t";
        }
        gradlog << current_fitness << "\t";
#endif
        iteration++;
        if (iteration > 1)
        {
            new_sample();
            // find the closest sample in the map
        }
        else
        {
#ifdef GRADIENT_LOG
            for (size_t i = 0; i < BEHAV_DIM; ++i)
            {
                gradlog << 0 << "\t" ;
            }
            gradlog << std::endl;
            gradlog.flush();
#endif
        }
        previous_fitness = current_fitness;
        previous_sample = current_sample;
        //finally, you can add the sample that was previously used to the list
        keys_sampled[current_key] = Params::archiveparams::archive[current_key]; // (conversion to key will be later so this is previous)
        keys_sampled[current_key].fit = current_fitness;
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
        size_t num_evals = 0;
        while (proposal.keys_left.size() > 0 && num_evals < max_evals)
        {
            // choose new sample
            std::vector<double> sample = proposal.generate();
            Params::archiveparams::elem_archive el = Params::archiveparams::archive.at(sample);
            // run new sample
            proposal.current_fitness = perform_command(el.controller, global::current_config);
            if (proposal.current_fitness > best_so_far)
            {
                best_so_far = proposal.current_fitness;
            }
            // update
            proposal.update();

            // write the statistic to file
            for (size_t i = 0; i < sample.size(); ++i)
            {
                os << sample[i] << " ";
            }
            os << proposal.current_fitness << std::endl;
            std::cout << "fitness = " << proposal.current_fitness << std::endl;
            num_evals++;
        }
    }
};

void run_baseline(const std::string &choice, const std::string &prefix)
{
    global::results_path = prefix;
    std::string filename = prefix + "/" + choice;
    std::ofstream writer(filename.c_str());
    if (choice == "random")
    {
        Baseline<Proposal> baseline;
        baseline.run(writer);
    }
    else if (choice == "gradient_closest")
    {
        Baseline<GradientAscent> baseline;
        baseline.run(writer);
    }
    else
    {
        throw std::runtime_error("not yet implemented");
    }
}

#endif