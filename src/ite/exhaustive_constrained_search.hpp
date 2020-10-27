#ifndef EXHAUSTIVE_CONSTRAINED_SEARCH_ARCHIVE_HPP_
#define EXHAUSTIVE_CONSTRAINED_SEARCH_ARCHIVE_HPP_

#include <set>
#include <iterator>

namespace limbo
{
    namespace opt
    {
        template <typename Params>
        struct ExhaustiveConstrainedSearchArchive
        {
            typedef typename Params::archiveparams::archive_t::const_iterator archive_it_t;
            ExhaustiveConstrainedSearchArchive() {}
            template <typename F>
            Eigen::VectorXd operator()(const F &f, const Eigen::VectorXd &init, bool bounded, const Eigen::VectorXd &constraint) const
            {
                std::cout << "In ExhaustiveConstrainedSearchArchive operator " << std::endl;

                float best_acqui = -INFINITY;
                Eigen::VectorXd result;
                std::vector<double> checked_vec;
                int best_index;
                size_t constraint_size = constraint.size();
                archive_it_t best_it;
                int map_index;
                if (Params::multi)
                {
                    map_index = Params::map_index;
                    Params::archiveparams::archive = Params::archiveparams::multimap[map_index];
                }
                for (archive_it_t it = Params::archiveparams::archive.begin(); it != Params::archiveparams::archive.end(); ++it)
                {

                    Eigen::VectorXd temp(it->first.size());
                    Eigen::VectorXd constraint_check(it->first.size());
                    std::vector<double> vec_to_check;
                    //std::cout << "start filling vectors of size " << it->first.size() << std::endl;
                    for (size_t i = 0; i < it->first.size(); i++)
                    {
                        //std::cout << i << std::endl;
                        temp[i] = it->first[i];
                        if (i < BEHAV_DIM || constraint_size == 0)
                        {
                            constraint_check[i] = it->first[i];
                        }
                        else
                        {

                            constraint_check[i] = constraint[i - BEHAV_DIM];
                        }
                        vec_to_check.push_back(constraint_check[i]);
                    }

                    auto el = Params::archiveparams::archive.at(it->first);
                    bool checked = Params::archiveparams::checked_constraints[vec_to_check];
                    if (checked)
                    {
                        continue; // no need to check again, assuming static environment
                    }
                    //std::cout << "not checked yet" << std::endl;
                    //std::cout << "will compare to constraint" << std::endl;
                    //std::cout << "temp=\n"<<temp;
                    //std::cout << "constraint_check\n" << constraint_check;
                    if (Params::archiveparams::classcomp::inequality(temp, constraint_check) || Params::archiveparams::classcomp::inequality(constraint_check, temp))
                    {
                        // std::cout << "Skipping bd not according to constraint: \n"
                        //           << temp;
                        // std::cout << "constraint:\n"
                        //           << constraint.transpose();
                        continue;
                    }

                    float new_acqui = eval(f, temp);
                    //std::cout << "temp: "<<temp.transpose()<<std::endl;
                    //std::cout << "fit: " << new_acqui << std::endl;
                    if (best_acqui < new_acqui || it == Params::archiveparams::archive.begin())
                    {
                        best_acqui = new_acqui;
                        result = temp;
                        best_it = it;
                        checked_vec = vec_to_check;
                    }
                }
                std::cout << "best UCB " << best_acqui << std::endl;
                std::cout << "vector " << result << std::endl;
                if (checked_vec.size() != 0)
                {
                    Params::archiveparams::checked_constraints[checked_vec] = true;
                }
                if (Params::multi)
                {
                    Params::archiveparams::multimap[map_index] = Params::archiveparams::archive;
                }
                return result;
            }
        };
    } // namespace opt
} // namespace limbo
#endif
