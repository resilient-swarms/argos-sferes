#ifndef EXHAUSTIVE_CONSTRAINED_SEARCH_ARCHIVE_HPP_
#define EXHAUSTIVE_CONSTRAINED_SEARCH_ARCHIVE_HPP_

#include <set>
#include <iterator>

namespace limbo {
    namespace opt {
        template <typename Params>
        struct ExhaustiveConstrainedSearchArchive {
            typedef typename Params::archiveparams::archive_t::const_iterator archive_it_t;
            ExhaustiveConstrainedSearchArchive() {}
            template <typename F>
            Eigen::VectorXd operator()(const F& f, const Eigen::VectorXd& init, bool bounded, const Eigen::VectorXd& constraint) const
            {
                std::cout << "In ExhaustiveSearchArchive operator " << std::endl;

                float best_acqui = -INFINITY;
                Eigen::VectorXd result;
                
                int best_index;
                size_t constraint_size = constraint.size();
                archive_it_t best_it;
                for (archive_it_t it = Params::archiveparams::archive.begin(); it != Params::archiveparams::archive.end(); ++it) {
                    
                    
                    Eigen::VectorXd temp(it->first.size());
                    for (size_t i = 0; i < it->first.size(); i++)
                        temp[i] = it->first[i];

                    bool checked = Params::archiveparams::archive.at(it->first).checked;
                    if (checked )
                    {
                        continue;// no need to check again, assuming static environment
                    }
                    if(temp.tail(constraint_size) != constraint)
                    {
                        //std::cout<< "Skipping bd not according to constraint: \n"<< temp;
                        continue;
                    }
                    
                    float new_acqui = eval(f, temp);
                    std::cout << "temp: "<<temp.transpose()<<std::endl;
                    std::cout << "fit: " << new_acqui << std::endl;
                    if (best_acqui < new_acqui || it == Params::archiveparams::archive.begin()) {
                        best_acqui = new_acqui;
                        result = temp;
                        best_it = it;
                    }
                }
                std::cout << "best UCB " << best_acqui << std::endl;
                std::cout << "vector " << result << std::endl;
                Params::archiveparams::archive.at(best_it->first).checked=true;
                return result;
            }
        };
    }
}
#endif
