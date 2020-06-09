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
		    Eigen::VectorXd constraint_check(it->first.size());
                    //std::cout << "start filling vectors of size " << it->first.size() << std::endl;
		    for (size_t i = 0; i < it->first.size(); i++)
		    {
                        //std::cout << i << std::endl;
                        temp[i] = it->first[i];
			if(i< BEHAV_DIM || constraint_size == 0)
		        {
			    constraint_check[i] = it->first[i];
			}
			else{
			
			    constraint_check[i] = constraint[i-BEHAV_DIM];
			}		
		    }

                    auto el = Params::archiveparams::archive.at(it->first);
                    if (el.checked )
                    {
                        continue;// no need to check again, assuming static environment
                    }
		    //std::cout << "not checked yet" << std::endl;
		    //std::cout << "will compare to constraint" << std::endl;
		    //std::cout << "temp=\n"<<temp;
		    //std::cout << "constraint_check\n" << constraint_check;
                    if(Params::archiveparams::classcomp::inequality(temp,constraint_check) || Params::archiveparams::classcomp::inequality(constraint_check,temp))
                    {
                        std::cout<< "Skipping bd not according to constraint: \n"<< temp;
                        std::cout<<"constraint:\n"<<constraint;
                        continue;
                    }
                    
                    float new_acqui = eval(f, temp);
                    //std::cout << "temp: "<<temp.transpose()<<std::endl;
                    //std::cout << "fit: " << new_acqui << std::endl;
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
