#ifndef EXHAUSTIVE_CONSTRAINED_LOCALPENALTY_HPP_
#define EXHAUSTIVE_CONSTRAINED_LOCALPENALTY_HPP_

#include <set>
#include <iterator>

namespace limbo
{
    namespace opt
    {
        template <typename Params>
        struct ExhaustiveConstrainedLocalPenalty
        {
            const double minL = 0.1; //minimal lipschitz constant
            typedef typename Params::archiveparams::archive_t::const_iterator archive_it_t;
            ExhaustiveConstrainedLocalPenalty() {}
            template <typename F, typename AggregatorFun>
            Eigen::VectorXd operator()(F &f, const Eigen::VectorXd &init, bool bounded, const Eigen::VectorXd &constraint, AggregatorFun afun) const
            {

                Params::M =  get_M<F>(f, constraint);// set to max observation instead
                Params::L = std::max(minL, get_L<F>(f, constraint));
                std::cout << "using L=" << Params::L << std::endl;
                std::cout << "In ExhaustiveConstrainedLocalPenalty operator " << std::endl;

                float best_acqui = -INFINITY;
                Eigen::VectorXd result;

                int best_index;
                size_t constraint_size = constraint.size();
                archive_it_t best_it;
                for (archive_it_t it = Params::archiveparams::archive.begin(); it != Params::archiveparams::archive.end(); ++it)
                {

                    Eigen::VectorXd temp(it->first.size());
                    Eigen::VectorXd constraint_check(it->first.size());
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
                    }

                    auto el = Params::archiveparams::archive.at(it->first);
                    if (el.checked)
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
                        //           << constraint;
                        // continue;
                    }

                    float new_acqui = eval(f, temp);
                    //std::cout << "temp: "<<temp.transpose()<<std::endl;
                    //std::cout << "fit: " << new_acqui << std::endl;
                    if (best_acqui < new_acqui || it == Params::archiveparams::archive.begin())
                    {
                        best_acqui = new_acqui;
                        result = temp;
                        best_it = it;
                    }
                }
                std::cout << "best UCB " << best_acqui << std::endl;
                std::cout << "vector " << result << std::endl;
                Params::archiveparams::archive.at(best_it->first).checked = true;
                return result;
            }

            /* helper to get the estimated maximal performance in the archive */
            template <typename F>
            double get_M(F &f, const Eigen::VectorXd &constraint) const
            {
                if (Params::busy_samples.empty())
                {
                    return 0.0f;
                }
                float best_mean = -INFINITY;
                size_t constraint_size = constraint.size();
                for (archive_it_t it = Params::archiveparams::archive.begin(); it != Params::archiveparams::archive.end(); ++it)
                {

                    Eigen::VectorXd temp(it->first.size());
                    Eigen::VectorXd constraint_check(it->first.size());
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
                    }

                    auto el = Params::archiveparams::archive.at(it->first);
                    //std::cout << "not checked yet" << std::endl;
                    //std::cout << "will compare to constraint" << std::endl;
                    //std::cout << "temp=\n"<<temp;
                    //std::cout << "constraint_check\n" << constraint_check;
                    if (Params::archiveparams::classcomp::inequality(temp, constraint_check) || Params::archiveparams::classcomp::inequality(constraint_check, temp))
                    {
                        // std::cout << "Skipping bd not according to constraint: \n"
                        //           << temp;
                        // std::cout << "constraint:\n"
                        //           << constraint;
                        continue;
                    }

                    double new_mean = f.get_performance(temp);
                    if (best_mean < new_mean || it == Params::archiveparams::archive.begin())
                    {
                        best_mean = new_mean;
                    }
                }
                std::cout << "best mean performance " << best_mean << std::endl;

                return best_mean;
            }

            /* helper to get lipschitz constant */
            template <typename F>
            double get_L(F &f, const Eigen::VectorXd &constraint) const
            {
                if (Params::busy_samples.empty())
                {
                    return 0.0f;
                }
                float best_gradnorm = -INFINITY;
                size_t constraint_size = constraint.size();
                archive_it_t best_it;

                for (archive_it_t it = Params::archiveparams::archive.begin(); it != Params::archiveparams::archive.end(); ++it)
                {

                    Eigen::VectorXd temp(it->first.size());
                    Eigen::VectorXd constraint_check(it->first.size());
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
                    }

                    auto el = Params::archiveparams::archive.at(it->first);
                    if (el.checked)
                    {
                        continue; // no need to check again, assuming static environment
                    }
                    //std::cout << "not checked yet" << std::endl;
                    //std::cout << "will compare to constraint" << std::endl;
                    //std::cout << "temp=\n"<<temp;
                    //std::cout << "constraint_check\n" << constraint_check;
                    if (Params::archiveparams::classcomp::inequality(temp, constraint_check) || Params::archiveparams::classcomp::inequality(constraint_check, temp))
                    {
                        //std::cout << "Skipping bd not according to constraint: \n"
                        //          << temp;
                        //std::cout << "constraint:\n"
                        //          << constraint;
                        continue;
                    }

                    float new_gradnorm = f.get_gradnorm(temp);
                    //std::cout << "temp: "<<temp.transpose()<<std::endl;
                    //std::cout << "fit: " << new_acqui << std::endl;
                    if (best_gradnorm < new_gradnorm || it == Params::archiveparams::archive.begin())
                    {
                        best_gradnorm = new_gradnorm;
                    }
                }
                std::cout << "best gradnorm " << best_gradnorm << std::endl;
                return best_gradnorm;
            }
        };

    } // namespace opt
} // namespace limbo
#endif
