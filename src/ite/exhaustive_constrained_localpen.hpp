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

                std::cout << "In ExhaustiveConstrainedLocalPenalty operator " << std::endl;
                std::cout << "with constraint " << constraint << std::endl;
                float best_acqui = -INFINITY;
                Eigen::VectorXd result;

                int best_index;
                size_t constraint_size = constraint.size();
                archive_it_t best_it;
                int map_index;
                if (Params::multi)
                {
                    map_index = Params::map_index;
                    Params::archiveparams::archive = Params::archiveparams::multimap[map_index];
                }
                if (Params::LOCAL_L)
                {
                    Params::LL.clear();
                    for (int i = 0; i < Params::busy_samples.size(); ++i)
                    {
                        Params::LL.push_back(get_L<F>(f, constraint, i));
                        std::cout << "using L_" << i << "=" << Params::LL[i] << std::endl;
                    }
                }
                else
                {
                    Params::L = get_L<F>(f, constraint);
                    std::cout << "using L=" << Params::L << std::endl;
                }
                // Params::M = get_M<F>(f, constraint); //get_M<F>(f, constraint); // set to max observation instead

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
                    }
                }
                std::cout << "best UCB " << best_acqui << std::endl;
                std::cout << "vector " << result << std::endl;
                if (best_it->first.size() != 0)
                {
                    Params::archiveparams::archive.at(best_it->first).checked = true;
                }
                if (Params::multi)
                {
                    Params::archiveparams::multimap[map_index] = Params::archiveparams::archive;
                }
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

                    //auto el = Params::archiveparams::archive.at(it->first);
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
            double get_L(F &f, const Eigen::VectorXd &constraint, int busy_index = -1) const
            {
                if (Params::busy_samples.empty())
                {
                    return 0.0f;
                }

                float best_gradnorm = -INFINITY;
                size_t constraint_size = constraint.size();
                archive_it_t best_it;
                double neighbourhood;
                Eigen::VectorXd busy;
                if (busy_index != -1)
                {
                    //
                    // "In our experiments we used a Matern-52 kernel and defined
                    // the local region for evaluating the Lipschitz constant for
                    // a batch point xj to be a hypercube centred on xj with the length of the each side equal to the lengthscale of that input dimension
                    // --> in our case we do the same, just check that all dimensions within lengthscale
                    //""
                    busy = Params::busy_samples[busy_index];
                    std::cout << "getting local lipschitz constant for busy sample " << busy_index << ": " << busy.transpose() << std::endl;
                    std::cout << "looking within hypercube of size: " << Params::kernel_maternfivehalves::l() << std::endl;
                    neighbourhood = Params::kernel_maternfivehalves::l();
                }

                for (archive_it_t it = Params::archiveparams::archive.begin(); it != Params::archiveparams::archive.end(); ++it)
                {

                    Eigen::VectorXd temp(it->first.size());
                    Eigen::VectorXd constraint_check(it->first.size());
                    bool within_hypercube = true;

                    //std::cout << "start filling vectors of size " << it->first.size() << std::endl;
                    for (size_t i = 0; i < it->first.size(); i++)
                    {
                        //std::cout << i << std::endl;
                        temp[i] = it->first[i];
                        if (busy_index != -1)
                        {
                            double dist = std::abs(busy[i] - temp[i]);
                            if (dist > neighbourhood)
                            {
                                within_hypercube = false;
                                break;
                            }
                        }

                        if (i < BEHAV_DIM || constraint_size == 0)
                        {
                            constraint_check[i] = it->first[i];
                        }
                        else
                        {

                            constraint_check[i] = constraint[i - BEHAV_DIM];
                        }
                    }
                    if (!within_hypercube)
                    {
                        continue;
                    }

                    if (Params::archiveparams::classcomp::inequality(temp, constraint_check) || Params::archiveparams::classcomp::inequality(constraint_check, temp))
                    {
                        // std::cout << "Skipping bd not according to constraint: \n"
                        //           << temp;
                        // std::cout << "constraint:\n"
                        //           << constraint;
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
                if (best_gradnorm < 1e-7)
                {
                    best_gradnorm = 10.0; //to avoid problems in cases in which the model is flat; not sure why 1e-7 vs 10 but this is used in GPyOpt
                }
                return best_gradnorm;
            }
        };

    } // namespace opt
} // namespace limbo
#endif
