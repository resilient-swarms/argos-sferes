#ifndef MEAN_ARCHIVE_HPP_
#define MEAN_ARCHIVE_HPP_

namespace limbo {
    namespace mean {
        template <typename Params>
        struct MeanArchive {     // see http://www.resibots.eu/limbo/guides/limbo_concepts.html
            MeanArchive(size_t dim_out = 1)
            {
            }
            template <typename GP>
            Eigen::VectorXd operator()(const Eigen::VectorXd& v, const GP&) const
            {
                //std::cout << "In MeanArchive operator " << std::endl;

                std::vector<double> key(v.size(), 0);
                for (int i = 0; i < v.size(); i++)
                    key[i] = v[i];
                Eigen::VectorXd r(1);
                r(0) = Params::archiveparams::archive.at(key).fit;
                return r;
            }
        };
    }
}

#endif
