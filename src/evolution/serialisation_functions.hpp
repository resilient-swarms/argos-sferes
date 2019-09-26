//
// Created by Matthew Hayslep on 02/08/19.
//

#ifndef ARGOS_SFERES_NNSERIALISER_FUNCTIONS_HPP
#define ARGOS_SFERES_NNSERIALISER_FUNCTIONS_HPP

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <fstream>
#include <src/evolution/params.h>
//#include "server_functions.hpp"

using namespace boost::archive;
using namespace robots_nn;

class NNSerialiser {
private:
    std::string savefile = "BOOST_SERIALIZATION_NVP";
public:
    NNSerialiser(){};
    NNSerialiser(std::string name)
    {
        savefile = name;
    }


    /// save data (neural network) to text archive file (must be implemented in the header)
    template<typename Indiv>
    void Save(Indiv &ind) {
	std::cout<<"Serialising individual"<<std::endl;
        std::ofstream ofs(savefile.c_str());
        text_oarchive oa(ofs);
        /// save genotype to text archive file
        oa << BOOST_SERIALIZATION_NVP(ind.gen());
    }

    /// load data (neural network) from text archive file
    nn_t Load() {

        gen_t loaded_ind;

        std::ifstream ifs(savefile);
        text_iarchive ia(ifs);
        ia >> BOOST_SERIALIZATION_NVP(loaded_ind);

        // develop the parameters
        BGL_FORALL_VERTICES_T(v, loaded_ind.get_graph(),
                              typename nn_t::graph_t)
            {
                loaded_ind.get_graph()[v].get_afparams().develop();
                loaded_ind.get_graph()[v].get_pfparams().develop();
                loaded_ind.get_graph()[v].set_afparams(loaded_ind.get_graph()[v].get_afparams());
                loaded_ind.get_graph()[v].set_pfparams(loaded_ind.get_graph()[v].get_pfparams());
            }
        BGL_FORALL_EDGES_T(e, loaded_ind.get_graph(),
                           typename nn_t::graph_t)
            {
                loaded_ind.get_graph()[e].get_weight().develop();
            }
        // init everything
        loaded_ind.init();
        return loaded_ind;
}

};
#endif //ARGOS_SFERES_NNSERIALISER_FUNCTIONS_HPP
