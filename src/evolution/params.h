#ifndef ARGOS_SFERES_PARAMS_H
#define ARGOS_SFERES_PARAMS_H

#include <sferes/phen/parameters.hpp>

#include <Eigen/Core>

#include <modules/nn2/mlp.hpp>
#include "phen_dnn.hpp"

#include <sferes/run.hpp>
#include <sferes/stc.hpp>
#include <sferes/misc.hpp>

#include <sferes/gen/evo_float.hpp>
//#include <sferes/ea/nsga2.hpp>
#ifdef CVT
#include <modules/cvt_map_elites/cvt_map_elites.hpp>
#include <modules/cvt_map_elites/fit_map.hpp>
#include <modules/cvt_map_elites/stat_map.hpp>
#include <modules/cvt_map_elites/stat_progress.hpp>

#else
#include <modules/map_elites/map_elites.hpp>
#include <modules/map_elites/fit_map.hpp>
#include <modules/map_elites/stat_map.hpp>
#include <modules/map_elites/stat_progress.hpp>

#endif
//#include <sferes/ea/nsga2.hpp>

#include <sferes/fit/fitness.hpp>
#include <sferes/eval/parallel.hpp>
#include <sferes/eval/eval.hpp>
#include <sferes/stat/pareto_front.hpp>
#include <sferes/stat/best_fit.hpp>
#include <sferes/modif/dummy.hpp>


using namespace sferes;
using namespace sferes::gen::evo_float;
using namespace sferes::gen::dnn;
using namespace nn;

struct ParamsDnn
{
    /* use only the proximity sensors */
    struct dnn
    {
    #ifdef RAB_CONTROL
        static constexpr size_t nb_inputs = 24; // ! 7 ir sensors + 8 RAB sensors + 8 RAB data + bias input at +1
        static constexpr size_t nb_outputs = 3; // 2 motors: left and right wheel + rab output
    #else
        static constexpr size_t nb_inputs = 16; // ! 7 ir sensors + 8 RAB sensors + bias input at +1
        static constexpr size_t nb_outputs = 2; // 2 motors: left and right wheel
    #endif

        static constexpr int io_param_evolving = true;
        static constexpr float m_rate_add_conn = 0.15f;
        static constexpr float m_rate_del_conn = 0.15f;
        static constexpr float m_rate_change_conn = 0.15f;
        static constexpr float m_rate_add_neuron = 0.10f;
        static constexpr float m_rate_del_neuron = 0.10f;

        static constexpr init_t init = random_topology; //random_topology or ff (feed-forward)
        //these only count w/ random init, instead of feed forward
        static constexpr size_t min_nb_neurons = 0;  // does not include input and output neurons
        static constexpr size_t max_nb_neurons = 20; // does not include input and output neurons
        static constexpr size_t min_nb_conns = 0;
        static constexpr size_t max_nb_conns = 40;
    };

    struct parameters
    {
        //Min and max weights of MLP?
        static constexpr float min = -2.0f;
        static constexpr float max = 2.0f;
    };

    struct evo_float
    {
        static constexpr mutation_t mutation_type = polynomial;
        //static const cross_over_t cross_over_type = sbx;
        static constexpr cross_over_t cross_over_type = no_cross_over;
        static constexpr float cross_rate = 0.0f;
        static constexpr float mutation_rate = 0.05f;
        static constexpr float eta_m = 15.0f;
        static constexpr float eta_c = 10.0f;
    };
};
struct EAParams
{
#ifdef CVT
    struct ea
    {
        SFERES_CONST size_t number_of_clusters = 4096;
        SFERES_CONST size_t number_of_dimensions = BEHAV_DIM;
        typedef boost::array<double, number_of_dimensions> point_t;
        static std::vector<point_t> centroids;
    };

#else
    struct ea
    {

        SFERES_CONST double epsilon = 0; //0.05;
        SFERES_CONST size_t behav_dim = BEHAV_DIM;
#if BEHAV_DIM == 1
        SFERES_ARRAY(size_t, behav_shape, 1000);
#elif BEHAV_DIM == 2
        SFERES_ARRAY(size_t, behav_shape, 64, 64);
#elif BEHAV_DIM == 3
        SFERES_ARRAY(size_t, behav_shape, 16,16,16);
#elif BEHAV_DIM == 6
        SFERES_ARRAY(size_t, behav_shape, 4, 4, 4, 4, 4, 4);
#elif BEHAV_DIM == 7
        SFERES_ARRAY(size_t, behav_shape, 3, 3, 3, 3, 3, 3, 3);
#else
#error "Unsupported BEHAV_DIM setting (choose 2,3,6 or 7)"
#endif
    };
#endif

    struct parameters
    {
        //Min and max weights of MLP?
        static constexpr float min = -2.0f;
        static constexpr float max = 2.0f;
    };

    struct evo_float
    {
        static constexpr mutation_t mutation_type = polynomial;
        //static const cross_over_t cross_over_type = sbx;
        static constexpr cross_over_t cross_over_type = no_cross_over;
        static constexpr float cross_rate = 0.0f;
        static constexpr float mutation_rate = 0.05f;
        static constexpr float eta_m = 15.0f;
        static constexpr float eta_c = 10.0f;
    };

    struct pop
    {
        // number of initial random points
        SFERES_CONST size_t init_size = 1000; //1000;
        // size of a batch
        SFERES_CONST size_t size = 40;//1000;
        SFERES_CONST size_t nb_gen = 10001;
        SFERES_CONST size_t dump_period = 100;
    };
};

namespace robots_nn
{
typedef phen::Parameters<gen::EvoFloat<1, ParamsDnn>, fit::FitDummy<>, ParamsDnn> weight_t;
typedef phen::Parameters<gen::EvoFloat<1, ParamsDnn>, fit::FitDummy<>, ParamsDnn> bias_t;

typedef PfWSum<weight_t> pf_t;
typedef AfTanh<bias_t> af_t;
typedef Neuron<pf_t, af_t> neuron_t;
typedef Connection<weight_t> connection_t;
typedef sferes::gen::Dnn<neuron_t, connection_t, ParamsDnn> gen_t;
typedef typename gen_t::nn_t nn_t; // not sure if typename should be here?
} // namespace robots_nn

#endif //ARGOS_SFERES_PARAMS_H