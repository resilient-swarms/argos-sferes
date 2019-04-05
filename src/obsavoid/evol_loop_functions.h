/****************************************/
/****************************************/
/* ARGoS related headers */
/* The NN controller */
#include <tuple>
#include <cmath>
#include <unordered_set>
#include <src/obsavoid/nn_controller.h>

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/thymio/simulator/thymio_entity.h>

/****************************************/
/****************************************/
/* Sferes related headers */

#include <sferes/phen/parameters.hpp>

#include <Eigen/Core>

#include <modules/nn2/mlp.hpp>
#include "phen_dnn.hpp"

#include <sferes/run.hpp>
#include <sferes/stc.hpp>
#include <sferes/misc.hpp>

#include <sferes/gen/evo_float.hpp>
//#include <sferes/ea/nsga2.hpp>

#include <modules/map_elites/map_elites.hpp>
#include <modules/map_elites/fit_map.hpp>
#include <modules/map_elites/stat_map.hpp>
#include <modules/map_elites/stat_progress.hpp>

#include <sferes/fit/fitness.hpp>
#include <sferes/eval/parallel.hpp>
#include <sferes/eval/eval.hpp>
#include <sferes/stat/pareto_front.hpp>
#include <sferes/stat/best_fit.hpp>
#include <sferes/modif/dummy.hpp>

#define PRINTING

/****************************************/
/****************************************/

using namespace sferes;
using namespace sferes::gen::evo_float;
using namespace sferes::gen::dnn;
using namespace nn;


class RunningStat
{
public:
    RunningStat(): n(0),_M(0.),_S(0.)
    {

    }
    size_t n;
    float _M, _S;
    void push(float x)
    {
        ++n;
        if (n==1)
        {
            _M = x;
        }
        else
        {
            float oldM = _M;
            _M += (x - oldM)/(float)n;
            _S = _S + (x - oldM)*(x - _M);
        }

    }
    float mean()
    {

        return _M;
    }
    float var()
    {

        return n > 1 ? _S/(float)(n - 1) : _M*_M;
    }
    float std()
    {
        return sqrt(var());
    }

};

struct ParamsDnn
{
    struct dnn
    {
        static constexpr size_t nb_inputs       = 8; //! 7 ir sensors + bias input at +1
        static constexpr size_t nb_outputs      = 2; // 2 motors: left and right wheel

        static constexpr int io_param_evolving = true;
        static constexpr float m_rate_add_conn  = 0.05f;
        static constexpr float m_rate_del_conn  = 0.1f;
        static constexpr float m_rate_change_conn = 0.05f;
        static constexpr float m_rate_add_neuron  = 0.05f;
        static constexpr float m_rate_del_neuron  = 0.1f;

        static constexpr init_t init = random_topology; //random_topology or ff (feed-forward)
        //these only count w/ random init, instead of feed forward
        static constexpr size_t min_nb_neurons  = 4; // does not include input and output neurons
        static constexpr size_t max_nb_neurons  = 10; // does not include input and output neurons
        static constexpr size_t min_nb_conns    = nb_inputs*nb_outputs;
        static constexpr size_t max_nb_conns    = 40;
    };

    struct parameters
    {
        //Min and max weights of MLP?
        static constexpr float min = -5.0f;
        static constexpr float max = 5.0f;
    };

    struct evo_float
    {
        static constexpr mutation_t mutation_type = polynomial;
        //static const cross_over_t cross_over_type = sbx;
        static constexpr cross_over_t cross_over_type = no_cross_over;
        static constexpr float cross_rate = 0.0f;
        static constexpr float mutation_rate = 0.1f;
        static constexpr float eta_m = 15.0f;
        static constexpr float eta_c = 10.0f;
    };
};

namespace robots_nn
{
typedef phen::Parameters<gen::EvoFloat<1, ParamsDnn>, fit::FitDummy<>, ParamsDnn> weight_t;
typedef phen::Parameters<gen::EvoFloat<1, ParamsDnn>, fit::FitDummy<>, ParamsDnn> bias_t;

typedef PfWSum<weight_t> pf_t;
typedef AfTanh<bias_t> af_t;
typedef Neuron<pf_t, af_t>  neuron_t;
typedef Connection <weight_t> connection_t;
typedef sferes::gen::Dnn<neuron_t, connection_t, ParamsDnn> gen_t;
typedef typename gen_t::nn_t nn_t; // not sure if typename should be here?
}


/****************************************/
/****************************************/



class Descriptor;


class StatFuns{
public:
    /*  Combine info across trials  */
    static float mean(std::vector<float> results){
        float sum = std::accumulate(results.begin(), results.end(), 0.0);
        return sum / (float) results.size();
    }
    static float standard_dev(std::vector<float> results){
        float mean=StatFuns::mean(results);
        float var = 0.0;
        for( size_t n = 0; n < results.size(); n++ )
        {
            var += (results[n] - mean) * (results[n] - mean);
        }
        var /= (float) results.size();
        return sqrt(var);
    }
    static float min(std::vector<float> results)  
    {
        return *std::min_element(results.begin(), results.end());
    }
    static float sum(std::vector<float> results)
    {
        return std::accumulate(results.begin(), results.end(), 0.0);
    }

    static float get_minkowski_distance(CVector3 x, CVector3 y,int k=3) {
        /* get the minkowski-distance between two 3D vectors ; k is the parameter that determines e.g. manhattan vs Euclid vs 3D movements 
           on a flat surface, the Thymio can only move in two directions, so in that case use k=2
           in general, surface may not be flat, then k=3 makes sense

           alternative in Argos is Distance
           https://www.argos-sim.info/api/a00293_source.php#l00205
           l. 684; problem is it always uses k=2
        */
        float sum = 0.0f;
        sum += std::pow(std::abs(x.GetX() -y.GetX()), k);

        
        sum += std::pow(std::abs(x.GetY() -y.GetY()), k);

   
        sum += std::pow(std::abs(x.GetZ() -y.GetZ()), k);

        return std::pow(sum, 1.0 / (float) k);
    }
    static float uniform_prob(size_t n)
    {
        float uniform_p = 1.0f/(float)n;
        return uniform_p;
    }
    static float max_variation_distance(size_t n)
    {
        float uniform_prob=StatFuns::uniform_prob(n);
        return 1.0 - uniform_prob;

    }
    static float min_variation_distance()
    {

        return 0.;

    }
    static float uniformity(std::vector<float> probabilities)
    {
        /* variation distance is a measure of distance between distributions (here to uniform distr); other options: KL divergence, Kolmogorov distance  
        *  it is defined as the largest possible probability difference for the same event
        */
        float uni_p = StatFuns::uniform_prob(probabilities.size());
        float dist = 0.;
        for ( float p : probabilities ) 
        {
            dist += std::abs(p - uni_p);
        }
        dist*=0.50f;
        float m=StatFuns::max_variation_distance(probabilities.size());
        assert (dist<=m);
        return m - dist;
    }
    static float get_avg_dist(std::vector<CVector3> positions , CVector3 cm)
    {
        float avg_dist=0.0f;
        for(CVector3 pos:positions)
        {
            float dist=StatFuns::get_minkowski_distance(pos,cm);
            #ifdef PRINTING
                std::cout<<"position: "<<pos<<std::endl;
                std::cout<<"dist: "<<dist<<std::endl;
            #endif
            avg_dist+=dist;
        }
        
        avg_dist/=(float)positions.size();
        #ifdef PRINTING
            std::cout<<"avg_dist: "<<avg_dist<<std::endl;
        #endif
        return avg_dist;
    }
};


class FitFun;


class CObsAvoidEvolLoopFunctions : public CLoopFunctions
{

public:
    CObsAvoidEvolLoopFunctions();
    virtual ~CObsAvoidEvolLoopFunctions();

    virtual void Init(TConfigurationNode& t_node);

    virtual void Reset();

    /* Called by the evolutionary algorithm to set the current trial */
    inline void SetTrial(size_t un_trial)
    {
        m_unCurrentTrial = un_trial;
    }

//    /* Configures the robot controller from the genome */
//    void ConfigureFromGenome(robots_nn::nn_t& ctrl)
//    {
//        _ctrlrob = ctrl;
//    }

    virtual void  PreStep();
    virtual void  PostStep();

    CVector3 get_position(CThymioEntity* robot)
    {

      return robot->GetEmbodiedEntity().GetOriginAnchor().Position;
    }




private:

    /* The initial setup of a trial */
    struct SInitSetup
    {
        CVector3 Position;
        CQuaternion Orientation;
    };

    //CEPuckEntity* m_pcEPuck;
    //CEPuckNNController* m_pcController;
    
    std::vector<CThymioNNController*>  m_pcvecController;


public:
    std::vector<CThymioEntity*>  m_pcvecRobot;


    CRandom::CRNG* m_pcRNG;

public:
    std::vector< std::vector<SInitSetup> > m_vecInitSetup;
    size_t m_unNumberTrials, m_unCurrentTrial, m_unNumberRobots;


public:
    //robots_nn::nn_t _ctrlrob;
    std::vector<robots_nn::nn_t> _vecctrlrob;
    std::vector<float> outf, inputs;

    Real nb_coll, stand_still;
    bool stop_eval;
    float speed, lin_speed;

    /*descriptors*/
    float num_ds;
    Descriptor* descriptor;
    FitFun* fitfun;
    int bd_dims;


    // only used for the checks which are not used (presumably the checks quite expensive) ?; also not suitable for multi-agent ?
    CVector3 old_pos;  CRadians old_theta;
    CVector3 curr_pos; CRadians curr_theta;
    CVector3 centre, max;
};

class FitFun{
public:
    std::vector<float> fitness_per_trial;
    FitFun(){
    }


    /*after getting inputs, can update the descriptor if needed*/
    virtual void apply(CObsAvoidEvolLoopFunctions& cLoopFunctions,Real time)=0;

    /*after getting outputs, can update the descriptor if needed*/
    virtual float after_trials()=0;


};


class FloreanoMondada : public FitFun{
    //Floreano & Mondada '94; minimum of the linear speed, controlling for collision 
public:
    FloreanoMondada(){}

    /*after getting inputs, can update the descriptor if needed*/
    virtual void apply(CObsAvoidEvolLoopFunctions& cLoopFunctions,Real time){
        float fit=cLoopFunctions.lin_speed/time*(Real)cLoopFunctions.nb_coll/time;
        fitness_per_trial.push_back(fit);
    };

    /*after getting outputs, can update the descriptor if needed*/
    virtual float after_trials(){
        float minfit=StatFuns::min(fitness_per_trial);
        fitness_per_trial.clear();
    };


};

class MeanSpeed : public FitFun{
    //Floreano & Mondada '94; minimum of the linear speed
public:
    MeanSpeed(){}

    /*after getting inputs, can update the descriptor if needed*/
    virtual void apply(CObsAvoidEvolLoopFunctions& cLoopFunctions,Real time){
        float fit=cLoopFunctions.lin_speed/time*(Real)cLoopFunctions.nb_coll/time;
        fitness_per_trial.push_back(fit);
    };

    /*after getting outputs, can update the descriptor if needed*/
    virtual float after_trials(){
        float meanfit = StatFuns::mean(fitness_per_trial);
        fitness_per_trial.clear();
    };


};


class Aggregation : public FitFun{
    //Floreano & Mondada '94; minimum of the linear speed
public:
    Aggregation(){
    }

    /*after getting inputs, can update the descriptor if needed*/
    virtual void apply(CObsAvoidEvolLoopFunctions& cLoopFunctions,Real time){
        std::pair<std::vector<CVector3>,CVector3> data =centre_of_mass(cLoopFunctions);
        float dist = StatFuns::get_avg_dist(data.first,data.second);
        fitness_per_trial.push_back(1.0f/dist);
    };

    /*after getting outputs, can update the descriptor if needed*/
    virtual float after_trials(){
        float meanfit = StatFuns::mean(fitness_per_trial);
        fitness_per_trial.clear();
    };
    float get_mass(CThymioEntity* robot)
    {
        return 1.0f;
    }

    std::pair<std::vector<CVector3>,CVector3> centre_of_mass(CObsAvoidEvolLoopFunctions&  cLoopFunctions)
    {
        float M = 0.0;
        CVector3 cm=CVector3(0.,0.,0.);
        std::vector<CVector3> positions;

        for (CThymioEntity* robot : cLoopFunctions.m_pcvecRobot)
        {
            
            float mass = get_mass(robot);
            M+=mass;
            CVector3 pos = cLoopFunctions.get_position(robot);
            cm += mass*pos;
            positions.push_back(pos);
        }
        cm/=M;
        #ifdef PRINTING
            std::cout<<"centre of mass: "<<cm<<std::endl;
        #endif
        return std::pair<std::vector<CVector3>,CVector3>(positions, cm);

    }


};



class Descriptor{
public:
    Descriptor(){        

    }

    static const size_t behav_dim=ParamsDnn::dnn::nb_inputs - 1; // for now all the same
    /* final value of bd*/
    std::vector<float> bd;
        /* prepare for trials*/
    virtual void before_trials(argos::CSimulator& cSimulator){

        bd.resize(behav_dim,0.0f);
    }
        /*reset BD at the start of a trial*/
    virtual void start_trial()
    {
    }
    /*after getting inputs, can update the descriptor if needed*/
    virtual void set_input_descriptor(CObsAvoidEvolLoopFunctions& cLoopFunctions){};

    /*after getting outputs, can update the descriptor if needed*/
    virtual void set_output_descriptor(CObsAvoidEvolLoopFunctions& cLoopFunctions){};




    /*end the trial*/
    virtual void end_trial(CObsAvoidEvolLoopFunctions& cLoopFunctions)
    {
    }

    /*summarise BD at the end of trials*/
    virtual std::vector<float> after_trials(Real time, CObsAvoidEvolLoopFunctions& cLoopFunctions)=0;

};
class AverageDescriptor: public Descriptor{
    /* Get the average sensory readings averaged within and between trial
     */
public:
    AverageDescriptor(){

        
    }
    
    /*after getting inputs, can update the descriptor if needed*/
    virtual void set_input_descriptor(CObsAvoidEvolLoopFunctions& cLoopFunctions);


    /*summarise BD at the end of trials*/
    virtual std::vector<float> after_trials(Real time, CObsAvoidEvolLoopFunctions& cLoopFunctions);
};


class IntuitiveHistoryDescriptor: public Descriptor{
    /* 
    *  track observation-action or state-action pairs over time 
    *  after all trials, gather statistics of the observed history
    */
public:
    IntuitiveHistoryDescriptor(CLoopFunctions& cLoopFunctions){
        //bd.resize(ParamsDnn::dnn::nb_inputs + ParamsDnn::dnn::nb_outputs - 1, 0.0f); // Params::dnn::nb_inputs includes sensors and a bias input
    }
    std::map<CVector3, size_t> unique_visited_positions;
    RunningStat velocity_stats;
    CVector3 center;
    const float grid_step=0.50;
    size_t visitation_count;
    float max_deviation, deviation;



    /* prepare for trials*/
    virtual void before_trials(argos::CSimulator& cSimulator){
        bd.resize(behav_dim,0.0f);
        //define member variables
        center = cSimulator.GetSpace().GetArenaCenter();
        
        // initialise grid (for calculating coverage and uniformity)
        CVector3 max = cSimulator.GetSpace().GetArenaSize();
        CVector3 min = center - 0.5*cSimulator.GetSpace().GetArenaSize();
        max_deviation = StatFuns::get_minkowski_distance(max,center);
    

    }
    /*reset BD at the start of a trial*/
    virtual void start_trial()
    {
        deviation=0.0;
        visitation_count=0;
        velocity_stats=RunningStat();
    }
    /*after getting inputs, can update the descriptor if needed*/
    virtual void set_input_descriptor(CObsAvoidEvolLoopFunctions& cLoopFunctions);
    /*end the trial*/
    virtual void end_trial(CObsAvoidEvolLoopFunctions& cLoopFunctions);
    /*summarise BD at the end of trials*/
    virtual std::vector<float> after_trials(Real time, CObsAvoidEvolLoopFunctions& cLoopFunctions);






    CVector3 get_bin(CVector3 vec)
    {
        int binx= (int) (vec.GetX()/grid_step);
        int biny= (int) (vec.GetY()/grid_step);
        int binz= (int) (vec.GetZ()/grid_step);
        CVector3 new_vec = CVector3(binx*grid_step,biny*grid_step,binz*grid_step);
        #ifdef PRINTING
            std::cout<<"binned "<< vec  <<"into "<<new_vec<<std::endl;
        #endif
        return new_vec;

    }
    std::vector<float> get_probs()
    {
        std::vector<float> a;
         for( auto pair : unique_visited_positions )
        {
            a.push_back(pair.second/(float) visitation_count);
            #ifdef PRINTING
                std::cout<<"total visits"<<visitation_count<<std::endl;
                std::cout<<"location "<<pair.first<<std::endl;
                std::cout<<"visits " << pair.second<<std::endl;
                std::cout<<"a.back() " << a.back()<<std::endl;
            #endif

        }

        return a;
    }




};

// class RNNHistoryDescriptor: public HistoryDescriptor{
//     /* 
//     *  
//     *  use RNN to process observed oa-history
//     */
// public:
//     ModusHistoryDescriptor(){

//     }
//     void set_RNN_from_config()
//     {

//     }
//     static const size_t behav_dim=7;
//     /*reset BD at the start of a trial*/
//     virtual void start_trial()
//     {
//         //bd.resize(ParamsDnn::dnn::nb_inputs + ParamsDnn::dnn::nb_outputs - 1, 0.0f); 
//         bd.resize(ParamsDnn::dnn::nb_inputs - 1, 0.0f); 
//     }
//     /*end the trial*/
//     virtual void end_trial()
//     {
//         rnn->forward();
//     }
//     /*summarise BD at the end of trials*/
//     virtual std::vector<float> after_trials(Real time);
// };

class AutoDescriptor: public Descriptor{
public:
    AutoDescriptor(){}

    /*after getting inputs, can update the descriptor if needed*/
    virtual void set_input_descriptor(CObsAvoidEvolLoopFunctions& cLoopFunctions){};

    /*after getting outputs, can update the descriptor if needed*/
    virtual void set_output_descriptor(CObsAvoidEvolLoopFunctions& cLoopFunctions){};

    /*summarise BD at the end of trial*/
    virtual std::vector<float> after_trials(Real time, CObsAvoidEvolLoopFunctions& cLoopFunctions){};
};
/****************************************/
/****************************************/
/* typedef function pointer, useful for fitness functions */
//typedef void (*functionPtr)();




