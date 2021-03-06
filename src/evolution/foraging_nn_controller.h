#ifndef Thymio_NN_H
#define Thymio_NN_H

/*
 * Include some necessary headers.
 */
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/thymio/control_interface/ci_thymio_leds_actuator.h>
#include <argos3/plugins/robots/thymio/control_interface/ci_thymio_proximity_sensor.h>
#include <argos3/plugins/robots/thymio/control_interface/ci_thymio_ground_sensor.h>
#include <argos3/core/utility/logging/argos_log.h>

#include "src/core/base_controller.h"
#include "src/evolution/serialisation_functions.hpp"


/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
//using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class ForagingThymioNN : public BaseController
{

public:
#if HETEROGENEOUS 
    #ifndef RECORD_FIT  // only need worker when optimising
    struct Worker
    {
        bool initial_phase;
        size_t total_time;
        size_t max_trials, trials_completed;
        size_t index;
        size_t opt_index;
        Eigen::VectorXd new_sample, F;
        size_t numFoodCollected = 0;
        Worker() {}
        Worker(size_t trials, size_t i) : total_time(0), index(i), opt_index(0), initial_phase(true){};
        Worker(size_t trials, size_t i, size_t opt_idx) : total_time(0), index(i), opt_index(opt_idx), initial_phase(true){};

        /* at the end of trial, convert food collected to fitness consistent with map;
            also reset the number of foods collected for the next trial
         */
        double fitness(size_t num_workers)
        {
            return numFoodCollected * (float)num_workers;
        }
        /* get the sample with the identification */
        Eigen::VectorXd get_sample()
        {
            Eigen::VectorXd vec_joined(new_sample.size() + F.size());
            vec_joined << new_sample, F;
            return vec_joined;
        }

        // /* get the worker index; all 0 in the initial phase; not needed due to ID vector distinguishing them */
        // size_t get_index()
        // {
        //     return initial_phase ? 0 : index;
        // }
    } worker;
    int num_ticks_left;
    int num_trials_left;
    
    float collision_value = 0.0,  reward_value=0.0f;  // running average of collision (1/0)
    float maximum_performance;    // maximum performance (avg over trials) so far for all controllers
    float norm_trial_performance; // performance of normal controller on single trial
    bool collision_stop()
    {
        if(worker.initial_phase)
        {
            return false;// always finish complete identif. phase
        }
        float collision_thresh = 0.90;
        return collision_value > collision_thresh;
    }
    bool reward_stop()
    {
        // float collision_thresh = 0.80;
        // return cController.reward_value < reward_thresh;
        return false; //not yet implemented
    }
    void reset_stopvals()
    {
        collision_value=0.0f;
        reward_value=0.0f;
    }
    void select_net(std::vector<double> bd, size_t num_subtrials, size_t ticks_per_subtrial);
    #endif
    void select_net(std::vector<double> bd);
    void select_net(size_t ctr_index);

    int trial;
#endif
    
    bool holdingFood = false;
    int foodID = -1; // index used to track the "SOFTWARE_FOOD" fault
    std::string savefile;
    ForagingThymioNN();
    virtual ~ForagingThymioNN();

    virtual void ControlStep();
    virtual void Reset()
    {
        holdingFood = false;
#if HETEROGENEOUS & !RECORD_FIT
        worker.numFoodCollected = 0;
#endif
    };
    virtual void Destroy(){};
    virtual void init_sensact(argos::TConfigurationNode &t_node);
    virtual void init_fault_config(TConfigurationNode &t_node);
    void init_network();
    std::vector<Real> GetNormalizedSensorReadings();

    std::vector<float> inputs;
    /* get the inputs as a vector of floats*/
    std::vector<float> InputStep();
    robots_nn::nn_t nn;
};

#endif
