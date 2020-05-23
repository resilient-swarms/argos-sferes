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
class ForagingThymioNN : public BaseController {

public:
#if  HETEROGENEOUS & !PRINT_NETWORK
    struct Worker
    {
        bool initial_phase;
        size_t max_trials, trials_completed;
        size_t index;
        Eigen::VectorXd new_sample, F;
        size_t numFoodCollected = 0;
        Worker(){}
        Worker(size_t trials,size_t i) : max_trials(trials), trials_completed(0), index(i), initial_phase(true){};
        /* call from outside limbo */
        void finish_trial()
        {
            ++trials_completed;
        }
        /* call inside the optimization_step if a worker has finished trial */
        bool reset()
        {
            if (trials_completed == max_trials)
            {
                trials_completed = 0;
                return true;
            }
            else
            {
                return false;
            }
        }
        /* convert food collected to fitness consistent with map */
        double fitness(size_t num_workers)
        {
            return numFoodCollected*(float) num_workers;
        }
        /* get the sample with the identification */
        Eigen::VectorXd get_sample()
        {
            Eigen::VectorXd vec_joined(new_sample.size() + F.size());
            vec_joined << new_sample, F;
            return vec_joined;
        }

        /* get the worker index; all 0 in the initial phase */
        size_t get_index()
        {
            return initial_phase ? 0 : index;
        }
    } worker;
    std::string controller_index;
    int num_ticks_left;
    int num_trials_left;
    
    void select_net(std::vector<double> bd,size_t num_subtrials, size_t ticks_per_subtrial, bool init=false);
#endif
   bool  holdingFood=false;
   int foodID=-1;// index used to track the "SOFTWARE_FOOD" fault

   
   ForagingThymioNN();
   virtual ~ForagingThymioNN();

   virtual void ControlStep();
   virtual void Reset() {
        holdingFood=false;
#if HETEROGENEOUS & !PRINT_NETWORK
        worker.numFoodCollected = 0;
#endif
    };
   virtual void Destroy(){};
   virtual void init_sensact(argos::TConfigurationNode& t_node);
   virtual void init_fault_config(TConfigurationNode &t_node);
   void init_network();
    std::vector<Real> GetNormalizedSensorReadings();

    std::vector<float> inputs;
    /* get the inputs as a vector of floats*/
    std::vector<float> InputStep();
    robots_nn::nn_t nn;
};

#endif
