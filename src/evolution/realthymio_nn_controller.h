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
#include "serialisation_functions.hpp"

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class RealThymioNN : public BaseController {

public:
   RealThymioNN();
   virtual ~RealThymioNN();

   virtual void ControlStep();
   virtual void Reset() {};
   virtual void Destroy(){};
   virtual void init_sensact(TConfigurationNode& t_node);
   void init_network();
    std::vector<Real> GetNormalizedSensorReadings();

    std::vector<float> inputs;
    /* get the inputs as a vector of floats*/
    std::vector<float> InputStep();
    robots_nn::nn_t nn;
};

#endif
