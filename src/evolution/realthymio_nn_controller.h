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

    struct SWheelTurningParams
    {
        Real MaxSpeed;
        void Init(TConfigurationNode &t_tree);
    } m_sWheelTurningParams;

public:
   RealThymioNN();
   virtual ~RealThymioNN();

   virtual void Init(TConfigurationNode& t_node);
   virtual void ControlStep();
   virtual void Reset() {};
   virtual void Destroy(){};
   virtual void init_sensact();


   /* Pointer to the LEDs */
    CCI_ThymioLedsActuator*   m_pcLeds;
   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the Thymio proximity sensor */
   CCI_ThymioProximitySensor* m_pcProximity;
   /* Pointer to the Thymio ground sensors */
   CCI_ThymioGroundSensor* m_pcGround;

    /* Wheel speeds */
    Real m_fLeftSpeed, m_fRightSpeed;

    std::vector<Real> GetNormalizedSensorReadings();

    std::vector<float> inputs;
    /* get the inputs as a vector of floats*/
    std::vector<float> InputStep();
    robots_nn::nn_t nn;
};

#endif