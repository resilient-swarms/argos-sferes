#ifndef NN_CONTROLLER
#define NN_CONTROLLER

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the Thymio proximity sensor */
#include <argos3/plugins/robots/thymio/control_interface/ci_thymio_proximity_sensor.h>


/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;



/*
 * A controller is simply an implementation of the CCI_Controller class.
 * In this case, we also inherit from the CPerceptron class. We use
 * virtual inheritance so that matching methods in the CCI_Controller
 * and CPerceptron don't get messed up.
 */
class CThymioNNController : public CCI_Controller
{
          /* The possible faults on robot */
    enum FaultBehavior
    {
        FAULT_NONE = 0,

        // /*faults whose effects cause one of the following four general failures */
        // FAULT_STRAIGHTLINE,
        // FAULT_RANDOMWALK,
        // FAULT_CIRCLE,
        // FAULT_STOP,

        /* Implementing the faults themselves. The resulting behaviors will now depend on the normal behavior implementation. */
        FAULT_PROXIMITYSENSORS_SETMIN,
        FAULT_PROXIMITYSENSORS_SETMAX,
        FAULT_PROXIMITYSENSORS_SETRANDOM,
        FAULT_PROXIMITYSENSORS_SETOFFSET,

        // FAULT_RABSENSOR_SETOFFSET,
        // FAULT_RABSENSOR_MISSINGRECEIVERS,

        FAULT_ACTUATOR_LWHEEL_SETZERO,
        FAULT_ACTUATOR_RWHEEL_SETZERO,
        FAULT_ACTUATOR_BWHEELS_SETZERO,

        // FAULT_SOFTWARE,

        // FAULT_POWER_FAILURE
    } FBehavior;

  public:
    CThymioNNController();
    virtual ~CThymioNNController();
    /* get the inputs */
    std::vector<float> InputStep();
    void Init(TConfigurationNode &t_node);
    void ControlStep();
    void Reset();
    void Destroy();

    //private:
  public:
    /* Pointer to the differential steering actuator */
    CCI_DifferentialSteeringActuator *m_pcWheels;
    /* Pointer to the foot-bot proximity sensor */
    CCI_ThymioProximitySensor *m_pcProximity;

    /* Wheel speeds */
    Real m_fLeftSpeed, m_fRightSpeed;

    void process_faultbehaviour(std::string errorbehav);
    void damage_sensors(std::vector<float> inputs);
    void damage_actuators();




    size_t id_FaultyRobotInSwarm;

    CRandom::CRNG* m_pcRNG;

};
#endif
