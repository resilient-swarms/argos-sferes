#ifndef BASE_CONTROLLER
#define BASE_CONTROLLER

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>

/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/thymio/control_interface/ci_thymio_leds_actuator.h>

/* Definition of proximity sensor */
#include <argos3/plugins/robots/thymio/control_interface/ci_thymio_proximity_sensor.h>

/* Definition of ground sensor */
#include <argos3/plugins/robots/thymio/control_interface/ci_thymio_ground_sensor.h>

/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>

/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

/* Definition of the differential steering wheel encoder */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_sensor.h>

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
class BaseController : public CCI_Controller
{

public:
  /* The possible faults on robot */
  enum FaultBehavior
  {
    FAULT_NONE = 0,

    /*faults whose effects cause one of the following four general failures */
    // FAULT_STRAIGHTLINE,
    // FAULT_RANDOMWALK,
    // FAULT_CIRCLE,
    // FAULT_STOP,

    /* random fault will be injected */
    FAULT_RANDOM,

    /* Implementing the faults themselves. The resulting behaviors will now depend on the normal behavior implementation. */
    FAULT_PROXIMITYSENSORS_SETMIN,
    FAULT_PROXIMITYSENSORS_SETMAX,
    FAULT_PROXIMITYSENSORS_SETRANDOM,
    FAULT_PROXIMITYSENSORS_SETOFFSET,

    FAULT_RABSENSOR_SETOFFSET,
    FAULT_RABSENSOR_MISSINGRECEIVERS,
    FAULT_RABSENSOR_HALFRANGE,
    FAULT_RABPACKETLOSS,
    FAULT_RABACTUATOR,

    FAULT_ACTUATOR_LWHEEL_SETHALF,
    FAULT_ACTUATOR_RWHEEL_SETHALF,
    FAULT_ACTUATOR_BWHEELS_SETHALF,

    // FAULT_SOFTWARE,

    // FAULT_POWER_FAILURE
  } FBehavior;

  struct SWheelTurningParams
  {
    Real MaxSpeed;
    void Init(TConfigurationNode &t_tree);
  } m_sWheelTurningParams;
public:
  BaseController();
  virtual ~BaseController();

  virtual void Init(TConfigurationNode &t_node);
  virtual void ControlStep();
  virtual void Reset();
  virtual void Destroy();


  //private:
public:
    /* Pointer to the LEDs */
    CCI_ThymioLedsActuator*   m_pcLeds;
    /* Pointer to the differential steering actuator */
    CCI_DifferentialSteeringActuator* m_pcWheels;
    /* Pointer to the differential steering encoder */
    CCI_DifferentialSteeringSensor* m_pcWheelsEncoder;
    /* Pointer to the Thymio proximity sensor */
    CCI_ThymioProximitySensor* m_pcProximity;
    /* Pointer to the Thymio ground sensors */
    CCI_ThymioGroundSensor* m_pcGround;
    /* Pointer to the range and bearing actuator */
    CCI_RangeAndBearingActuator*  m_pcRABA;
    /* Pointer to the range and bearing sensor */
    CCI_RangeAndBearingSensor* m_pcRABS;

    /* maximum RAB range */
    Real max_rab_range = 100.0; // in cm
    /* rab cones */
    std::vector<CRadians> rab_cones;
    /* Wheel speeds */
    Real m_fLeftSpeed, m_fRightSpeed;

    /* random number generator */
    CRandom::CRNG *m_pcRNG;

    /* whether or not the robot is damaged */
    bool b_damagedrobot;
    /* damage probability */
    float damage_probability=0.0f;
    
    /* whether or not to only use proximity sensors */
    bool only_proximity;

    std::vector<CCI_ThymioProximitySensor::SReading> GetIRSensorReadings(bool b_DamagedRobot, FaultBehavior fault_type);

    CCI_RangeAndBearingSensor::TReadings GetRABSensorReadings(bool b_DamagedRobot, FaultBehavior fault_type);

    std::vector<Real> GetNormalizedSensorReadings();

    void process_faultbehaviour(std::string errorbehav);
    void parse_perturbation_set(std::string error_behav);
    void damage_sensors(std::vector<float> &inputs);
    void damage_actuators();
    /* left wheel velocity normalised to [0,1]*/
    float left_wheel_velocity_01();
    /* right wheel velocity normalised to [0,1]*/
    float right_wheel_velocity_01();
    /* linear speed normalised to [0,1]*/
    float linear_speed_01();
    /* linear velocity normalised to [0,1]*/
    float linear_velocity_01();
    /* linear velocity normalised to [-1,1] */
    float linear_velocity_signed();
    /* turn speed normalised to [0,1]*/
    float turn_speed_01();

    virtual void init_sensact(TConfigurationNode &t_node);
    void init_fault_config(TConfigurationNode &t_node);

};
#endif
