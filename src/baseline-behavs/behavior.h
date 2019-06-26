#ifndef BEHAVIOR_H_
#define BEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include <vector>

/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/vector2.h>

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




/******************************************************************************/
/******************************************************************************/
class CBehavior;

typedef std::vector<CBehavior*>           TBehaviorVector;
typedef std::vector<CBehavior*>::iterator TBehaviorVectorIterator;


/******************************************************************************/
/******************************************************************************/

//class CEPuckForaging;
using namespace argos;

/******************************************************************************/
/******************************************************************************/

#define NO_TURN 0
#define SOFT_TURN 1
#define HARD_TURN 2

/******************************************************************************/
/******************************************************************************/

class CBehavior
{
public:
    CBehavior();
    virtual ~CBehavior();

    virtual void SimulationStep() = 0;

    virtual bool TakeControl() = 0;
    virtual void Suppress();
    virtual void Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

    virtual void PrintBehaviorIdentity();

    virtual void WheelSpeedsFromHeadingVector(CVector2 &m_cHeadingVector, Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

    struct RobotData
    {
        Real     MaxSpeed;
        Real     iterations_per_second;
        Real     INTERWHEEL_DISTANCE, HALF_INTERWHEEL_DISTANCE;
        Real     WHEEL_RADIUS;
        Real     seconds_per_iterations;
        CRadians m_cNoTurnOnAngleThreshold;
        CRadians m_cSoftTurnOnAngleThreshold;

        size_t   BEACON_SIGNAL_MARKER, NEST_BEACON_SIGNAL_MARKER;

        size_t   SELF_INFO_PACKET_MARKER;
        size_t   SELF_INFO_PACKET_FOOTER_MARKER;

        size_t   CHAIN_CONNECTOR_PACKET_MARKER;
        size_t   CHAIN_CONNECTOR_REQUEST_MARKER;
        size_t   CHAIN_CONNECTOR_REQUESTACCEPTED_MARKER;
        size_t   CHAIN_CONNECTOR_PACKET_FOOTER_MARKER;

        size_t   RELAY_PACKET_MARKER;
        size_t   RELAY_PACKET_FOOTER_MARKER;

        size_t   VOTER_PACKET_MARKER;
        size_t   VOTER_PACKET_FOOTER_MARKER;

        size_t   DATA_BYTE_BOUND_MARKER;
    };

    struct SensoryData
    {
       CRandom::CRNG* m_pcRNG;

       Real m_rTime;

       std::vector<CCI_ThymioProximitySensor::SReading> m_ProximitySensorData;
       std::vector<Real> m_GroundSensorData; //CCI_GroundSensor::TReadings
       CCI_RangeAndBearingSensor::TReadings  m_RABSensorData;

       Real f_LeftWheelSpeed, f_RightWheelSpeed;

       void SetSensoryData(CRandom::CRNG* rng, Real Time, std::vector<CCI_ThymioProximitySensor::SReading> proximity, std::vector<Real> ground,
                           CCI_RangeAndBearingSensor::TReadings rab)
       {
           m_pcRNG = rng;
           m_rTime = Time;
           m_ProximitySensorData = proximity;
           m_GroundSensorData = ground;
           m_RABSensorData = rab;
       }

       void SetSensoryData(CRandom::CRNG* rng, Real Time, std::vector<CCI_ThymioProximitySensor::SReading> proximity, CCI_RangeAndBearingSensor::TReadings rab)
       {
           m_pcRNG = rng;
           m_rTime = Time;
           m_ProximitySensorData = proximity;
           m_RABSensorData = rab;
       }

       void SetWheelSpeedsFromEncoders(Real LeftWheelSpeed, Real RightWheelSpeed)
       {
           f_LeftWheelSpeed = LeftWheelSpeed; f_RightWheelSpeed = RightWheelSpeed;
       }
    };

    static SensoryData m_sSensoryData;
    static RobotData m_sRobotData;

protected:
    //CEPuckForaging*   m_pcEPuck;
};

/******************************************************************************/
/******************************************************************************/

#endif

/******************************************************************************/
/******************************************************************************/
