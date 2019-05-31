/* Include the controller definition */
#include "baseline-behavs.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/****************************************/
/****************************************/

CBaselineBehavs::CBaselineBehavs() :
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_pcGround(NULL),
    m_pcLeds(NULL),
    m_pcRABA(NULL),
    m_pcRABS(NULL),
    m_cAlpha(10.0f),
    m_fDelta(0.5f),
    m_fWheelVelocity(2.5f),
    m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                            ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CBaselineBehavs::Init(TConfigurationNode& t_node)
{
    /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><Thymio_diffusion><actuators> and
    * <controllers><Thymio_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcLeds      = GetActuator<CCI_ThymioLedsActuator          >("thymio_led");
    m_pcProximity = GetSensor  <CCI_ThymioProximitySensor       >("Thymio_proximity");
    m_pcGround    = GetSensor  <CCI_ThymioGroundSensor          >("Thymio_ground");
    m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing" );
    m_pcRABS      = GetSensor <CCI_RangeAndBearingSensor        >("range_and_bearing" );
    /*
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
    GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
    m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
    GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
    GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
}

/****************************************/
/****************************************/

void CBaselineBehavs::ControlStep()
{
    m_pcRABA->ClearData(); // clear the channel at the start of each control cycle

    m_pcRABA->SetData(0, 100); // send test-value of 100 on RAB medium

    /* Get readings from proximity sensor */
    const CCI_ThymioProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
    /* Get readings from ground sensor */
    const CCI_ThymioGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();

    m_pcLeds->SetProxHIntensity(tProxReads);

    //   LOG << tProxReads;
    //   LOG << tProxReads[2].Value<< tProxReads[2].Angle.GetValue();
    //   std::cout << tProxReads;

    m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);


    /* Sum them together */
    CVector2 cAccumulator;
    for(size_t i = 0; i < tProxReads.size(); ++i)
    {
        cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
    }
    cAccumulator /= tProxReads.size();

    short cground = 0;
    for(size_t i = 0; i < tGroundReads.size(); ++i)
    {
        cground += tGroundReads[i].Value;
    }

    /* If the angle of the vector is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    */
    CRadians cAngle = cAccumulator.Angle();
    LOG << cAngle.GetValue();
    if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
            cAccumulator.Length() < m_fDelta )
    {
        /* Go straight */
        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
    }
    else
    {
        /* Turn, depending on the sign of the angle */
        if(cAngle.GetValue() < 0)
        {
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0);
        }
        else
        {
            m_pcWheels->SetLinearVelocity(0, m_fWheelVelocity);
        }
    }

    CCI_RangeAndBearingSensor::TReadings sensor_readings = m_pcRABS->GetReadings();
    std::cout << "Robots in RAB range of " << GetId() << " is " << sensor_readings.size() << std::endl;
    for(size_t i = 0; i < sensor_readings.size(); ++i)
    {
        std::cout << "RAB range " << sensor_readings[i].Range << " Bearing "  << sensor_readings[i].HorizontalBearing << " Message size " << sensor_readings[i].Data.Size() << std::endl;
        for(size_t j = 0; j < sensor_readings[i].Data.Size(); ++j)
            std::cout << "Data-Packet at index " << j << " is " << sensor_readings[i].Data[j] << std::endl;
    }
}

CBaselineBehavs::~CBaselineBehavs()
{
    m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CBaselineBehavs, "baseline-behavs")
