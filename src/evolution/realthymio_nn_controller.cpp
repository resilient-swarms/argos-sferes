/* Include the controller definition */
#include "realthymio_nn_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */


/****************************************/
/****************************************/

RealThymioNN::RealThymioNN()
{
}


/****************************************/
/****************************************/



void RealThymioNN::init_network()
{
    NNSerialiser ser = NNSerialiser();
    nn = ser.Load();
    nn.init();
}
/****************************************/
/****************************************/

void RealThymioNN::ControlStep()
{
    /* Get readings from proximity sensor */
    const CCI_ThymioProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
    /* Get readings from ground sensor */
    //const CCI_ThymioGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();

    m_pcLeds->SetProxHIntensity(tProxReads);

    inputs = InputStep();
    nn.step(inputs);

    m_fLeftSpeed = m_sWheelTurningParams.MaxSpeed * nn.get_outf()[0];
    m_fRightSpeed = m_sWheelTurningParams.MaxSpeed * nn.get_outf()[1];

    BaseController::ControlStep();// needed to actually move and inject faults
}

RealThymioNN::~RealThymioNN()
{
    m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
}

std::vector<float> RealThymioNN::InputStep()
{
    std::vector<Real> readings = GetNormalizedSensorReadings();
    std::vector<float> in = std::vector<float>(readings.begin(), readings.end());
    in.push_back(+1.0); //Bias input
    //std::cout << in.size() << std::endl;
    return in;
}

std::vector<Real> RealThymioNN::GetNormalizedSensorReadings()
{

    std::vector<Real> norm_readings;
    /* Get readings from proximity sensor */
    const CCI_ThymioProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
    for(UInt8 i = 0; i < tProxReads.size(); ++i)
        norm_readings.push_back((1.0f - tProxReads[i].Value) * 2.0f - 1.0f);

    /* Get readings from ground sensor */
    const CCI_ThymioGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();
    for(UInt8 i = 0; i < tGroundReads.size(); ++i) {
        norm_readings.push_back((tGroundReads[i].Value/1000) * 2.0f - 1.0f);
	//std::cout << (tGroundReads[i].Value/1000) *2.0f -1.0f<< std::endl;
    }
    return norm_readings;
}


void RealThymioNN::init_sensact(TConfigurationNode& t_node)
{
    /*
    * Get sensor/actuator handles
    */
    try
    {
        m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
        m_pcLeds = GetActuator<CCI_ThymioLedsActuator>("thymio_led");
        m_pcProximity = GetSensor<CCI_ThymioProximitySensor>("Thymio_proximity");
        m_pcGround = GetSensor<CCI_ThymioGroundSensor>("Thymio_ground");
 
    }

    catch (CARGoSException &ex2){
        THROW_ARGOSEXCEPTION_NESTED("Error initializing sensors/actuators", ex2);
    }
}
/****************************************/
/****************************************/

REGISTER_CONTROLLER(RealThymioNN, "realthymio_nn_controller")
