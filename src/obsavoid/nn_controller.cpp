#include <src/obsavoid/nn_controller.h>

/****************************************/
/****************************************/

CThymioNNController::CThymioNNController()
{
}

/****************************************/
/****************************************/

CThymioNNController::~CThymioNNController()
{
}

/****************************************/
/****************************************/

void CThymioNNController::Init(TConfigurationNode &t_node)
{
    /*
    * Get sensor/actuator handles
    */
    try
    {
        m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
        m_pcProximity = GetSensor<CCI_ThymioProximitySensor>("Thymio_proximity");
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing sensors/actuators", ex);
    }
}

/****************************************/
/****************************************/

void CThymioNNController::ControlStep()
{
    m_pcWheels->SetLinearVelocity(
        m_fLeftSpeed,
        m_fRightSpeed);
}

/****************************************/
/****************************************/

void CThymioNNController::Reset()
{
    //    m_cPerceptron.Reset();
}

/****************************************/
/****************************************/

void CThymioNNController::Destroy()
{
    //    m_cPerceptron.Destroy();
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CThymioNNController, "obsavoid_nn_controller")
