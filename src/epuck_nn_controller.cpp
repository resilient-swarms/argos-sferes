#include "epuck_nn_controller.h"

/****************************************/
/****************************************/

CEPuckNNController::CEPuckNNController()
{
}

/****************************************/
/****************************************/

CEPuckNNController::~CEPuckNNController()
{
}

/****************************************/
/****************************************/

void CEPuckNNController::Init(TConfigurationNode& t_node)
{
    /*
    * Get sensor/actuator handles
    */
    try
    {
        m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
        m_pcProximity = GetSensor  <CCI_ProximitySensor      >("proximity"    );
    }
    catch(CARGoSException& ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing sensors/actuators", ex);
    }


}

/****************************************/
/****************************************/

void CEPuckNNController::ControlStep()
{
    m_pcWheels->SetLinearVelocity(
                m_fLeftSpeed,
                m_fRightSpeed);
}

/****************************************/
/****************************************/

void CEPuckNNController::Reset()
{
//    m_cPerceptron.Reset();
}

/****************************************/
/****************************************/

void CEPuckNNController::Destroy()
{
//    m_cPerceptron.Destroy();
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CEPuckNNController, "epuck_nn_controller")
