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
//    /* Get sensory data */
//    const CCI_FootBotProximitySensor::TReadings& tProx = m_pcProximity->GetReadings();
//    const CCI_FootBotLightSensor::TReadings& tLight = m_pcLight->GetReadings();
//    /* Fill NN inputs from sensory data */
//    for(size_t i = 0; i < tProx.size(); ++i)
//    {
//        m_cPerceptron.SetInput(i, tProx[i].Value);
//    }
//    for(size_t i = 0; i < tLight.size(); ++i)
//    {
//        m_cPerceptron.SetInput(tProx.size()+i, tLight[i].Value);
//    }
//    /* Compute NN outputs */
//    m_cPerceptron.ComputeOutputs();
//    /*
//    * Apply NN outputs to actuation
//    * The NN outputs are in the range [0,1]
//    * To allow for backtracking, we remap this range
//    * into [-5:5] linearly.
//    */
//    NN_OUTPUT_RANGE.MapValueIntoRange(
//                m_fLeftSpeed,               // value to write
//                m_cPerceptron.GetOutput(0), // value to read
//                WHEEL_ACTUATION_RANGE       // target range (here [-5:5])
//                );
//    NN_OUTPUT_RANGE.MapValueIntoRange(
//                m_fRightSpeed,              // value to write
//                m_cPerceptron.GetOutput(1), // value to read
//                WHEEL_ACTUATION_RANGE       // target range (here [-5:5])
//                );
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
