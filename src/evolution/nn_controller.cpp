#include <src/evolution/nn_controller.h>

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
    * Create the random number generator
    */
    m_pcRNG = CRandom::CreateRNG("argos");
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

    /* Experiment to run */
    TConfigurationNode sub_node = GetNode(t_node, "faults");
    std::string errorbehav;
    try
    {
        GetNodeAttribute(sub_node, "fault_behavior", errorbehav);
        GetNodeAttribute(sub_node, "id_faulty_robot", id_FaultyRobotInSwarm);
    }
    catch(CARGoSException& ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing type of experiment to run, and fault to simulate.", ex);
    }

    process_faultbehaviour(errorbehav);
}

/****************************************/
/****************************************/
std::vector<float> CThymioNNController::InputStep()
{
    
    std::vector<float> in;
    for (size_t i = 0; i < m_pcProximity->GetReadings().size(); ++i)
    {
        in.push_back(m_pcProximity->GetReadings()[i].Value);
    }

    return in;
}
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



void CThymioNNController::process_faultbehaviour(std::string errorbehav)
{

    if (errorbehav.compare("FAULT_NONE") == 0)
        FBehavior = FAULT_NONE;
    // else if  (errorbehav.compare("FAULT_STRAIGHTLINE") == 0)
    //     FBehavior = FAULT_STRAIGHTLINE;
    // else if  (errorbehav.compare("FAULT_RANDOMWALK") == 0)
    //     FBehavior = FAULT_RANDOMWALK;
    // else if  (errorbehav.compare("FAULT_CIRCLE") == 0)
    //     FBehavior = FAULT_CIRCLE;
    // else if  (errorbehav.compare("FAULT_STOP") == 0)
    //     FBehavior = FAULT_STOP;
    else if  (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETMIN") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETMIN;
    else if  (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETMAX") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETMAX;
    else if  (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETRANDOM") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETRANDOM;
    else if  (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETOFFSET") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETOFFSET;
    // else if  (errorbehav.compare("FAULT_RABSENSOR_SETOFFSET") == 0)
    //     FBehavior = FAULT_RABSENSOR_SETOFFSET;
    // else if  (errorbehav.compare("FAULT_RABSENSOR_MISSINGRECEIVERS") == 0)
    //     FBehavior = FAULT_RABSENSOR_MISSINGRECEIVERS;
    else if  (errorbehav.compare("FAULT_ACTUATOR_LWHEEL_SETHALF") == 0)
        FBehavior = FAULT_ACTUATOR_LWHEEL_SETHALF;
    else if  (errorbehav.compare("FAULT_ACTUATOR_RWHEEL_SETHALF") == 0)
        FBehavior = FAULT_ACTUATOR_RWHEEL_SETHALF;
    else if  (errorbehav.compare("FAULT_ACTUATOR_BWHEELS_SETHALF") == 0)
        FBehavior = FAULT_ACTUATOR_BWHEELS_SETHALF;

    // else if  (errorbehav.compare("FAULT_SOFTWARE") == 0)
    //     FBehavior = FAULT_SOFTWARE;

    // else if  (errorbehav.compare("FAULT_POWER_FAILURE") == 0)
    //     FBehavior = FAULT_POWER_FAILURE;

    else
    {
        std::cerr << "invalid fault behavior";
        assert(-1);
    }
}

void CThymioNNController::damage_sensors(std::vector<float>& inputs)
{

        if (FBehavior == FaultBehavior::FAULT_PROXIMITYSENSORS_SETMIN)
        {
            /* Front IR sensors */
            inputs[0] = 0.0f;
            inputs[1] = 0.0f;
            inputs[2] = 0.0f;
            inputs[3] = 0.0f;
            inputs[4] = 0.0f;
            return;
        }
        else if (FBehavior == FaultBehavior::FAULT_PROXIMITYSENSORS_SETMAX)
        {
            /* Front  IR sensors */
            inputs[0] = 1.0f;
            inputs[1] = 1.0f;
            inputs[2] = 1.0f;
            inputs[3] = 1.0f;
            inputs[4] = 1.0f;
            return;
        }
        else if (FBehavior == FaultBehavior::FAULT_PROXIMITYSENSORS_SETRANDOM)
        {
            /* Front four IR sensors */
            inputs[0] = m_pcRNG->Uniform(CRange<Real>(0.0f, 1.0f));
            inputs[1] = m_pcRNG->Uniform(CRange<Real>(0.0f, 1.0f));
            inputs[2] = m_pcRNG->Uniform(CRange<Real>(0.0f, 1.0f));
            inputs[3] = m_pcRNG->Uniform(CRange<Real>(0.0f, 1.0f));
            inputs[4] = m_pcRNG->Uniform(CRange<Real>(0.0f, 1.0f));
            return;
        }
        else if (FBehavior == FaultBehavior::FAULT_PROXIMITYSENSORS_SETOFFSET)
        {
            /* Front four IR sensors */
            inputs[0] += m_pcRNG->Uniform(CRange<Real>(-0.5f, 0.5f));
            inputs[1] += m_pcRNG->Uniform(CRange<Real>(-0.5f, 0.5f));
            inputs[2] += m_pcRNG->Uniform(CRange<Real>(-0.5f, 0.5f));
            inputs[3] += m_pcRNG->Uniform(CRange<Real>(-0.5f, 0.5f));
            inputs[4] += m_pcRNG->Uniform(CRange<Real>(-0.5f, 0.5f));
            if (inputs[0] > 1.0f)
                inputs[0] = 1.0f;
            if (inputs[0] < 0.0f)
                inputs[0] = 0.0f;

            if (inputs[1] > 1.0f)
                inputs[1] = 1.0f;
            if (inputs[1] < 0.0f)
                inputs[1] = 0.0f;

            if (inputs[2] > 1.0f)
                inputs[2] = 1.0f;
            if (inputs[2] < 0.0f)
                inputs[2] = 0.0f;

            if (inputs[3] > 1.0f)
                inputs[3] = 1.0f;
            if (inputs[3] < 0.0f)
                inputs[3] = 0.0f;

            if (inputs[4] > 1.0f)
                inputs[4] = 1.0f;
            if (inputs[4] < 0.0f)
                inputs[4] = 0.0f;
            return;
        }
        else
        {
            /* the robot is running one of the general faults or one of the specific faults that doesnot influence IR sensor readings*/
            return;
        }
}

void CThymioNNController::damage_actuators()
{
        if (FBehavior == FaultBehavior::FAULT_ACTUATOR_LWHEEL_SETHALF)
            m_fLeftSpeed *= 0.5;

        if (FBehavior == FaultBehavior::FAULT_ACTUATOR_RWHEEL_SETHALF)
            m_fRightSpeed  *= 0.5;

        if (FBehavior == FaultBehavior::FAULT_ACTUATOR_BWHEELS_SETHALF)
        {
            m_fLeftSpeed  *= 0.5;
            m_fRightSpeed  *= 0.5;
        }
}
/****************************************/
/****************************************/

// don't register, use perturb instead, more general
REGISTER_CONTROLLER(CThymioNNController, "nn_controller")
