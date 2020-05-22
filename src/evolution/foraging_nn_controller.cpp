/* Include the controller definition */
#include <src/evolution/foraging_nn_controller.h>
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */

#if HETEROGENEOUS & !PRINT_NETWORK

#include <src/ite/ite_swarms.hpp>

#endif

/****************************************/
/****************************************/

ForagingThymioNN::ForagingThymioNN()
{
}

/****************************************/
/****************************************/

void ForagingThymioNN::init_network()
{
    NNSerialiser ser = NNSerialiser();
    nn = ser.Load();
    nn.init();
}

#if HETEROGENEOUS & !PRINT_NETWORK

void ForagingThymioNN::select_net(std::vector<double> bd, size_t num_subtrials, size_t ticks_per_subtrial, bool init)
{
    controller_index = print_individual_to_network(bd, Params::archiveparams::archive);
    std::cout << "select controller " << controller_index << std::endl;
    init_network();
    num_trials_left = num_subtrials;
    num_ticks_left = ticks_per_subtrial;
}

#endif

/****************************************/
/****************************************/

void ForagingThymioNN::ControlStep()
{

    inputs = InputStep();
    nn.step(inputs);

    m_fLeftSpeed = m_sWheelTurningParams.MaxSpeed * nn.get_outf()[0];
    m_fRightSpeed = m_sWheelTurningParams.MaxSpeed * nn.get_outf()[1];

    BaseController::ControlStep(); // needed to actually move and inject faults
}

ForagingThymioNN::~ForagingThymioNN()
{
    m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
}

std::vector<float> ForagingThymioNN::InputStep()
{
    std::vector<Real> readings = GetNormalizedSensorReadings();
    std::vector<float> in = std::vector<float>(readings.begin(), readings.end());
    in.push_back(+1.0); //Bias input
    //std::cout << in.size() << std::endl;
    return in;
}

std::vector<Real> ForagingThymioNN::GetNormalizedSensorReadings()
{

    std::vector<Real> norm_readings;
    /* Get readings from proximity sensor */
    const argos::CCI_ThymioProximitySensor::TReadings &tProxReads = this->GetIRSensorReadings(b_damagedrobot, FBehavior);
    /* Get readings from ground sensor */
    const CCI_ThymioGroundSensor::TReadings &tGroundReads = this->GetGroundSensorReadings(b_damagedrobot, FBehavior);

#ifdef PRINTING
    std::cout << "Prox read " << tProxReads << std::endl;
    std::cout << "Ground read " << tGroundReads << std::endl;

#endif
    //m_pcLeds->SetProxHIntensity(tProxReads);// don't need it; maybe useful for debugging
    for (UInt8 i = 0; i < tProxReads.size(); ++i)
        norm_readings.push_back((1.0f - tProxReads[i].Value) * 2.0f - 1.0f);

    for (UInt8 i = 0; i < tGroundReads.size(); ++i)
    {
        float norm_reading = (tGroundReads[i].Value / 255.) * 2.0f - 1.0f;
        norm_reading = std::min(1.0f, std::max(-1.0f, norm_reading));
        norm_readings.push_back(norm_reading);
#ifdef PRINTING

        std::cout << "norm reading " << i << ": " << norm_reading << std::endl;
#endif
    }
    return norm_readings;
}

void ForagingThymioNN::init_sensact(argos::TConfigurationNode &t_node)
{
    /*
    * Get sensor/actuator handles
    */
    try
    {
        m_pcWheels = GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering");
        //m_pcLeds = GetActuator<argos::CCI_ThymioLedsActuator>("thymio_led");
        m_pcProximity = GetSensor<argos::CCI_ThymioProximitySensor>("Thymio_proximity");
        m_pcGround = GetSensor<argos::CCI_ThymioGroundSensor>("Thymio_ground");
    }

    catch (argos::CARGoSException &ex2)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing sensors/actuators", ex2);
    }
}

void ForagingThymioNN::init_fault_config(TConfigurationNode &t_node)
{
    /* Experiment to run */
    TConfigurationNode sub_node = GetNode(t_node, "experiment_run");
    std::string food_id;
    try
    {
        GetNodeAttribute(sub_node, "id_food_source", food_id);
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing type of experiment to run, and fault to simulate.", ex);
    }
    foodID = std::stoi(food_id);
    BaseController::init_fault_config(t_node);
}
/****************************************/
/****************************************/

REGISTER_CONTROLLER(ForagingThymioNN, "foraging_nn_controller" + std::string(TAG))
