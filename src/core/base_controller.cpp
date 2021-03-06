#include <src/core/base_controller.h>
#include <fstream>

/****************************************/
/****************************************/

BaseController::BaseController() : m_pcWheels(NULL),
                                   m_pcProximity(NULL),
                                   m_pcGround(NULL),
                                   m_pcLeds(NULL),
                                   m_pcRABA(NULL),
                                   m_pcRABS(NULL),
                                   m_pcRNG(CRandom::CreateRNG("argos")),
                                   b_damagedrobot(false)
{
}

/****************************************/
/****************************************/

BaseController::~BaseController()
{
}

/****************************************/
/****************************************/

void BaseController::SWheelTurningParams::Init(TConfigurationNode &t_node)
{
    try
    {
        GetNodeAttribute(t_node, "max_speed", MaxSpeed); //why is this not getting a value from argos config file?
    }
    catch (CARGoSException &ex)
        THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);

    //std::cout << " MaxSpeed " << MaxSpeed << std::endl;
}

void BaseController::Init(TConfigurationNode &t_node)
{
    //std::cout<<"base controller"<<std::endl;
    /*
    * Create the random number generator
    */
    m_pcRNG = CRandom::CreateRNG("argos");

    init_sensact(t_node);
    /* Wheel turning */
    m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
    Reset();

    init_fault_config(t_node);
}

void BaseController::parse_perturbation_set(std::string filename)
{
    std::string line;
    //open file
    filename.erase(0, std::string("FILE:").length()); // strip the prefix
    std::ifstream file(filename);
    //get the line
    if (file.good())
    {
        std::getline(file, line);
    }
    file.close();
    //
    std::stringstream ss(line);
    std::vector<std::string> result;

    while (ss.good())
    {
        std::string substr;
        std::getline(ss, substr, ',');
        result.push_back(substr);
    }
    // get the fault for this controller
    std::string id = this->GetId();
    id.erase(0, std::string("thymio").length());
    size_t index = std::stoi(id);
    process_faultbehaviour(result[index]);
    b_damagedrobot = true;
}
/****************************************/
/****************************************/

void BaseController::ControlStep()
{
    if (b_damagedrobot)
    {
        damage_actuators();
    }

    m_pcWheels->SetLinearVelocity(
        m_fLeftSpeed,
        m_fRightSpeed);
}

/****************************************/
/****************************************/

void BaseController::Reset()
{
    //    m_cPerceptron.Reset();
    if (damage_probability > 0.0f && m_pcRNG->Uniform(argos::CRange<Real>(0.0, 1.0)) < damage_probability)
    {
        b_damagedrobot = true;
    }
}

/****************************************/
/****************************************/

void BaseController::Destroy()
{
    //    m_cPerceptron.Destroy();
}

void BaseController::process_faultbehaviour(std::string errorbehav)
{

    if (errorbehav.compare("FAULT_NONE") == 0)
    {
        FBehavior = FAULT_NONE;
    }
    else if (errorbehav.compare("FAULT_SOFTWARE") == 0)
    {
        FBehavior = FAULT_SOFTWARE;
    }
    else if (errorbehav.compare("FAULT_SOFTWARE_FOOD") == 0)
    {
        FBehavior = FAULT_SOFTWARE_FOOD;
    }
    else if (errorbehav.compare("FAULT_SOFTWARE_NEIGHBOURHOOD") == 0)
    {
        FBehavior = FAULT_SOFTWARE_NEIGHBOURHOOD;
    }
    else if (errorbehav.compare("FAULT_FOOD_SCARCITY") == 0)
    {
        FBehavior = FAULT_FOOD_SCARCITY;
    }

    // these are not supported yet for the NN controller
    // else if  (errorbehav.compare("FAULT_STRAIGHTLINE") == 0)
    //     FBehavior = FAULT_STRAIGHTLINE;
    // else if  (errorbehav.compare("FAULT_RANDOMWALK") == 0)
    //     FBehavior = FAULT_RANDOMWALK;
    // else if  (errorbehav.compare("FAULT_CIRCLE") == 0)
    //     FBehavior = FAULT_CIRCLE;
    // else if  (errorbehav.compare("FAULT_STOP") == 0)
    //     FBehavior = FAULT_STOP;

    // random fault
    else if (errorbehav.compare("FAULT_RANDOM") == 0)
    {
        FBehavior = FAULT_RANDOM;
    }
    // proximity sensor faults
    else if (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETMIN") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETMIN;
    else if (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETMAX") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETMAX;
    else if (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETRANDOM") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETRANDOM;
    // else if  (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETOFFSET") == 0)
    //     FBehavior = FAULT_PROXIMITYSENSORS_SETOFFSET;

    // proximity sensor faults
    else if (errorbehav.compare("FAULT_GROUNDSENSORS_SETMIN") == 0)
        FBehavior = FAULT_GROUNDSENSORS_SETMIN;
    else if (errorbehav.compare("FAULT_GROUNDSENSORS_SETMAX") == 0)
        FBehavior = FAULT_GROUNDSENSORS_SETMAX;
    else if (errorbehav.compare("FAULT_GROUNDSENSORS_SETRANDOM") == 0)
        FBehavior = FAULT_GROUNDSENSORS_SETRANDOM;

    // RAB sensor faults
    else if (errorbehav.compare("FAULT_RABSENSOR_SETOFFSET") == 0)
        FBehavior = FAULT_RABSENSOR_SETOFFSET;
    else if (errorbehav.compare("FAULT_RABSENSOR_MISSINGRECEIVERS") == 0)
        FBehavior = FAULT_RABSENSOR_MISSINGRECEIVERS;
    else if (errorbehav.compare("FAULT_RABSENSOR_HALFRANGE") == 0) // affected robots half the range RAB sensors
        FBehavior = FAULT_RABSENSOR_HALFRANGE;
    else if (errorbehav.compare("FAULT_RABPACKETLOSS") == 0) // all the robots cannot receive any RAB info
        FBehavior = FAULT_RABPACKETLOSS;
    //RAB actuator faults
    else if (errorbehav.compare("FAULT_RABACTUATOR") == 0)
        FBehavior = FAULT_RABACTUATOR;

    else if (errorbehav.compare("FAULT_ACTUATOR_LWHEEL_SETHALF") == 0)
        FBehavior = FAULT_ACTUATOR_LWHEEL_SETHALF;
    else if (errorbehav.compare("FAULT_ACTUATOR_RWHEEL_SETHALF") == 0)
        FBehavior = FAULT_ACTUATOR_RWHEEL_SETHALF;
    else if (errorbehav.compare("FAULT_ACTUATOR_BWHEELS_SETHALF") == 0)
        FBehavior = FAULT_ACTUATOR_BWHEELS_SETHALF;

    // else if  (errorbehav.compare("FAULT_SOFTWARE") == 0)
    //     FBehavior = FAULT_SOFTWARE;

    // else if  (errorbehav.compare("FAULT_POWER_FAILURE") == 0)
    //     FBehavior = FAULT_POWER_FAILURE;
    else if (errorbehav.rfind("FILE:", 0) == 0) // process perturbation set
    {

        parse_perturbation_set(errorbehav);
    }
    else
    {
        std::cerr << "invalid fault behavior";
        assert(-1);
    }
}

void BaseController::damage_actuators()
{
    if (FBehavior == FaultBehavior::FAULT_ACTUATOR_LWHEEL_SETHALF)
        m_fLeftSpeed *= 0.5;

    else if (FBehavior == FaultBehavior::FAULT_ACTUATOR_RWHEEL_SETHALF)
        m_fRightSpeed *= 0.5;

    else if (FBehavior == FaultBehavior::FAULT_ACTUATOR_BWHEELS_SETHALF)
    {
        m_fLeftSpeed *= 0.5;
        m_fRightSpeed *= 0.5;
    }
    else if (FBehavior == FaultBehavior::FAULT_SOFTWARE)
    {
        float r = m_pcRNG->Uniform(CRange<Real>(0.0, 1.0));
        if (r < 0.05)
        {
            software_sign *= -1.0f;
        }
        m_fLeftSpeed = software_sign * m_sWheelTurningParams.MaxSpeed;
        m_fRightSpeed = software_sign * m_sWheelTurningParams.MaxSpeed;
    }
    else if (FBehavior == FaultBehavior::FAULT_SOFTWARE_FOOD || FBehavior == FaultBehavior::FAULT_SOFTWARE_NEIGHBOURHOOD)
    {
        m_fLeftSpeed = 0.0f;
        m_fRightSpeed = 0.0f;
    }
    else
    {
        return;
    }
}

std::vector<CCI_ThymioProximitySensor::SReading> BaseController::GetIRSensorReadings(bool b_DamagedRobot, FaultBehavior fault_type)
{
    std::vector<CCI_ThymioProximitySensor::SReading> sensor_readings = m_pcProximity->GetReadings();

    if (!b_DamagedRobot)
        return sensor_readings;

    if (fault_type == FaultBehavior::FAULT_PROXIMITYSENSORS_SETMIN)
    {
        /* Front IR sensors */
        for (size_t i = 0; i < 5; ++i)
            sensor_readings[i].Value = 0.0f;
        return sensor_readings;
    }
    else if (fault_type == FaultBehavior::FAULT_PROXIMITYSENSORS_SETMAX)
    {
        /* Front IR sensors */
        for (size_t i = 0; i < 5; ++i)
            sensor_readings[i].Value = 1.0f;
        return sensor_readings;
    }
    else if (fault_type == FaultBehavior::FAULT_PROXIMITYSENSORS_SETRANDOM)
    {
        /* Front IR sensors */
        for (size_t i = 0; i < 5; ++i)
            sensor_readings[i].Value = m_pcRNG->Uniform(CRange<Real>(0.0f, 1.0f));
        return sensor_readings;
    }
    else if (fault_type == FaultBehavior::FAULT_PROXIMITYSENSORS_SETOFFSET)
    {
        /* Front IR sensors */
        for (size_t i = 0; i < 5; ++i)
        {
            sensor_readings[i].Value += m_pcRNG->Uniform(CRange<Real>(-0.5f, 0.5f));
            if (sensor_readings[i].Value > 1.0f)
                sensor_readings[i].Value = 1.0f;
            if (sensor_readings[i].Value < 0.0f)
                sensor_readings[i].Value = 0.0f;
        }
        return sensor_readings;
    }

    else
    {
        /* the robot is running one of the general faults or one of the specific faults that doesnot influence IR sensor readings*/
        return sensor_readings;
    }
}

CCI_ThymioGroundSensor::TReadings BaseController::GetGroundSensorReadings(bool b_DamagedRobot, FaultBehavior fault_type)
{
    CCI_ThymioGroundSensor::TReadings sensor_readings = m_pcGround->GetReadings();
    if (!b_DamagedRobot)
        return sensor_readings;

    if (fault_type == FaultBehavior::FAULT_GROUNDSENSORS_SETMIN)
    {
        /* Front IR sensors */
        for (size_t i = 0; i < 2; ++i)
            sensor_readings[i].Value = 0.0f;
        return sensor_readings;
    }
    else if (fault_type == FaultBehavior::FAULT_GROUNDSENSORS_SETMAX)
    {
        /* Front IR sensors */
        for (size_t i = 0; i < 2; ++i)
            sensor_readings[i].Value = 255.0f;
        return sensor_readings;
    }
    else if (fault_type == FaultBehavior::FAULT_GROUNDSENSORS_SETRANDOM)
    {
        /* Front IR sensors */
        for (size_t i = 0; i < 2; ++i)
            sensor_readings[i].Value = m_pcRNG->Uniform(CRange<Real>(0.0f, 255.0f));
        return sensor_readings;
    }
    else
    {
        /* the robot is running one of the general faults or one of the specific faults that doesnot influence IR sensor readings*/
        return sensor_readings;
    }
}

CCI_RangeAndBearingSensor::TReadings BaseController::GetRABSensorReadings(bool b_DamagedRobot, FaultBehavior fault_type)
{
    CCI_RangeAndBearingSensor::TReadings sensor_readings = m_pcRABS->GetReadings();

    if (!b_DamagedRobot)
        return sensor_readings;

    if (fault_type == FaultBehavior::FAULT_RABSENSOR_SETOFFSET)
    {
        for (size_t i = 0; i < sensor_readings.size(); ++i)
        {
            //std::cout<<"range before:"<<sensor_readings[i].Range<<std::endl;
            //std::cout<<"horizontal bearing before:"<<sensor_readings[i].HorizontalBearing<<std::endl;
            CVector2 tmp(sensor_readings[i].Range, sensor_readings[i].HorizontalBearing);
            //std::cout<<"tmp variable: ("<<tmp.GetX()<<","<<tmp.GetY()<<")"<<std::endl;
            tmp += CVector2(m_pcRNG->Uniform(CRange<Real>(75.0f, 100.0f)),
                            m_pcRNG->Uniform(CRange<CRadians>(-CRadians::PI, CRadians::PI)));

            //std::cout<<"tmp variable: ("<<tmp.GetX()<<","<<tmp.GetY()<<")"<<std::endl;
            sensor_readings[i].Range = tmp.Length();
            if (sensor_readings[i].Range > max_rab_range)
            {
                sensor_readings[i].Range = max_rab_range;
            }
            sensor_readings[i].HorizontalBearing = tmp.Angle();

            //std::cout<<"range after:"<<sensor_readings[i].Range<<std::endl;
            //std::cout<<"horizontal bearing after:"<<sensor_readings[i].HorizontalBearing<<std::endl;
        }

        return sensor_readings;
    }

    else if (fault_type == FaultBehavior::FAULT_RABSENSOR_MISSINGRECEIVERS)
    {
        Real ReceiverSensingInterval[12][2];

        ReceiverSensingInterval[0][0] = 0.0f;
        ReceiverSensingInterval[0][1] = 15.0f + (50.0f - 15.0f) / 2.0f;
        ReceiverSensingInterval[1][0] = 15.0f + (50.0f - 15.0f) / 2.0f;
        ReceiverSensingInterval[1][1] = 50.0f + (75.0f - 50.0f) / 2.0f;
        ReceiverSensingInterval[2][0] = 50.0f + (75.0f - 50.0f) / 2.0f;
        ReceiverSensingInterval[2][1] = 75.0f + (105.0f - 75.0f) / 2.0f;
        ReceiverSensingInterval[3][0] = 75.0f + (105.0f - 75.0f) / 2.0f;
        ReceiverSensingInterval[3][1] = 105.0f + (133.0f - 105.0f) / 2.0f;
        ReceiverSensingInterval[4][0] = 105.0f + (133.0f - 105.0f) / 2.0f;
        ReceiverSensingInterval[4][1] = 133.0f + (159.0f - 133.0f) / 2.0f;
        ReceiverSensingInterval[5][0] = 133.0f + (159.0f - 133.0f) / 2.0f;
        ReceiverSensingInterval[5][1] = 159.0f + (195.0f - 159.0f) / 2.0f;
        ReceiverSensingInterval[6][0] = 159.0f + (195.0f - 159.0f) / 2.0f;
        ReceiverSensingInterval[6][1] = 195.0f + (225.0f - 195.0f) / 2.0f;
        ReceiverSensingInterval[7][0] = 195.0f + (225.0f - 195.0f) / 2.0f;
        ReceiverSensingInterval[7][1] = 225.0f + (255.0f - 225.0f) / 2.0f;
        ReceiverSensingInterval[8][0] = 225.0f + (255.0f - 225.0f) / 2.0f;
        ReceiverSensingInterval[8][1] = 255.0f + (283.0f - 255.0f) / 2.0f;
        ReceiverSensingInterval[9][0] = 255.0f + (283.0f - 255.0f) / 2.0f;
        ReceiverSensingInterval[9][1] = 283.0f + (310.0f - 283.0f) / 2.0f;
        ReceiverSensingInterval[10][0] = 283.0f + (310.0f - 283.0f) / 2.0f;
        ReceiverSensingInterval[10][1] = 310.0f + (345.0f - 310.0f) / 2.0f;
        ReceiverSensingInterval[11][0] = 310.0f + (345.0f - 310.0f) / 2.0f;
        ReceiverSensingInterval[11][1] = 360.0f;

        Real startangle, endangle;
        // assume the front two receivers numbered 0 and 11 are missing
        for (size_t i = 0; i < sensor_readings.size(); ++i)
        {
            CRadians BearingAngle = sensor_readings[i].HorizontalBearing;
            Real BearingAngle_Degrees = ToDegrees(BearingAngle).UnsignedNormalize().GetValue();

            assert(BearingAngle_Degrees >= 0.0f);

            startangle = 105.0f + (133.0f - 105.0f) / 2.0f;
            endangle = 225.0f + (255.0f - 225.0f) / 2.0f;
            if ((BearingAngle_Degrees >= 0 && BearingAngle_Degrees <= startangle) || (BearingAngle_Degrees >= endangle && BearingAngle_Degrees <= 360.0f))
            {
                if (BearingAngle_Degrees >= 0 && BearingAngle_Degrees <= startangle)
                    BearingAngle_Degrees = startangle;
                else
                    BearingAngle_Degrees = endangle;

                //sensor_readings[i].Range             = ;
                sensor_readings[i].HorizontalBearing = ToRadians(CDegrees(BearingAngle_Degrees));

                continue;
            }
        }
        return sensor_readings;
    }

    else
    {
        /* the robot is running one of the general faults or one of the specific faults that doesnot influence RAB sensor readings*/
        return sensor_readings;
    }
}

std::vector<Real> BaseController::GetNormalizedSensorReadings()
{
    std::vector<Real> norm_readings;
    std::vector<CCI_ThymioProximitySensor::SReading> proximity_sensors = GetIRSensorReadings(b_damagedrobot, FBehavior);
    for (UInt8 i = 0; i < proximity_sensors.size(); ++i)
        norm_readings.push_back((1.0f - proximity_sensors[i].Value) * 2.0f - 1.0f);

    CCI_RangeAndBearingSensor::TReadings rab_sensors = GetRABSensorReadings(b_damagedrobot, FBehavior);

    for (UInt8 i = proximity_sensors.size(); i < proximity_sensors.size() + 8; ++i) // 8 cones
    {
        norm_readings.push_back(max_rab_range); // initialize rab sensor readings to max range of 100cm
    }

#ifdef RAB_CONTROL
    for (UInt8 i = proximity_sensors.size() + 8; i < proximity_sensors.size() + 16; ++i) // 8 data cones
    {
        norm_readings.push_back(0); // initialize rab data readings to 0 (default value)
    }
#endif

    for (UInt8 i = 0; i < rab_sensors.size(); ++i)
    {
        Real min_diff = 1000.0;
        UInt8 min_cone_index;
        for (UInt8 rab_cone_index = 0; rab_cone_index < rab_cones.size(); ++rab_cone_index)
        {
            if (NormalizedDifference(rab_sensors[i].HorizontalBearing, rab_cones[rab_cone_index]).GetAbsoluteValue() < min_diff)
            {
                min_diff = NormalizedDifference(rab_sensors[i].HorizontalBearing, rab_cones[rab_cone_index]).GetAbsoluteValue();
                min_cone_index = rab_cone_index;
            }
        }
        if (min_diff != 1000.0)
            if (norm_readings[proximity_sensors.size() + min_cone_index] > rab_sensors[i].Range)
            {
                norm_readings[proximity_sensors.size() + min_cone_index] = rab_sensors[i].Range;
#ifdef RAB_CONTROL
                // reinterpret 4 bytes into a float. This value will always have value [-1.0, 1.0]
                UInt8 sensed_value = rab_sensors[i].Data[0];
                norm_readings[proximity_sensors.size() + 8 + min_cone_index] = 1 - (sensed_value / 255.0f) * 2;
#endif
            }
    }

    for (UInt8 rab_cone_index = 0; rab_cone_index < rab_cones.size(); ++rab_cone_index)
        norm_readings[proximity_sensors.size() + rab_cone_index] = (norm_readings[proximity_sensors.size() + rab_cone_index] / max_rab_range) * 2.0 - 1.0;

    return norm_readings;
}

/* left wheel speed normalised to [0,1]*/
float BaseController::left_wheel_velocity_01()
{
    return (m_sWheelTurningParams.MaxSpeed + m_fLeftSpeed) / (2.0 * m_sWheelTurningParams.MaxSpeed); // in [0,1];
}
/* right wheel speed normalised to [0,1]*/
float BaseController::right_wheel_velocity_01()
{
    return (m_sWheelTurningParams.MaxSpeed + m_fRightSpeed) / (2.0 * m_sWheelTurningParams.MaxSpeed); // in [0,1];
}
/* linear speed normalised to [0,1]*/
float BaseController::linear_speed_01()
{
    return (std::abs(m_fLeftSpeed) + std::abs(m_fRightSpeed)) / (2.0 * m_sWheelTurningParams.MaxSpeed); // in [0,1]
}
/* linear velocity normalised to [0,1]*/
float BaseController::linear_velocity_01()
{
    return (2.0 * m_sWheelTurningParams.MaxSpeed + (m_fLeftSpeed + m_fRightSpeed)) / (4.0 * m_sWheelTurningParams.MaxSpeed); // in [0,1]
}
/* linear velocity normalised to [-1,1]*/
float BaseController::linear_velocity_signed()
{
    return (m_fLeftSpeed + m_fRightSpeed) / (2.0 * m_sWheelTurningParams.MaxSpeed); // in [-1,1]
}
/* turn speed normalised to [0,1]*/
float BaseController::turn_speed_01()
{
    return std::abs(m_fLeftSpeed - m_fRightSpeed) / (2.0 * m_sWheelTurningParams.MaxSpeed); // in [0,1]
}

float BaseController::linear_wheel_velocity_01()
{
    auto reading = m_pcWheelsEncoder->GetReading();
    float sum = reading.VelocityLeftWheel + reading.VelocityRightWheel;
    return 0.5 + sum / (4.0f*m_sWheelTurningParams.MaxSpeed);// [0,1]
}

float BaseController::turn_wheel_velocity_01()
{
    auto reading = m_pcWheelsEncoder->GetReading();
    float diff = reading.VelocityLeftWheel - reading.VelocityRightWheel;//[-2M,2M]
    return 0.5f + diff / (4.0f * m_sWheelTurningParams.MaxSpeed);// [0,1]
}

void BaseController::init_sensact(TConfigurationNode &t_node)
{
    /*
    * Get sensor/actuator handles
    */
    try
    {
        m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
        m_pcWheelsEncoder = GetSensor<CCI_DifferentialSteeringSensor>("differential_steering");
        m_pcLeds = GetActuator<CCI_ThymioLedsActuator>("thymio_led");
        m_pcProximity = GetSensor<CCI_ThymioProximitySensor>("Thymio_proximity");
        m_pcGround = GetSensor<CCI_ThymioGroundSensor>("Thymio_ground");
        m_pcRABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
        m_pcRABS = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
        only_proximity = false;

        rab_cones.push_back(ToRadians(CDegrees(0.0))); // going counter-clock wise
        rab_cones.push_back(ToRadians(CDegrees(45.0)));
        rab_cones.push_back(ToRadians(CDegrees(90.0)));
        rab_cones.push_back(ToRadians(CDegrees(135.0)));
        rab_cones.push_back(ToRadians(CDegrees(180.0)));
        rab_cones.push_back(ToRadians(CDegrees(-135.0)));
        rab_cones.push_back(ToRadians(CDegrees(-90.0)));
        rab_cones.push_back(ToRadians(CDegrees(-45.0)));
    }
    catch (CARGoSException &ex1)
    {
        try
        {
            // assume the user just wants to use only proximity sensors
            m_pcProximity = GetSensor<CCI_ThymioProximitySensor>("Thymio_proximity");

            only_proximity = true;
        }
        catch (CARGoSException &ex2)
        {
            THROW_ARGOSEXCEPTION_NESTED("Error initializing sensors/actuators", ex2);
        }
    }
}

void BaseController::init_fault_config(TConfigurationNode &t_node)
{
    /* Experiment to run */
    TConfigurationNode sub_node = GetNode(t_node, "experiment_run");
    std::string errorbehav, id_FaultyRobotInSwarm;
    try
    {
        GetNodeAttribute(sub_node, "fault_behavior", errorbehav);
        GetNodeAttribute(sub_node, "id_faulty_robot", id_FaultyRobotInSwarm);
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing type of experiment to run, and fault to simulate.", ex);
    }
    
    process_faultbehaviour(errorbehav);

    if (this->GetId().compare("thymio" + id_FaultyRobotInSwarm) == 0) //process by ID
    {

        b_damagedrobot = true;
        //std::cout<<"robot "<<id_FaultyRobotInSwarm<<"is damaged";
    }
    else if (id_FaultyRobotInSwarm.back() == '%') // process by proportion; assuming homogenous robots, can just do the first N robots
    {
        id_FaultyRobotInSwarm.pop_back();
        damage_probability = 0.01f * std::stoi(id_FaultyRobotInSwarm); // e.g. 99%
        // damage the first number_damaged robots
        //std::cout<<"will damage robots randomly, with probability "<< damage_probability<<std::endl;
    }
    else
    {
        // nothing: proceed as usual undamaged
        //std::cout<<"no damage "<<damage_probability<<std::endl;
    }
}
/****************************************/
/****************************************/
