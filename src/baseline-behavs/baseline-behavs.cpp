/* Include the controller definition */
#include "baseline-behavs.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>



/****************************************/
/****************************************/

CBehavior::SensoryData CBehavior::m_sSensoryData;
CBehavior::RobotData CBehavior::m_sRobotData;

/****************************************/
/****************************************/

CBaselineBehavs::ExperimentToRun::ExperimentToRun() :
    SBehavior(SWARM_AGGREGATION)
{
}

/****************************************/
/****************************************/

void CBaselineBehavs::ExperimentToRun::Init(TConfigurationNode& t_node)
{

    try
    {
        GetNodeAttribute(t_node, "swarm_behavior", swarmbehav);
    }
    catch(CARGoSException& ex)
            THROW_ARGOSEXCEPTION_NESTED("Error initializing type of experiment to run, and fault to simulate.", ex);

    if (swarmbehav.compare("SWARM_AGGREGATION") == 0)
        SBehavior = SWARM_AGGREGATION;
    else if (swarmbehav.compare("SWARM_DISPERSION") == 0)
        SBehavior = SWARM_DISPERSION;
    else if (swarmbehav.compare("SWARM_COVERAGE") == 0)
        SBehavior = SWARM_COVERAGE;
    else if (swarmbehav.compare("SWARM_BORDERCOVERAGE") == 0)
        SBehavior = SWARM_BORDERCOVERAGE;
    else if (swarmbehav.compare("SWARM_CHAINING") == 0)
        SBehavior = SWARM_CHAINING;
    else if (swarmbehav.compare("SWARM_FLOCKING") == 0)
        SBehavior = SWARM_FLOCKING;
    else if (swarmbehav.compare("SWARM_HOMING") == 0)
        SBehavior = SWARM_HOMING;
    else if (swarmbehav.compare("SWARM_STOP") == 0)
        SBehavior = SWARM_STOP;
    else
    {
        std::cerr << "invalid swarm behavior";
        assert(-1);
    }


}


/****************************************/
/****************************************/


CBaselineBehavs::CBaselineBehavs() :
    src_robot(false),
    dest_robot(false),
    m_unBorderCoverageStartTime(999999u)
{
    m_fInternalRobotTimer = 0.0f;

}

/****************************************/
/****************************************/

void CBaselineBehavs::Init(TConfigurationNode& t_node)
{
    BaseController::Init(t_node);

    /* Baseline behavior to run */
    m_sExpRun.Init(GetNode(t_node, "experiment_run"));

    m_sRobotDetails.SetKinematicDetails(m_sWheelTurningParams.MaxSpeed, m_sWheelTurningParams.MaxSpeed);

    CopyRobotDetails(m_sRobotDetails);

    if(GetExperimentType().SBehavior==m_sExpRun.SWARM_FLOCKING && m_sWheelTurningParams.MaxSpeed > 5.0f) // max speed and speed limit cm/s
    {
        printf("In Flocking, robots running at high speeds result in inaccurate observed heading vectors "
               "because the physics engine provides a too high \"momentum\" to the robots with the current step-size.\n"
               "Also make sure the simulation ticks is set to 10 / second and interations is set to 10 per tick.\n");
        exit(-1);
    }

    m_pFlockingBehavior = new CFlockingBehavior(m_sRobotDetails.iterations_per_second * 1.0f); // 5.0f

    // For chaining behaviours
    m_iParentRobotId   = -1;
    m_iChildRobotId    = -1;
    m_bBeaconSignalOn  = false;
    m_iTimeLastRequest = -1;
}

/****************************************/
/****************************************/

void CBaselineBehavs::CopyRobotDetails(RobotDetails& robdetails)
{

    CBehavior::m_sRobotData.MaxSpeed                    = robdetails.MaxLinearSpeed * robdetails.iterations_per_second; // max speed in cm/s to control behavior
    CBehavior::m_sRobotData.iterations_per_second       = robdetails.iterations_per_second;
    CBehavior::m_sRobotData.seconds_per_iterations      = 1.0f / robdetails.iterations_per_second;
    CBehavior::m_sRobotData.HALF_INTERWHEEL_DISTANCE    = robdetails.HALF_INTERWHEEL_DISTANCE;
    CBehavior::m_sRobotData.INTERWHEEL_DISTANCE         = robdetails.INTERWHEEL_DISTANCE;
    CBehavior::m_sRobotData.WHEEL_RADIUS                = robdetails.WHEEL_RADIUS;

    CBehavior::m_sRobotData.m_cNoTurnOnAngleThreshold   = robdetails.m_cNoTurnOnAngleThreshold;
    CBehavior::m_sRobotData.m_cSoftTurnOnAngleThreshold = robdetails.m_cSoftTurnOnAngleThreshold;

    CBehavior::m_sRobotData.BEACON_SIGNAL_MARKER           = BEACON_SIGNAL;
    CBehavior::m_sRobotData.NEST_BEACON_SIGNAL_MARKER      = NEST_BEACON_SIGNAL;
    CBehavior::m_sRobotData.SELF_INFO_PACKET_MARKER        = SELF_INFO_PACKET;
    CBehavior::m_sRobotData.SELF_INFO_PACKET_FOOTER_MARKER = SELF_INFO_PACKET_FOOTER;

    CBehavior::m_sRobotData.CHAIN_CONNECTOR_PACKET_MARKER           = CHAIN_CONNECTOR_PACKET;
    CBehavior::m_sRobotData.CHAIN_CONNECTOR_REQUEST_MARKER          = CHAIN_CONNECTOR_REQUEST;
    CBehavior::m_sRobotData.CHAIN_CONNECTOR_REQUESTACCEPTED_MARKER  = CHAIN_CONNECTOR_REQUESTACCEPTED;
    CBehavior::m_sRobotData.CHAIN_CONNECTOR_PACKET_FOOTER_MARKER    = CHAIN_CONNECTOR_PACKET_FOOTER;

    CBehavior::m_sRobotData.RELAY_PACKET_MARKER            = RELAY_PACKET;
    CBehavior::m_sRobotData.RELAY_PACKET_FOOTER_MARKER     = RELAY_PACKET_FOOTER;
    CBehavior::m_sRobotData.VOTER_PACKET_MARKER            = VOTER_PACKET;
    CBehavior::m_sRobotData.VOTER_PACKET_FOOTER_MARKER     = VOTER_PACKET_FOOTER;
    CBehavior::m_sRobotData.DATA_BYTE_BOUND_MARKER         = DATA_BYTE_BOUND;
}

/****************************************/
/****************************************/

void CBaselineBehavs::ControlStep()
{
    m_pcRABA->ClearData(); // clear the channel at the start of each control cycle
    m_uRABDataIndex = 0;

    bool b_RunningGeneralFaults(false);
    if(b_damagedrobot && (FBehavior == FaultBehavior::FAULT_STRAIGHTLINE ||
                          FBehavior == FaultBehavior::FAULT_RANDOMWALK ||
                          FBehavior == FaultBehavior::FAULT_CIRCLE ||
                          FBehavior == FaultBehavior::FAULT_STOP))
    {
        b_RunningGeneralFaults = true;
        RunGeneralFaults();
    }

    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_AGGREGATION    ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_DISPERSION     ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_FLOCKING       ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_HOMING         ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_COVERAGE       ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_BORDERCOVERAGE ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_CHAINING       ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_STOP)
        RunHomogeneousSwarmExperiment();


    // note: next lines were doing the same so I commented them
    CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, FBehavior), GetRABSensorReadings(b_damagedrobot, FBehavior));
    // if(!b_damagedrobot || b_RunningGeneralFaults || FBehavior == FaultBehavior::FAULT_NONE)
    //     CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, FBehavior), GetRABSensorReadings(b_damagedrobot, FBehavior));
    // else
    // {
        // if(FBehavior == FaultBehavior::FAULT_PROXIMITYSENSORS_SETMIN)
        //     CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, FBehavior), GetRABSensorReadings(b_damagedrobot, FBehavior));
        // else if(FBehavior == FaultBehavior::FAULT_PROXIMITYSENSORS_SETMAX)
        //     CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, FBehavior), GetRABSensorReadings(b_damagedrobot, FBehavior));
        // else if(FBehavior == FaultBehavior::FAULT_PROXIMITYSENSORS_SETRANDOM)
        //     CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, FBehavior), GetRABSensorReadings(b_damagedrobot, FBehavior));
        // else if(FBehavior == FaultBehavior::FAULT_PROXIMITYSENSORS_SETOFFSET)
        //     CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, FBehavior), GetRABSensorReadings(b_damagedrobot, FBehavior));


        // else if(FBehavior == FaultBehavior::FAULT_RABSENSOR_SETOFFSET)
        //     CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, FBehavior), GetRABSensorReadings(b_damagedrobot, FBehavior));
        // else if(FBehavior == FaultBehavior::FAULT_RABSENSOR_MISSINGRECEIVERS)
        //     CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, FBehavior), GetRABSensorReadings(b_damagedrobot, FBehavior));


        // else if(FBehavior == FaultBehavior::FAULT_ACTUATOR_LWHEEL_SETZERO)
        // {
        //     // does not affect the sensors - they stay the same
        //     CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, FBehavior), GetRABSensorReadings(b_damagedrobot, FBehavior));
        // }
        // else if(FBehavior == FaultBehavior::FAULT_ACTUATOR_RWHEEL_SETZERO)
        // {
        //     // does not affect the sensors - they stay the same
        //     CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, FBehavior), GetRABSensorReadings(b_damagedrobot, FBehavior));
        // }
        // else if(FBehavior == FaultBehavior::FAULT_ACTUATOR_BWHEELS_SETZERO)
        // {
        //     // does not affect the sensors - they stay the same
        //     CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, FBehavior), GetRABSensorReadings(b_damagedrobot, FBehavior));
        // }


        // else if(FBehavior == FaultBehavior::FAULT_SOFTWARE)
        //     CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, FBehavior), GetRABSensorReadings(b_damagedrobot, FBehavior));

        // else if(FBehavior == FaultBehavior::FAULT_POWER_FAILURE)
        //     CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, FBehavior), GetRABSensorReadings(b_damagedrobot, FBehavior));
    // }

    /*For flocking behavior - to compute relative velocity*/
    CBehavior::m_sSensoryData.SetWheelSpeedsFromEncoders(m_pcWheelsEncoder->GetReading().VelocityLeftWheel, m_pcWheelsEncoder->GetReading().VelocityRightWheel);

    /*The robot has to continually track the velocity of its neighbours - since this is done over a period of time. It can't wait until the flocking behavior is activated to start tracking neighbours*/
    m_pFlockingBehavior->SimulationStep();

    bool bControlTaken = false;
    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
    {
        if (!bControlTaken)
        {
            bControlTaken = (*i)->TakeControl();
            if (bControlTaken)
            {
                (*i)->Action(m_fLeftSpeed, m_fRightSpeed);
            }
        } else
            (*i)->Suppress();
    }

    if (b_damagedrobot)
        damage_actuators();

    m_pcWheels->SetLinearVelocity(m_fLeftSpeed, m_fRightSpeed); // in cm/s


    m_uRobotId = RobotIdStrToInt();

    /*Communicate self-info -- robot id */
    //SenseCommunicate(RobotIdStrToInt(), m_pcRABA, m_uRABDataIndex);
    m_pcRABA->SetData(m_uRABDataIndex++, CBehavior::m_sRobotData.SELF_INFO_PACKET_MARKER);
    m_pcRABA->SetData(m_uRABDataIndex++, m_uRobotId);
    if(m_uRABDataIndex > m_pcRABA->GetSize()-1)
    {
        std::cerr << " rab buffer full. exiting";
        exit(-1);
    }
     m_pcRABA->SetData(m_uRABDataIndex++, CBehavior::m_sRobotData.SELF_INFO_PACKET_FOOTER_MARKER);



    m_fInternalRobotTimer++;
    

//    if(this->GetId().compare("thymio0") == 0)
//    {
//            CCI_RangeAndBearingSensor::TReadings sensor_readings = m_pcRABS->GetReadings();
//            std::cout << "Robots in RAB range of " << GetId() << " is " << sensor_readings.size() << std::endl;
//            for(size_t i = 0; i < sensor_readings.size(); ++i)
//            {
//                std::cout << "RAB range " << sensor_readings[i].Range << " Bearing "  << sensor_readings[i].HorizontalBearing << std::endl;
////                for(size_t j = 0; j < sensor_readings[i].Data.Size(); ++j)
////                    std::cout << "Data-Packet at index " << j << " is " << sensor_readings[i].Data[j] << std::endl;
//            }
//            for (UInt8 i = 0; i < GetNormalizedSensorReadings().size(); ++i)
//            {
//                std::cout << GetNormalizedSensorReadings()[i] << " ";
//            }
//            std::cout << std::endl;
//    }





    //    m_pcRABA->SetData(0, 100); // send test-value of 100 on RAB medium

    //    /* Get readings from proximity sensor */
    //    const CCI_ThymioProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
    //    /* Get readings from ground sensor */
    //    const CCI_ThymioGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();

    //    m_pcLeds->SetProxHIntensity(tProxReads);

    //    //   LOG << tProxReads;
    //    //   LOG << tProxReads[2].Value<< tProxReads[2].Angle.GetValue();
    //    //   std::cout << tProxReads;

    //    m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);


    //    /* Sum them together */
    //    CVector2 cAccumulator;
    //    for(size_t i = 0; i < tProxReads.size(); ++i)
    //    {
    //        cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
    //    }
    //    cAccumulator /= tProxReads.size();

    //    short cground = 0;
    //    for(size_t i = 0; i < tGroundReads.size(); ++i)
    //    {
    //        cground += tGroundReads[i].Value;
    //    }

    //    /* If the angle of the vector is small enough and the closest obstacle
    //    * is far enough, continue going straight, otherwise curve a little
    //    */
    //    CRadians cAngle = cAccumulator.Angle();
    //    LOG << cAngle.GetValue();
    //    if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
    //            cAccumulator.Length() < m_fDelta )
    //    {
    //        /* Go straight */
    //        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
    //    }
    //    else
    //    {
    //        /* Turn, depending on the sign of the angle */
    //        if(cAngle.GetValue() < 0)
    //        {
    //            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0);
    //        }
    //        else
    //        {
    //            m_pcWheels->SetLinearVelocity(0, m_fWheelVelocity);
    //        }
    //    }


}

CBaselineBehavs::~CBaselineBehavs()
{
    m_pcWheels->SetLinearVelocity(0.0f, 0.0f);

    // delete all behaviors
}

/****************************************/
/****************************************/


void CBaselineBehavs::RunGeneralFaults()
{
    //m_pcLEDs->SetAllColors(CColor::RED);

    m_vecBehaviors.clear();
    if(FBehavior == FaultBehavior::FAULT_STRAIGHTLINE)
    {
        CRandomWalkBehavior* pcStraightLineBehavior = new CRandomWalkBehavior(0.0f);
        m_vecBehaviors.push_back(pcStraightLineBehavior);
    }
    else if (FBehavior == FaultBehavior::FAULT_RANDOMWALK)
    {
        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f);  // 0.05f
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    else if (FBehavior == FaultBehavior::FAULT_CIRCLE)
    {
        CCircleBehavior* pcCircleBehavior = new CCircleBehavior();
        m_vecBehaviors.push_back(pcCircleBehavior);
    }

    else //FBehavior == FaultBehavior::FAULT_STOP
    {}
}

/****************************************/
/****************************************/

void CBaselineBehavs::RunHomogeneousSwarmExperiment()
{
    m_vecBehaviors.clear();

    if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_AGGREGATION)
    {
        std::cout << "Running aggregation " << std::endl;
        CObstacleAvoidanceBehavior* pcObstacleAvoidanceBehavior = new CObstacleAvoidanceBehavior(0.1f);    // 0.1f reflects a distance of about 4.5cm
        m_vecBehaviors.push_back(pcObstacleAvoidanceBehavior);

        CAggregateBehavior* pcAggregateBehavior = new CAggregateBehavior(100.0f); //range threshold in cm //60.0
        m_vecBehaviors.push_back(pcAggregateBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f); //0.05f
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_DISPERSION)
    {
        CObstacleAvoidanceBehavior* pcObstacleAvoidanceBehavior = new CObstacleAvoidanceBehavior(0.1f);
        m_vecBehaviors.push_back(pcObstacleAvoidanceBehavior);

        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior();
        m_vecBehaviors.push_back(pcDisperseBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f); //0.05f
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_COVERAGE)
    {
        CObstacleAvoidanceBehavior* pcObstacleAvoidanceBehavior = new CObstacleAvoidanceBehavior(0.1f);
        m_vecBehaviors.push_back(pcObstacleAvoidanceBehavior);

        CCoverageBehavior* pcCoverageBehavior = new CCoverageBehavior(100.0f);
        m_vecBehaviors.push_back(pcCoverageBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01f); //0.05f
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_BORDERCOVERAGE)
    {
        // hug walls, then slide along them

        //CDisperseBehavior* pcObstacleAvoidanceBehavior = new CDisperseBehavior(0.1f);
        //m_vecBehaviors.push_back(pcObstacleAvoidanceBehavior);

        CCoverageBehavior* pcCoverageBehavior = new CCoverageBehavior(1.0f); // use rab sensors to keep from colliding robots.
        m_vecBehaviors.push_back(pcCoverageBehavior);

        CBorderCoverageBehavior* pcBorderCoverageBehavior = new CBorderCoverageBehavior(0.1f, &m_unBorderCoverageStartTime);
        m_vecBehaviors.push_back(pcBorderCoverageBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01f, &m_unBorderCoverageStartTime);
        m_vecBehaviors.push_back(pcRandomWalkBehavior);

        //std::cout << "m_unBorderCoverageStartTime = " << m_unBorderCoverageStartTime << std::endl;
    }

    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_FLOCKING)
    {
        CObstacleAvoidanceBehavior* pcObstacleAvoidanceBehavior = new CObstacleAvoidanceBehavior(0.1f);
        m_vecBehaviors.push_back(pcObstacleAvoidanceBehavior);

        m_vecBehaviors.push_back(m_pFlockingBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01f);
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_HOMING)
    {
        if(this->GetId().compare("thymio0") == 0)
        {
            // thymio0 is the beacon robot
            /* Sends out data 'BEACON_SIGNAL' with RABS that you are a beacon. Neighbouring robots will use this data to home in on your position */
            // BEACON_SIGNAL is way above the DATA_BYTE_BOUND

            m_pcRABA->SetData(0, BEACON_SIGNAL);
            m_uRABDataIndex++;
            //m_pcLEDs->SetAllColors(CColor::YELLOW);
        }
        else
        {
            CObstacleAvoidanceBehavior* pcObstacleAvoidanceBehavior = new CObstacleAvoidanceBehavior(0.1f);    // 0.1f reflects a distance of about 4.5cm
            m_vecBehaviors.push_back(pcObstacleAvoidanceBehavior);

            Real MAX_BEACON_SIGNAL_RANGE = 1.0f; //1m
            CHomingToFoodBeaconBehavior* pcHomingToFoodBeaconBehavior = new CHomingToFoodBeaconBehavior(BEACON_SIGNAL, MAX_BEACON_SIGNAL_RANGE);
            m_vecBehaviors.push_back(pcHomingToFoodBeaconBehavior);

            CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01f); //0.05f
            m_vecBehaviors.push_back(pcRandomWalkBehavior);
        }
    }

    else if (m_sExpRun.SBehavior == ExperimentToRun::SWARM_CHAINING)
    {
        if(src_robot)
        {
            CSrcChainBehavior* pcSrcChainBehavior = new CSrcChainBehavior(&m_uRABDataIndex, m_pcRABA, m_pcLeds, RobotIdStrToInt());
            m_vecBehaviors.push_back(pcSrcChainBehavior);
        }
        else
        {
            /*CDisperseBehavior* pcObstacleAvoidanceBehavior = new CDisperseBehavior(0.1f);
            m_vecBehaviors.push_back(pcObstacleAvoidanceBehavior);*/

            CLinkChainBehavior* pcLnkChainBehavior = new CLinkChainBehavior(&m_uRABDataIndex, m_pcRABA, m_pcLeds, RobotIdStrToInt(), &m_bBeaconSignalOn, &m_iParentRobotId,
                                                                            &m_iChildRobotId, &m_iTimeLastRequest);
            m_vecBehaviors.push_back(pcLnkChainBehavior);

            /*CDisperseBehavior* pcObstacleAvoidanceBehavior = new CDisperseBehavior(0.1f);
            m_vecBehaviors.push_back(pcObstacleAvoidanceBehavior);

            CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01f);
            m_vecBehaviors.push_back(pcRandomWalkBehavior);*/
        }

        /*if(dest_robot)
            m_pcLeds->SetColor(CColor::GREEN);*/
    }

    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_STOP)
    {
    }
}

/****************************************/
/****************************************/


/****************************************/
/****************************************/

unsigned CBaselineBehavs::RobotIdStrToInt()
{
    std::string id = GetId();
    id.erase(0, 6); // remove the first six characters 'thymio'

    std::string::size_type sz;   // alias of size_t
    unsigned u_id = std::stoi(id, &sz);
    return u_id;
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
