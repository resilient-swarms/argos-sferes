#include "dispersebehavior.h"

#include <algorithm>

/******************************************************************************/
/******************************************************************************/

CDisperseBehavior::CDisperseBehavior()
{
    m_fCommunicationRange = 10000.0;
}

/******************************************************************************/
/******************************************************************************/

CDisperseBehavior::CDisperseBehavior(Real CommunicationRange) :
    m_fCommunicationRange(CommunicationRange)
{
}

/******************************************************************************/
/******************************************************************************/

bool CDisperseBehavior::TakeControl() 
{
    bool controltaken(false);

    m_cDiffusionVector.Set(0.0f, 0.0f);

    size_t m_unRobotsInRange = 0;
    Real min_dist = m_fCommunicationRange;
    for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        bool beacon = false;
        for(size_t j = 0; j <  m_sSensoryData.m_RABSensorData[i].Data[j]; ++j)
            if(m_sSensoryData.m_RABSensorData[i].Data[j] == CBehavior::m_sRobotData.BEACON_SIGNAL_MARKER ||
               m_sSensoryData.m_RABSensorData[i].Data[j] == CBehavior::m_sRobotData.NEST_BEACON_SIGNAL_MARKER)
            {
                beacon = true;
                break;
            }

        if(beacon)
            continue;

        if(m_sSensoryData.m_RABSensorData[i].Range < min_dist)
        {
            min_dist = m_sSensoryData.m_RABSensorData[i].Range;
            m_cDiffusionVector = CVector2(m_sSensoryData.m_RABSensorData[i].Range, m_sSensoryData.m_RABSensorData[i].HorizontalBearing);
            m_unRobotsInRange++;
        }
    }

    if(m_unRobotsInRange > 0u)
        controltaken = true;

    return controltaken;
}

/******************************************************************************/
/******************************************************************************/

// Move in the opposite direction of CoM
void CDisperseBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
    CVector2 m_cHeadingVector =  m_sRobotData.MaxSpeed  * (-1.0f * m_cDiffusionVector.Normalize());

    WheelSpeedsFromHeadingVector(m_cHeadingVector, fLeftWheelSpeed, fRightWheelSpeed);
}

/******************************************************************************/
/******************************************************************************/

void CDisperseBehavior::PrintBehaviorIdentity()
{
    std::cout << "Disperse taking over" << std::endl;
}

/******************************************************************************/
/******************************************************************************/

