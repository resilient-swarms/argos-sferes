#include "dispersebehavior.h"

#include <algorithm>

/******************************************************************************/
/******************************************************************************/

CDisperseBehavior::CDisperseBehavior()
{
}

/******************************************************************************/
/******************************************************************************/

bool CDisperseBehavior::TakeControl() 
{
    bool controltaken(false);

    m_cDiffusionVector.Set(0.0f, 0.0f);

    size_t m_unRobotsInRange = 0;
    Real min_dist = 10000.0;
    for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
    {
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

