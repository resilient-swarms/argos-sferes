#include "bordercoveragebehavior.h"

#include <algorithm>

/******************************************************************************/
/******************************************************************************/

CBorderCoverageBehavior::CBorderCoverageBehavior(Real m_fProximitySensorThreshold, UInt64 *BorderCoverageStartTime) :
    m_fProximitySensorThreshold(m_fProximitySensorThreshold),
    m_ptBorderCoverageStartTime(BorderCoverageStartTime)
{
}

/******************************************************************************/
/******************************************************************************/

bool CBorderCoverageBehavior::TakeControl()
{

    /* Get readings from the five front proximity sensor */
    /* Sum them together */
    m_cDiffusionVector.Set(0.0f, 0.0f);
    for(size_t i = 0; i <  5; ++i)
    {
        m_cDiffusionVector += CVector2(m_sSensoryData.m_ProximitySensorData[i].Value, m_sSensoryData.m_ProximitySensorData[i].Angle);
    }
    m_cDiffusionVector /= 5.0f;

    m_cDiffusionVector.Rotate(CRadians(3.142f/4.0f)); // Rotate the diffusion vector by 45 degrees

    ////    std::cout << " m_cDiffusionVector: " << m_cDiffusionVector.Length() << " and angle " <<  m_cDiffusionVector.Angle().GetAbsoluteValue() << std::endl;alue() << std::endl;


    for(size_t i = 0; i < 5; ++i) // considering the front 5 proximity sensors
        if(m_sSensoryData.m_ProximitySensorData[i].Value >= m_fProximitySensorThreshold)
        {
            *m_ptBorderCoverageStartTime = m_sSensoryData.m_rTime;
            return true;
        }

    return false;
}

/******************************************************************************/
/******************************************************************************/

// Move in the opposite direction of CoM
void CBorderCoverageBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
    CVector2 m_cHeadingVector =  m_sRobotData.MaxSpeed  * (-1.0f * m_cDiffusionVector.Normalize());
    WheelSpeedsFromHeadingVector(m_cHeadingVector, fLeftWheelSpeed, fRightWheelSpeed);

    //! can we bias random walk to turn slightly left only if CBorderCoverageBehavior had taken control in say last 1 sec?
}

/******************************************************************************/
/******************************************************************************/

void CBorderCoverageBehavior::PrintBehaviorIdentity()
{
    std::cout << "Border coverage behavior taking over" << std::endl;
}

/******************************************************************************/
/******************************************************************************/

