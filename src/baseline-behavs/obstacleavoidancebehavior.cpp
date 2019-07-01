#include "obstacleavoidancebehavior.h"

#include <algorithm>

/******************************************************************************/
/******************************************************************************/

CObstacleAvoidanceBehavior::CObstacleAvoidanceBehavior(Real m_fProximitySensorThreshold, CRadians m_cGoStraightAngleThreshold) :
    m_fProximitySensorThreshold(m_fProximitySensorThreshold),
    m_cGoStraightAngleThreshold(m_cGoStraightAngleThreshold)
{
}

/******************************************************************************/
/******************************************************************************/

CObstacleAvoidanceBehavior::CObstacleAvoidanceBehavior(Real m_fProximitySensorThreshold) :
    m_fProximitySensorThreshold(m_fProximitySensorThreshold)
{
}

/******************************************************************************/
/******************************************************************************/

bool CObstacleAvoidanceBehavior::TakeControl()
{


    //    /* Get readings from proximity sensor */
    //    /* Sum them together */
    //    m_cDiffusionVector.Set(0.0f, 0.0f);
    //    for(size_t i = 0; i <  m_sSensoryData.m_ProximitySensorData.size(); ++i)
    //    {
    //        m_cDiffusionVector += CVector2(m_sSensoryData.m_ProximitySensorData[i].Value, m_sSensoryData.m_ProximitySensorData[i].Angle);
    //    }
    //    m_cDiffusionVector /= m_sSensoryData.m_ProximitySensorData.size();

    ////    std::cout << " m_cDiffusionVector length " << m_cDiffusionVector.Length() << " and threshold " << m_fProximitySensorThreshold << std::endl;
    ////    std::cout << " m_cDiffusionVector angle " <<  m_cDiffusionVector.Angle().GetAbsoluteValue() << " and threshold " << m_cGoStraightAngleThreshold.GetValue() << std::endl;


    for(size_t i = 0; i < 5; ++i) // considering the front 5 proximity sensors
        if(m_sSensoryData.m_ProximitySensorData[i].Value >= m_fProximitySensorThreshold)
            return true;

    return false;


    //    /* If the angle of the vector is small enough and the closest obstacle
    //          is far enough, ignore the vector and go straight, otherwise return
    //          it */
    //    if(m_cDiffusionVector.Angle().GetAbsoluteValue() >= m_cGoStraightAngleThreshold.GetValue() && m_cDiffusionVector.Length() < m_fProximitySensorThreshold)
    //        return false;
    //    else
    //    {
    //        //            if(m_cDiffusionVector.Length() < 0.05) /* because of noise, we can have very small non-zero sensor readings. but we don't want to responmd to them*/
    //        //                return false;

    //        std::cout << " m_cDiffusionVector length " << m_cDiffusionVector.Length() << " and threshold " << m_fProximitySensorThreshold << std::endl;
    //        std::cout << " m_cDiffusionVector angle " <<  m_cDiffusionVector.Angle().GetAbsoluteValue() << " and threshold " << m_cGoStraightAngleThreshold.GetValue() << std::endl;
    //        return true;
    //    }
}

/******************************************************************************/
/******************************************************************************/

// Move in the opposite direction of CoM
void CObstacleAvoidanceBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
    /* Get the highest reading in front of the robot, which corresponds to the closest object */
    Real fMaxReadVal = m_sSensoryData.m_ProximitySensorData[0].Value;
    UInt32 unMaxReadIdx = 0;
    if(fMaxReadVal < m_sSensoryData.m_ProximitySensorData[1].Value)
    {
        fMaxReadVal = m_sSensoryData.m_ProximitySensorData[1].Value;
        unMaxReadIdx = 1;
    }
    if(fMaxReadVal < m_sSensoryData.m_ProximitySensorData[2].Value)
    {
        fMaxReadVal = m_sSensoryData.m_ProximitySensorData[2].Value;
        unMaxReadIdx = 2;
    }
    if(fMaxReadVal < m_sSensoryData.m_ProximitySensorData[3].Value)
    {
        fMaxReadVal = m_sSensoryData.m_ProximitySensorData[3].Value;
        unMaxReadIdx = 3;
    }
    if(fMaxReadVal < m_sSensoryData.m_ProximitySensorData[4].Value)
    {
        fMaxReadVal = m_sSensoryData.m_ProximitySensorData[4].Value;
        unMaxReadIdx = 4;
    }
    /* Do we have an obstacle in front? */
    if(fMaxReadVal > 0.0f) {
        /* Yes, we do: avoid it */
        if(unMaxReadIdx == 0 || unMaxReadIdx == 1)
        {
            /* The obstacle is on the left, turn right */
            fLeftWheelSpeed = m_sRobotData.MaxSpeed; fRightWheelSpeed = 0.0f;
        }
        else
        {
            /* The obstacle is on the left, turn right */
            fLeftWheelSpeed = 0.0f; fRightWheelSpeed = m_sRobotData.MaxSpeed;
        }
    }
    else
    {
        /* No, we don't: go straight */
        fLeftWheelSpeed = m_sRobotData.MaxSpeed; fRightWheelSpeed = m_sRobotData.MaxSpeed;
    }
}

/******************************************************************************/
/******************************************************************************/

void CObstacleAvoidanceBehavior::PrintBehaviorIdentity()
{
    std::cout << "Obs Avoidance taking over" << std::endl;
}

/******************************************************************************/
/******************************************************************************/

