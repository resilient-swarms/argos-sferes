#include "dest_chain_behavior.h"

/******************************************************************************/
/******************************************************************************/

CDestChainBehavior::CDestChainBehavior(Real m_fRangeAndBearing_RangeThreshold) :
    m_fRangeAndBearing_RangeThreshold(m_fRangeAndBearing_RangeThreshold)
{
}

/******************************************************************************/
/******************************************************************************/
    
bool CDestChainBehavior::TakeControl()
{
    bool controltaken(false);

    m_cCoverageVector.Set(0.0f, 0.0f);

    m_unRobotsInRange = 0;
    for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        if(m_sSensoryData.m_RABSensorData[i].Range < m_fRangeAndBearing_RangeThreshold)
        {
            // controltaken = true;
            m_cCoverageVector += CVector2(m_sSensoryData.m_RABSensorData[i].Range, m_sSensoryData.m_RABSensorData[i].HorizontalBearing);
            m_unRobotsInRange++;
        }
    }

    if(m_unRobotsInRange > 0u)
        controltaken = true;

    if(controltaken)
        m_cCoverageVector /= m_unRobotsInRange;

    return controltaken;
}

/******************************************************************************/
/******************************************************************************/

// Move in the opposite direction of CoM of neighbours
// If you loose connection with neighbours, random walk takes over until connection established
void CDestChainBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
//     unsigned m_unMaxRobotsInRange = 3;
//     Real m_fAggregationPower = m_unRobotsInRange / m_unMaxRobotsInRange;
//     if (m_fAggregationPower > 1.0)
//         m_fAggregationPower = 1.0;

//     CVector2 m_cHeadingVector =  m_sRobotData.MaxSpeed  * (m_fAggregationPower * m_cCoverageVector.Normalize() +
//                                                            (1.0-m_fAggregationPower) * CVector2(1.0f, m_sSensoryData.m_pcRNG->Uniform(CRange<CRadians>(-CRadians::PI, CRadians::PI))));

     CVector2 m_cHeadingVector =  m_sRobotData.MaxSpeed  * (-1.0f * m_cCoverageVector.Normalize());


     // i add a strong random component to break stable aggregates of pair of robots.
     /*CVector2 m_cHeadingVector =  m_sRobotData.MaxSpeed  * (0.3f * m_cCoverageVector.Normalize() +
                                                            0.7f * CVector2(1.0f, m_sSensoryData.m_pcRNG->Uniform(CRange<CRadians>(-CRadians::PI, CRadians::PI))));*/

     WheelSpeedsFromHeadingVector(m_cHeadingVector, fLeftWheelSpeed, fRightWheelSpeed);
}

/******************************************************************************/
/******************************************************************************/

void CDestChainBehavior::PrintBehaviorIdentity()
{
    std::cout << "Dest-chain behaviour taking over";
}

/******************************************************************************/
/******************************************************************************/
