#include "aggregatebehavior.h"


/******************************************************************************/
/******************************************************************************/

CAggregateBehavior::CAggregateBehavior(Real m_fRangeAndBearing_RangeThreshold) :
    m_fRangeAndBearing_RangeThreshold(m_fRangeAndBearing_RangeThreshold)
{
}

/******************************************************************************/
/******************************************************************************/
    
bool CAggregateBehavior::TakeControl()
{
    bool controltaken(false);

    // using range and bearing sensors since proximity sensors have a very small range for both dispersion and aggregation to take place (around 10cm ideally).
    // also we need to aggregate to other robots - and not obstacles
    m_cAggregationVector.Set(0.0f, 0.0f);

    m_unRobotsInRange = 0;
    for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        if(m_sSensoryData.m_RABSensorData[i].Range < m_fRangeAndBearing_RangeThreshold)
        {
            // controltaken = true;
            m_cAggregationVector += CVector2(m_sSensoryData.m_RABSensorData[i].Range, m_sSensoryData.m_RABSensorData[i].HorizontalBearing);
            m_unRobotsInRange++;
        }
    }

    if(m_unRobotsInRange > 0u) // use > 3 instead of > 0 to avoid small aggregates of just 2 robots or just 3 robots.
        controltaken = true;

    /*if(robotsinrange > 0u)
        controltaken = true;*/

    if(controltaken)
    {
        m_cAggregationVector /= m_unRobotsInRange;
        //std::cout << "Aggregation behavior taking control " << std::endl;
    }

    return controltaken;
}

/******************************************************************************/
/******************************************************************************/

// Move in the opposite direction of CoM
void CAggregateBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
     unsigned m_unMaxRobotsInRange = 3;
     Real m_fAggregationPower = m_unRobotsInRange / m_unMaxRobotsInRange;
     if (m_fAggregationPower > 1.0)
         m_fAggregationPower = 1.0;

     CVector2 m_cHeadingVector =  m_sRobotData.MaxSpeed  * (m_fAggregationPower * m_cAggregationVector.Normalize() +
                                                            (1.0-m_fAggregationPower) * CVector2(1.0f, m_sSensoryData.m_pcRNG->Uniform(CRange<CRadians>(-CRadians::PI, CRadians::PI))));

     // i add a strong random component to break stable aggregates of pair of robots.
     /*CVector2 m_cHeadingVector =  m_sRobotData.MaxSpeed  * (0.3f * m_cAggregationVector.Normalize() +
                                                            0.7f * CVector2(1.0f, m_sSensoryData.m_pcRNG->Uniform(CRange<CRadians>(-CRadians::PI, CRadians::PI))));*/

     WheelSpeedsFromHeadingVector(m_cHeadingVector, fLeftWheelSpeed, fRightWheelSpeed);
}

/******************************************************************************/
/******************************************************************************/

void CAggregateBehavior::PrintBehaviorIdentity()
{
    std::cout << "Aggregation taking over";
}

/******************************************************************************/
/******************************************************************************/
