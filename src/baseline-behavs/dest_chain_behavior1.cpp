#include "dest_chain_behavior1.h"

/******************************************************************************/
/******************************************************************************/

CDestChainBehavior1::CDestChainBehavior1(unsigned *ptr_rabdataindex, CCI_RangeAndBearingActuator *RABA, CCI_ThymioLedsActuator *Leds, unsigned robot_id, bool obsavoid) :
    m_ptrRabDataIndex(ptr_rabdataindex),
    m_pcRABA(RABA),
    m_pcLeds(Leds),
    m_unRobotId(robot_id),
    m_bObsAvoid(obsavoid)
{
}

/******************************************************************************/
/******************************************************************************/

bool CDestChainBehavior1::TakeControl()
{
    return true;
}

/******************************************************************************/
/******************************************************************************/


void CDestChainBehavior1::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
    m_pcLeds->SetColor(CColor::YELLOW);

    m_pcRABA->SetData(*m_ptrRabDataIndex, CBehavior::m_sRobotData.NEST_BEACON_SIGNAL_MARKER);
    (*m_ptrRabDataIndex)++;

    if(m_bObsAvoid)
    {
        Real speed_reduction_factor = 0.25;

        for(size_t i = 0; i < 5; ++i) // considering the front 5 proximity sensors
            if(m_sSensoryData.m_ProximitySensorData[i].Value >= 0.1)
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

                fLeftWheelSpeed  = fLeftWheelSpeed * speed_reduction_factor;
                fRightWheelSpeed = fRightWheelSpeed * speed_reduction_factor;
                return;
            }


        CVector2 m_cDiffusionVector = CVector2(0.0f, 0.0f);

        size_t m_unRobotsInRange = 0;
        Real min_dist = 5000.0; // destination keeps atleast X cm from nbrs
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
        {
            CVector2 headingvector = m_sRobotData.MaxSpeed * speed_reduction_factor  * (-1.0f * m_cDiffusionVector.Normalize());
            WheelSpeedsFromHeadingVector(headingvector, fLeftWheelSpeed, fRightWheelSpeed);
        }
        else
        {
            if (m_sSensoryData.m_pcRNG->Uniform(CRange<Real>(0.0, 1.0)) < 0.1)
            {
                Real fSpeed;

                CRadians angle = m_sSensoryData.m_pcRNG->Uniform(CRange<CRadians>(-CRadians::PI, CRadians::PI));


                fSpeed = angle.GetAbsoluteValue() * m_sRobotData.HALF_INTERWHEEL_DISTANCE;
                fSpeed = fSpeed * 100.0; // converting to cm/s - as used in SetLinearVelocity
                fSpeed = Min<Real>(fSpeed, m_sRobotData.MaxSpeed);


                if(angle.GetValue() > 0.0f) //turn right
                {
                    fLeftWheelSpeed  = fSpeed;
                    fRightWheelSpeed = -fSpeed;
                }
                else
                {
                    fLeftWheelSpeed  = -fSpeed;
                    fRightWheelSpeed =  fSpeed;
                }
            }
            else
            {
                    fLeftWheelSpeed  = m_sRobotData.MaxSpeed;
                    fRightWheelSpeed = m_sRobotData.MaxSpeed;
            }

            fLeftWheelSpeed  = fLeftWheelSpeed * speed_reduction_factor;
            fRightWheelSpeed = fRightWheelSpeed * speed_reduction_factor;
        }
    }
    else
    {
        /*
         * The destination is always stationary
         */
        fLeftWheelSpeed  = 0.0;
        fRightWheelSpeed = 0.0;
    }
}

/******************************************************************************/
/******************************************************************************/

void CDestChainBehavior1::PrintBehaviorIdentity()
{
    std::cout << "Dest-chain behaviour - 1 taking over";
}

/******************************************************************************/
/******************************************************************************/
