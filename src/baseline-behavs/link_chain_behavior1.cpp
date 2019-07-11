#include "link_chain_behavior1.h"
#include <argos3/core/utility/datatypes/color.h>

/******************************************************************************/
/******************************************************************************/

CLinkChainBehavior1::CLinkChainBehavior1(unsigned *ptr_rabdataindex, CCI_RangeAndBearingActuator *RABA, CCI_ThymioLedsActuator *Leds, unsigned RobotId,
                                         CVector3 src_pos,
                                         CVector3 dest_pos,
                                         CVector3 glbl_pos,
                                         CRadians glbl_theta) :
    m_ptrRabDataIndex(ptr_rabdataindex),
    m_pcRABA(RABA),
    m_pcLeds(Leds),
    m_unRobotId(RobotId),
    m_vecSrcPos(src_pos),
    m_vecDestPos(dest_pos),
    m_vecGlblPos(glbl_pos),
    m_radGlblTheta(glbl_theta)

{
}

/******************************************************************************/
/******************************************************************************/

bool CLinkChainBehavior1::TakeControl()
{

    bool controltaken(false);

    m_cDiffusionVector.Set(0.0f, 0.0f);

    size_t m_unRobotsInRange = 0;
    for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        /*if(m_sSensoryData.m_RABSensorData[i].Range < 100.0)
        {*/
            // controltaken = true;
            m_cDiffusionVector += CVector2(m_sSensoryData.m_RABSensorData[i].Range, m_sSensoryData.m_RABSensorData[i].HorizontalBearing);
            m_unRobotsInRange++;
        //}
    }

    if(m_unRobotsInRange > 0u)
        controltaken = true;

    if(controltaken)
        m_cDiffusionVector /= m_unRobotsInRange;


    return controltaken;



//    Real comm_range = 95.0; // in cm
//    unsigned chain_density = 0;

//    m_cDiffusionVector.Set(0.0f, 0.0f);

//    size_t m_unRobotsInRange = 0;
//    Real min_dist = 10000.0;
//    for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
//    {
//        if(m_sSensoryData.m_RABSensorData[i].Range < min_dist)
//        {
//            min_dist = m_sSensoryData.m_RABSensorData[i].Range;
//            m_cDiffusionVector = CVector2(m_sSensoryData.m_RABSensorData[i].Range, m_sSensoryData.m_RABSensorData[i].HorizontalBearing);
//            m_unRobotsInRange++;
//        }
//    }

//    m_radSrcBearing  = ((m_vecSrcPos - m_vecGlblPos).GetZAngle() - m_radGlblTheta).SignedNormalize();

//    /*CDegrees tmp = ToDegrees(m_radSrcBearing);
//    printf("\n m_radSrcBearing = %f; m_radSrcBearing = %f", tmp.SignedNormalize().GetValue(), tmp.UnsignedNormalize().GetValue());
//    m_radDestBearing = ((m_vecDestPos - m_vecGlblPos).GetZAngle() - m_radGlblTheta).SignedNormalize();
//    CDegrees tmp1 = ToDegrees(m_radDestBearing);
//    printf("\n m_radDestBearing = %f; m_radDestBearing = %f", tmp1.SignedNormalize().GetValue(), tmp1.UnsignedNormalize().GetValue());*/




//    if(m_unRobotsInRange > chain_density)
//        controltaken = true;


//    if(min_dist < comm_range)
//        m_bAttractionFlag = false;
//    else
//        m_bAttractionFlag = true;


//    return true;
}

/******************************************************************************/
/******************************************************************************/

void CLinkChainBehavior1::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
    CVector2 m_cHeadingVector(0,0);

    /*Add uniform noise of +/- 30 degrees to the src and destination bearings */
    m_radSrcBearing  = ((m_vecSrcPos - m_vecGlblPos).GetZAngle() - m_radGlblTheta).SignedNormalize();
    m_radSrcBearing  = CBehavior::m_sSensoryData.m_pcRNG->Uniform(CRange<CRadians>(m_radSrcBearing - CRadians::PI_OVER_SIX, m_radSrcBearing + CRadians::PI_OVER_SIX));

    m_radDestBearing = ((m_vecDestPos - m_vecGlblPos).GetZAngle() - m_radGlblTheta).SignedNormalize();
    m_radDestBearing  = CBehavior::m_sSensoryData.m_pcRNG->Uniform(CRange<CRadians>(m_radDestBearing - CRadians::PI_OVER_SIX, m_radDestBearing + CRadians::PI_OVER_SIX));


    CRadians bearing_diff = CRadians(NormalizedDifference(m_radSrcBearing, m_radDestBearing).GetAbsoluteValue()/2.0f);
    CRadians heading_bearing;

    if(m_radSrcBearing.GetAbsoluteValue() > m_radDestBearing.GetAbsoluteValue())
        heading_bearing = CRadians(NormalizedDifference(m_radSrcBearing, -bearing_diff));
    else
        heading_bearing = CRadians(NormalizedDifference(m_radDestBearing, -bearing_diff));

    //printf("\n Half DiffBearing = %f and new heading = %f", ToDegrees(bearing_diff).GetValue(), ToDegrees(heading_bearing).GetValue());



    //m_cHeadingVector = m_sRobotData.MaxSpeed * CVector2(1.0f, heading_bearing).Normalize();

    m_cHeadingVector =  m_sRobotData.MaxSpeed * (1.0f * CVector2(1.0f, heading_bearing)).Normalize();



//    if(m_bAttractionFlag)
//        m_cHeadingVector =  m_sRobotData.MaxSpeed * 0.5f  * (1.0f * m_cDiffusionVector.Normalize()) + m_sRobotData.MaxSpeed * 0.5f * CVector2(1.0f, heading_bearing).Normalize();
//    else
//        m_cHeadingVector =  m_sRobotData.MaxSpeed * 0.5f  * (-1.0f * m_cDiffusionVector.Normalize()) + m_sRobotData.MaxSpeed * 0.5f * CVector2(1.0f, heading_bearing).Normalize();

    WheelSpeedsFromHeadingVector(m_cHeadingVector, fLeftWheelSpeed, fRightWheelSpeed);

}

/******************************************************************************/
/******************************************************************************/

void CLinkChainBehavior1::PrintBehaviorIdentity()
{
    std::cout << "Link-chain behaviour - 1 taking over";
}

/******************************************************************************/
/******************************************************************************/
