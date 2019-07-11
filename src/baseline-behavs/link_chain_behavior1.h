#ifndef LINKCHAINBEHAVIOR1_H_
#define LINKCHAINBEHAVIOR1_H_

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"
#include <argos3/core/utility/math/vector2.h>

/******************************************************************************/
/******************************************************************************/

using namespace argos;

class CLinkChainBehavior1 : public CBehavior
{
public:
    CLinkChainBehavior1(unsigned* ptr_rabdataindex, CCI_RangeAndBearingActuator* m_pcRABA, CCI_ThymioLedsActuator* m_pcLeds, unsigned RobotId,
                       CVector3 src_pos, CVector3 dest_pos, CVector3 glbl_pos, CRadians glbl_theta);

    void SimulationStep()
    {

    }


    virtual bool TakeControl();
    virtual void Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

    virtual void PrintBehaviorIdentity();

protected:
    unsigned*      m_ptrRabDataIndex;
    CCI_RangeAndBearingActuator *m_pcRABA;
    CCI_ThymioLedsActuator*   m_pcLeds;
    unsigned       m_unRobotId;

    CVector3 m_vecSrcPos, m_vecDestPos, m_vecGlblPos;
    CRadians m_radGlblTheta;
    CRadians m_radSrcBearing, m_radDestBearing;

    CVector2 m_cDiffusionVector;
    bool m_bAttractionFlag;
};


/******************************************************************************/
/******************************************************************************/

#endif 
