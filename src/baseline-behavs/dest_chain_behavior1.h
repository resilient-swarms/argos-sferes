#ifndef DESTCHAINBEHAVIOR1_H_
#define DESTCHAINBEHAVIOR1_H_

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"
#include <argos3/core/utility/math/vector2.h>

/******************************************************************************/
/******************************************************************************/

using namespace argos;

class CDestChainBehavior1 : public CBehavior
{
public:
    //CDestChainBehavior1(unsigned* ptr_rabdataindex, CCI_RangeAndBearingActuator* m_pcRABA, CCI_ThymioLedsActuator* m_pcLeds, unsigned robot_id);

    CDestChainBehavior1(unsigned* ptr_rabdataindex, CCI_RangeAndBearingActuator* m_pcRABA, CCI_ThymioLedsActuator* m_pcLeds, unsigned robot_id, bool obsavoid = false);


    void SimulationStep()
    {

    }


    virtual bool TakeControl();
    virtual void Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

    virtual void PrintBehaviorIdentity();

protected:

    bool m_bObsAvoid;
    unsigned*      m_ptrRabDataIndex;
    CCI_RangeAndBearingActuator *m_pcRABA;
    CCI_ThymioLedsActuator*   m_pcLeds;
    unsigned       m_unRobotId;

};


/******************************************************************************/
/******************************************************************************/

#endif 
