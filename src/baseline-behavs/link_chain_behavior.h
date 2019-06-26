#ifndef LINKCHAINBEHAVIOR_H_
#define LINKCHAINBEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"
#include <argos3/core/utility/math/vector2.h>

/******************************************************************************/
/******************************************************************************/

using namespace argos;

class CLinkChainBehavior : public CBehavior
{
public:
    CLinkChainBehavior(unsigned* ptr_rabdataindex, CCI_RangeAndBearingActuator* m_pcRABA, CCI_ThymioLedsActuator* m_pcLeds, unsigned RobotId);

    void SimulationStep()
    {

    }


    virtual bool TakeControl();
    virtual void Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

    virtual void PrintBehaviorIdentity();

protected:
    unsigned*      m_ptrRabDataIndex;
    static bool    m_bBeaconSignalOn;
    static int     m_iParentRobotId;
    static int     m_iChildRobotId;
    CCI_RangeAndBearingActuator *m_pcRABA;
    CCI_ThymioLedsActuator*   m_pcLeds;
    unsigned       m_unRobotId;

};


/******************************************************************************/
/******************************************************************************/

#endif 
