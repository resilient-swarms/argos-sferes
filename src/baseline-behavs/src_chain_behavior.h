#ifndef SRCCHAINBEHAVIOR_H_
#define SRCCHAINBEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"
#include <argos3/core/utility/math/vector2.h>

/******************************************************************************/
/******************************************************************************/

using namespace argos;

class CSrcChainBehavior : public CBehavior
{
public:
    CSrcChainBehavior(unsigned* ptr_rabdataindex, CCI_RangeAndBearingActuator* m_pcRABA, CCI_ThymioLedsActuator* m_pcLeds, unsigned robot_id);

    void SimulationStep()
    {

    }


    virtual bool TakeControl();
    virtual void Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

    virtual void PrintBehaviorIdentity();

protected:
    unsigned*      m_ptrRabDataIndex;
    static bool    m_bSrcSignalOn;
    static int     m_iChildRobotId;
    CCI_RangeAndBearingActuator *m_pcRABA;
    CCI_ThymioLedsActuator*   m_pcLeds;
    unsigned       m_unRobotId;

};


/******************************************************************************/
/******************************************************************************/

#endif 
