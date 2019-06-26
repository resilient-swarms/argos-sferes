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
    CLinkChainBehavior(unsigned* ptr_rabdataindex, CCI_RangeAndBearingActuator* m_pcRABA, CCI_ThymioLedsActuator* m_pcLeds);

    void SimulationStep()
    {

    }


    virtual bool TakeControl();
    virtual void Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

    virtual void PrintBehaviorIdentity();

protected:
    unsigned*      m_ptrRabDataIndex;
    bool           m_bSrcSignalOn;
    CCI_RangeAndBearingActuator *m_pcRABA;
    CCI_ThymioLedsActuator*   m_pcLeds;

};


/******************************************************************************/
/******************************************************************************/

#endif 
