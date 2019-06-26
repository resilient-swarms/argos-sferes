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
    CSrcChainBehavior(unsigned* ptr_rabdataindex, CCI_RangeAndBearingActuator* m_pcRABA, CCI_ThymioLedsActuator* m_pcLeds);

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
