#include "src_chain_behavior1.h"
#include <argos3/core/utility/datatypes/color.h>


/******************************************************************************/
/******************************************************************************/

CSrcChainBehavior1::CSrcChainBehavior1(unsigned *ptr_rabdataindex, CCI_RangeAndBearingActuator *RABA, CCI_ThymioLedsActuator *Leds, unsigned robot_id) :
    m_ptrRabDataIndex(ptr_rabdataindex),
    m_pcRABA(RABA),
    m_pcLeds(Leds),
    m_unRobotId(robot_id)

{
}

/******************************************************************************/
/******************************************************************************/
    
bool CSrcChainBehavior1::TakeControl()
{
    return true;
}

/******************************************************************************/
/******************************************************************************/

void CSrcChainBehavior1::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
    /*
     * The source is always stationary
     */

    fLeftWheelSpeed  = 0.0;
    fRightWheelSpeed = 0.0;

    m_pcLeds->SetColor(CColor::RED);

    m_pcRABA->SetData(*m_ptrRabDataIndex, CBehavior::m_sRobotData.BEACON_SIGNAL_MARKER);
    (*m_ptrRabDataIndex)++;
}

/******************************************************************************/
/******************************************************************************/

void CSrcChainBehavior1::PrintBehaviorIdentity()
{
    std::cout << "Src-chain behaviour - 1 taking over";
}

/******************************************************************************/
/******************************************************************************/
