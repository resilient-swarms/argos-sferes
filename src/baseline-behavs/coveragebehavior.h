#ifndef COVERAGEBEHAVIOR_H_
#define COVERAGEBEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"
#include <argos3/core/utility/math/vector2.h>

/******************************************************************************/
/******************************************************************************/

using namespace argos;

class CCoverageBehavior : public CBehavior
{
public:
    CCoverageBehavior(Real m_fRangeAndBearing_RangeThreshold);

    void SimulationStep()
    {

    }


    virtual bool TakeControl();
    virtual void Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

    virtual void PrintBehaviorIdentity();

protected:
    Real           m_fRangeAndBearing_RangeThreshold;
    CVector2       m_cCoverageVector;
    unsigned       m_unRobotsInRange;

};


/******************************************************************************/
/******************************************************************************/

#endif 
