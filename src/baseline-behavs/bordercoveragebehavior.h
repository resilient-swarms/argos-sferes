#ifndef BORDERCOVERAGEBEHAVIOR_H_
#define BORDERCOVERAGEBEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"
#include <argos3/core/utility/math/vector2.h>

/******************************************************************************/
/******************************************************************************/


using namespace argos;

class CProximitySensorEquippedEntity;

class CBorderCoverageBehavior : public CBehavior
{
public:
    CBorderCoverageBehavior(Real m_fProximitySensorThreshold, UInt64* BorderCoverageStartTime);
    virtual bool TakeControl();
    virtual void Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

    void SimulationStep()
    {
    }

    virtual void PrintBehaviorIdentity();


protected:
    Real           m_fProximitySensorThreshold;
    CRadians       m_cGoStraightAngleThreshold;
    CVector2       m_cDiffusionVector;
    UInt64         *m_ptBorderCoverageStartTime;

};


/******************************************************************************/
/******************************************************************************/

#endif
