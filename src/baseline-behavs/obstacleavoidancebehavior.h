#ifndef OBSTACLEAVOIDANCRBEHAVIOR_H_
#define OBSTACLEAVOIDANCRBEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"
#include <argos3/core/utility/math/vector2.h>

/******************************************************************************/
/******************************************************************************/


using namespace argos;

class CProximitySensorEquippedEntity;

class CObstacleAvoidanceBehavior : public CBehavior
{
public:
    CObstacleAvoidanceBehavior(Real m_fProximitySensorThreshold);
    CObstacleAvoidanceBehavior(Real m_fProximitySensorThreshold, CRadians m_cGoStraightAngleThreshold);

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
};


/******************************************************************************/
/******************************************************************************/

#endif 
