#include "baseline-behavs-robids.h"
#include "baseline-behavs.h"
#include <argos3/core/simulator/entity/controllable_entity.h>

using namespace argos;

/****************************************/
/****************************************/

CBaselineBehavsRobIds::CBaselineBehavsRobIds()
{
    RegisterUserFunction<CBaselineBehavsRobIds,CThymioEntity>(&CBaselineBehavsRobIds::Draw);
}

/****************************************/
/****************************************/

void CBaselineBehavsRobIds::Draw(CThymioEntity& c_entity)
{
    /* The position of the text is expressed wrt the reference point of the robot
        * Tthe reference point is the center of its base.
        */
    DrawText(CVector3(0.0, 0.0, 0.3),   // position
             c_entity.GetId().c_str()); // text
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CBaselineBehavsRobIds, "baseline-behavs-robids")
