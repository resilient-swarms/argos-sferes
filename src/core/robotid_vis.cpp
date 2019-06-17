#include "robotid_vis.h"
#include <argos3/core/simulator/entity/controllable_entity.h>

using namespace argos;

/****************************************/
/****************************************/

RobotIDVis::RobotIDVis()
{
    RegisterUserFunction<RobotIDVis,CThymioEntity>(&RobotIDVis::Draw);
}

/****************************************/
/****************************************/

void RobotIDVis::Draw(CThymioEntity& c_entity)
{
    /* The position of the text is expressed wrt the reference point of the robot
        * Tthe reference point is the center of its base.
        */
    DrawText(CVector3(0.0, 0.0, 0.3),   // position
             c_entity.GetId().c_str()); // text
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(RobotIDVis, "robotid_vis")
