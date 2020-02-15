#include <src/evolution/foraging_vis.h>
#include <src/evolution/foraging_nn_controller.h>



/****************************************/
/****************************************/

ForagingVis::ForagingVis() {
   RegisterUserFunction<ForagingVis,argos::CThymioEntity&>(&ForagingVis::Draw);
}

/****************************************/
/****************************************/

void ForagingVis::Draw(argos::CThymioEntity& c_entity) {
   ForagingThymioNN& cController = dynamic_cast<ForagingThymioNN&>(c_entity.GetControllableEntity().GetController());
   if(cController.holdingFood) {
      DrawCylinder(
         CVector3(0.0f, 0.0f, 0.3f), 
         CQuaternion(),
         0.1f,
         0.05f,
         CColor::BLACK);
   }
   DrawText(CVector3(0.0, 0.0, 0.3),   // position
             c_entity.GetId().c_str()); // text
}

/****************************************/
/****************************************/
REGISTER_QTOPENGL_USER_FUNCTIONS(ForagingVis, "foraging_vis")

