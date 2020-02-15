#include <src/evolution/foraging_qt_user_functions.h>
#include <src/evolution/foraging_nn_controller.h>



/****************************************/
/****************************************/

CForagingQTUserFunctions::CForagingQTUserFunctions() {
   RegisterUserFunction<CForagingQTUserFunctions,argos::CThymioEntity&>(&CForagingQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CForagingQTUserFunctions::Draw(argos::CThymioEntity& c_entity) {
   ForagingThymioNN& cController = dynamic_cast<ForagingThymioNN&>(c_entity.GetControllableEntity().GetController());
   if(cController.holdingFood) {
      DrawCylinder(
         CVector3(0.0f, 0.0f, 0.3f), 
         CQuaternion(),
         0.1f,
         0.05f,
         CColor::BLACK);
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CForagingQTUserFunctions, "foraging_qt_user_functions")
