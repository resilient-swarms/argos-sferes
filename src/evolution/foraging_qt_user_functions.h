#ifndef FORAGING_QT_USER_FUNCTIONS_H
#define FORAGING_QT_USER_FUNCTIONS_H
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/thymio/simulator/thymio_entity.h>

using namespace argos;


class CForagingQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CForagingQTUserFunctions();

   virtual ~CForagingQTUserFunctions() {}

   void Draw(argos::CThymioEntity& c_entity);
   
};

#endif
