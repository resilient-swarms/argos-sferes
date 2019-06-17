#ifndef BASELINEBEHAVS_ROB_IDS_H
#define BASELINEBEHAVS_ROB_IDS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/thymio/simulator/thymio_entity.h>

using namespace argos;

class RobotIDVis : public CQTOpenGLUserFunctions {

public:

   RobotIDVis();

   virtual ~RobotIDVis() {}

   void Draw(CThymioEntity &c_entity);
   
};

#endif
