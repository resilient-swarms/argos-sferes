#ifndef BASELINE_BEHAVS_LOOP_FUNCTIONS_H
#define BASELINE_BEHAVS_LOOP_FUNCTIONS_H


/* argos headers */

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/thymio/simulator/thymio_entity.h>

using namespace argos;

/* argos-sferes headers */
#include <src/core/base_loop_functions.h>


class CBaselineBehavsLoopFunctions : public BaseLoopFunctions
{

public:
   

   CBaselineBehavsLoopFunctions();
   virtual ~CBaselineBehavsLoopFunctions() {}
   virtual std::string get_controller_id()
   {
        return "bb";
   }

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
   virtual void PreStep();


private:

   CFloorEntity* m_pcFloor;

   std::string m_strOutput;
   std::ofstream m_cOutput;
   Real fArenaLength;

   bool swarm_chaining_behav;

};

#endif
