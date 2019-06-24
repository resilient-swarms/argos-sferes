#include "baseline-behavs-loopfunc.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include "baseline-behavs.h"

#include <src/core/fitness_functions.h>

/****************************************/
/****************************************/

CBaselineBehavsLoopFunctions::CBaselineBehavsLoopFunctions() :
    BaseLoopFunctions(),
    m_pcFloor(NULL),
    swarm_chaining_behav(false)
{
   
}

/****************************************/
/****************************************/

void CBaselineBehavsLoopFunctions::Init(TConfigurationNode &t_node)
{

    BaseLoopFunctions::Init(t_node);

    try
    {
        /* Get a pointer to the floor entity */
        m_pcFloor = &GetSpace().GetFloorEntity();
    }
    catch (CARGoSException &ex)
        THROW_ARGOSEXCEPTION_NESTED("Error parsing loop function parameters.", ex);

    CSpace::TMapPerType &m_cThymio = GetSpace().GetEntitiesByType("Thymio");
    for (CSpace::TMapPerType::iterator it = m_cThymio.begin(); it != m_cThymio.end(); ++it)
    {
        /* Get handle to thymio entity and controller */
         CThymioEntity& cThymio = *any_cast<CThymioEntity*>(it->second);
         CBaselineBehavs& cController = dynamic_cast<CBaselineBehavs&>(cThymio.GetControllableEntity().GetController());

         if(cController.GetExperimentType().SBehavior == CBaselineBehavs::ExperimentToRun::SWARM_CHAINING)
         {
             swarm_chaining_behav = true;
             break;
         }
    }
}


/****************************************/
/****************************************/

void CBaselineBehavsLoopFunctions::PreStep()
{
    CSpace::TMapPerType &m_cThymio = GetSpace().GetEntitiesByType("Thymio");

    size_t robotindex = 0;
    for (CSpace::TMapPerType::iterator it = m_cThymio.begin(); it != m_cThymio.end(); ++it) //!TODO: Make sure the CSpace::TMapPerType does not change during a simulation (i.e it is not robot-position specific)
    {
        CThymioEntity &cThymio = *any_cast<CThymioEntity *>(it->second);

        CVector3 axis;
        cThymio.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToAngleAxis(curr_theta[robotindex], axis);

        curr_pos[robotindex] = cThymio.GetEmbodiedEntity().GetOriginAnchor().Position;
         ++robotindex;
    }
   
}



/****************************************/
/****************************************/

void CBaselineBehavsLoopFunctions::Destroy()
{
}

/****************************************/
/****************************************/

CColor CBaselineBehavsLoopFunctions::GetFloorColor(const CVector2 &c_position_on_plane) // used to paint the floor by the floor entity
{
    CVector2 src(1.0,1.0), dest(6.5,6.5);
    if(swarm_chaining_behav)
    {
        if((c_position_on_plane - src).SquareLength() < 0.1f)
            return CColor::GRAY50;

        if((c_position_on_plane - dest).SquareLength() < 0.1f)
            return CColor::BLACK;
    }

    return CColor::WHITE;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CBaselineBehavsLoopFunctions, "baseline-behavs-loop-functions")
