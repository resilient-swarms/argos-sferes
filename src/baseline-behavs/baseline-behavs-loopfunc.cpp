#include "baseline-behavs-loopfunc.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include "baseline-behavs.h"



/****************************************/
/****************************************/

CBaselineBehavsLoopFunctions::CBaselineBehavsLoopFunctions() :
    m_pcFloor(NULL),
    m_pcRNG(CRandom::CreateRNG("argos"))
{
}

/****************************************/
/****************************************/

void CBaselineBehavsLoopFunctions::Init(TConfigurationNode& t_node)
{
    try
    {
        /* Get a pointer to the floor entity */
        m_pcFloor = &GetSpace().GetFloorEntity();
    }
    catch(CARGoSException& ex)
            THROW_ARGOSEXCEPTION_NESTED("Error parsing loop function parameters.", ex);
}

/****************************************/
/****************************************/

void CBaselineBehavsLoopFunctions::Reset()
{
}

/****************************************/
/****************************************/

void CBaselineBehavsLoopFunctions::PreStep()
{
}

/****************************************/
/****************************************/

void CBaselineBehavsLoopFunctions::PostStep()
{
}

/****************************************/
/****************************************/

void CBaselineBehavsLoopFunctions::Destroy()
{
}

/****************************************/
/****************************************/

CColor CBaselineBehavsLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) // used to paint the floor by the floor entity
{
    return CColor::WHITE;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CBaselineBehavsLoopFunctions, "baseline-behavs-loop-functions")
