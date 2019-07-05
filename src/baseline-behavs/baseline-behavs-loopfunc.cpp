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
    swarm_chaining_behav(false),
    swarm_chaining_behav1(false)
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
        else if(cController.GetExperimentType().SBehavior == CBaselineBehavs::ExperimentToRun::SWARM_CHAINING1)
        {
            swarm_chaining_behav1 = true;
            break;
        }
    }

    if(swarm_chaining_behav || swarm_chaining_behav1)
    {
        std::vector<CVector3> vec_robot_pos;
        for (CSpace::TMapPerType::iterator it = m_cThymio.begin(); it != m_cThymio.end(); ++it) //!TODO: Make sure the CSpace::TMapPerType does not change during a simulation (i.e it is not robot-position specific)
        {
            CThymioEntity &cThymio = *any_cast<CThymioEntity *>(it->second);
            vec_thymio.push_back(cThymio);
            vec_robot_pos.push_back(cThymio.GetEmbodiedEntity().GetOriginAnchor().Position);
        }

        Real max_sq_dist = -1000.0;
        for(size_t i = 0; i < vec_robot_pos.size(); ++i)
            for(size_t j = i+1; i < vec_robot_pos.size(); ++i)
                if(SquareDistance(vec_robot_pos[i],vec_robot_pos[j]) > max_sq_dist)
                {
                    max_sq_dist = SquareDistance(vec_robot_pos[i],vec_robot_pos[j]);
                    src_robot = i; dest_robot = j;
                }

        dynamic_cast<CBaselineBehavs&>(vec_thymio[src_robot].GetControllableEntity().GetController()).src_robot   = true;
        dynamic_cast<CBaselineBehavs&>(vec_thymio[dest_robot].GetControllableEntity().GetController()).dest_robot = true;



    }
}


/****************************************/
/****************************************/

void CBaselineBehavsLoopFunctions::PreStep()
{
    CVector3 src_pos, dest_pos;

    if(swarm_chaining_behav1)
    {
        src_pos  = vec_thymio[src_robot].GetEmbodiedEntity().GetOriginAnchor().Position;
        dest_pos = vec_thymio[dest_robot].GetEmbodiedEntity().GetOriginAnchor().Position;
    }

    CSpace::TMapPerType &m_cThymio = GetSpace().GetEntitiesByType("Thymio");

    size_t robotindex = 0;
    for (CSpace::TMapPerType::iterator it = m_cThymio.begin(); it != m_cThymio.end(); ++it)
    {
        CThymioEntity &cThymio = *any_cast<CThymioEntity *>(it->second);


        //Todo: Update to use get_orientation(size_t robot_index) of parent class
        CQuaternion quat = cThymio.GetEmbodiedEntity().GetOriginAnchor().Orientation;
        CRadians zAngle, yAngle, xAngle;
        quat.ToEulerAngles(zAngle,yAngle,xAngle);
        curr_theta[robotindex] = zAngle;


        curr_pos[robotindex] = cThymio.GetEmbodiedEntity().GetOriginAnchor().Position;


        if(swarm_chaining_behav1)
        {
            dynamic_cast<CBaselineBehavs&>(cThymio.GetControllableEntity().GetController()).dest_pos            = dest_pos;
            dynamic_cast<CBaselineBehavs&>(cThymio.GetControllableEntity().GetController()).src_pos             = src_pos;
            dynamic_cast<CBaselineBehavs&>(cThymio.GetControllableEntity().GetController()).global_orientation  = zAngle;
            dynamic_cast<CBaselineBehavs&>(cThymio.GetControllableEntity().GetController()).global_position     = curr_pos[robotindex];
        }

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
