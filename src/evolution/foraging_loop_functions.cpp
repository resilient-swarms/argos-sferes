#include <src/core/fitness_functions.h>
#include "src/evolution/foraging_loop_functions.h"
#include "src/evolution/foraging_nn_controller.h"
#include <argos3/plugins/robots/thymio/simulator/thymio_entity.h>

/****************************************/
/****************************************/

CForagingLoopFunctions::CForagingLoopFunctions() : m_pcFloor(NULL)
{
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Init(TConfigurationNode &t_node)
{
   m_pcFloor = &GetSpace().GetFloorEntity();
   BaseEvolutionLoopFunctions::Init(t_node);
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Reset()
{

   BaseEvolutionLoopFunctions::Reset();

   m_cVisitedFood.clear();
   for (size_t i = 0; i < m_cFoodPos.size(); i++)
   {
      m_cVisitedFood.push_back(0);
   }
   numfoodCollected = 0;
   /* Check whether a robot is on a food item */
   CSpace::TMapPerType &m_thymios = GetSpace().GetEntitiesByType("Thymio");

   for (CSpace::TMapPerType::iterator it = m_thymios.begin(); it != m_thymios.end(); ++it)
   {
      /* Get handle to foot-bot entity and controller */
      argos::CThymioEntity &cThym = *any_cast<argos::CThymioEntity *>(it->second);
      ForagingThymioNN &cController = dynamic_cast<ForagingThymioNN &>(cThym.GetControllableEntity().GetController());
      cController.holdingFood = false;
   }
}

void CForagingLoopFunctions::reset_agent_positions()
{

   for (size_t m_unRobot = 0; m_unRobot < m_unNumberRobots; ++m_unRobot)
   {
      CEmbodiedEntity *entity = get_embodied_entity(m_unRobot);
      BaseController *cController = get_controller(m_unRobot);
      CPhysicsModel *model;
      CVector3 Position;
      CQuaternion Orientation;
      if (cController->b_damagedrobot && cController->FBehavior == BaseController::FaultBehavior::FAULT_SOFTWARE)
      {
         do
         {
            Position = CVector3(nest_x, m_pcRNG->Uniform(CRange<Real>(0.3, 1.8)), 0.0f);
#ifdef PRINTING
            std::cout << "Position1 " << Position << " trial " << m_unTrial << " time " << GetSpace().GetSimulationClock() << std::endl;
#endif

            Orientation.FromEulerAngles(CRadians::PI_OVER_TWO, //orient the agent vertically
                                        CRadians::ZERO,
                                        CRadians::ZERO);
         } while (!entity->MoveTo(
             Position,    // to this position
             Orientation, // with this orientation
             false        // this is not a check, leave the robot there
             ));
      }
      else
      {

         if (!entity->MoveTo(
                 m_vecInitSetup[m_unCurrentTrial][m_unRobot].Position,    // to this position
                 m_vecInitSetup[m_unCurrentTrial][m_unRobot].Orientation, // with this orientation
                 false                                                    // this is not a check, leave the robot there
                 ))
         {
            // std::cout << "trial" << m_unCurrentTrial << std::endl;
            // std::cout << "robot" << m_unRobot << std::endl;
            // std::cout<<"entity pos "<<entity->GetOriginAnchor().Position << std::endl;
            // std::cout<<"trial pos " <<m_vecInitSetup[m_unCurrentTrial][m_unRobot].Position<<std::endl;
         }
      }

      // std::cout<<"agent "<<m_unRobot<<std::endl;
      // std::cout<<m_vecInitSetup[m_unCurrentTrial][m_unRobot].Position<<std::endl;
      // std::cout<<m_vecInitSetup[m_unCurrentTrial][m_unRobot].Orientation<<std::endl;
      // for (size_t i=0; i < 4; ++i)
      // {
      //     try{
      //         model = &entity->GetPhysicsModel("dyn2d_"+std::to_string(i));
      //         //std::cout<<"Found the entity !"<<std::endl;
      //     }
      //     catch(argos::CARGoSException e){
      //         continue;
      //     }
      // }

      old_pos[m_unRobot] = entity->GetOriginAnchor().Position;
      curr_pos[m_unRobot] = old_pos[m_unRobot];
      CRadians zAngle = get_orientation(m_unRobot);
      curr_theta[m_unRobot] = zAngle;
      old_theta[m_unRobot] = zAngle;
   }
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Destroy()
{
}

/****************************************/
/****************************************/

CColor CForagingLoopFunctions::GetFloorColor(const CVector2 &c_position_on_plane)
{
   if (c_position_on_plane.GetX() < nest_x)
   {
      return CColor::WHITE;
   }
   for (UInt32 i = 0; i < m_cFoodPos.size(); ++i)
   {
      if ((c_position_on_plane - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius[i])
      {
         return CColor::BLACK;
      }
   }
   return CColor::GRAY50; // closest match to lab floor (110-160 ~ 127);
   //to mimic the variability of lighting and ground itself, further add noise ~ U(-20,20) via thymio ground sensors
   //this makes the agent robust to small deviations
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::PostStep()
{
   /* Logic to pick and drop food items */
   /*
    * If a robot is in the nest, drop the food item
    * If a robot is on a food item, pick it
    * Each robot can carry only one food item per time
    */
   /* Check whether a robot is on a food item */
   CSpace::TMapPerType &m_thymios = GetSpace().GetEntitiesByType("Thymio");
   size_t j = 0;
   for (CSpace::TMapPerType::iterator it = m_thymios.begin(); it != m_thymios.end(); ++it)
   {

      /* Get handle to foot-bot entity and controller */
      argos::CThymioEntity &cThym = *any_cast<argos::CThymioEntity *>(it->second);
      ForagingThymioNN &cController = dynamic_cast<ForagingThymioNN &>(cThym.GetControllableEntity().GetController());
      /* Get the position of the foot-bot on the ground as a CVector2 */
      CVector2 cPos;
      cPos.Set(cThym.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
               cThym.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      /* Get food data */
      //CFootBotForaging::SFoodData &sFoodData = cController.GetFoodData();
      /* The foot-bot has a food item */
      if (cController.holdingFood)
      {
         /* Check whether the foot-bot is in the nest */
         if (cPos.GetX() < nest_x)
         {
            /* Drop the food item */
            cController.holdingFood = false;
            /* Increase the food count */
            fitfun->fitness_per_trial[m_unCurrentTrial]++;
#ifdef PRINTING
            std::cout << "thymio" << j << " dropped off food. Total collected: " << fitfun->fitness_per_trial[m_unCurrentTrial] << std::endl;
#endif
            /* The floor texture must be updated */
            m_pcFloor->SetChanged();
         }
      }
      else
      {
         /* The foot-bot has no food item */
         /* Check whether the foot-bot is out of the nest */
         if (cPos.GetX() > nest_x)
         {
            /* Check whether the foot-bot is on a food item */
            bool bDone = false;
            for (size_t i = 0; i < m_cFoodPos.size() && !bDone; ++i)
            {
               if (m_cVisitedFood[i] == 0 && (cPos - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius[i])
               {
                  /* If so, we move that item out of sight */
                  //m_cFoodPos[i].Set(100.0f, 100.f);
                  /* The foot-bot is now carrying an item */
                  /* The thymio is now carrying an item */
                  cController.holdingFood = true;
                  /* the food has now been visited */
                  m_cVisitedFood[i] = HARVEST_TIME;
                  /* The floor texture must be updated */
                  m_pcFloor->SetChanged();
/* The floor texture must be updated */
//m_pcFloor->SetChanged();
/* We are done */
#ifdef PRINTING
                  std::cout << "thymio" << j << " picked up food item " << i << " from location " << m_cFoodPos[i] << std::endl;
#endif
                  bDone = true;
               }
            }
         }
      }
      ++j;
   }
   for (size_t f = 0; f < num_food; ++f)
   {
      m_cVisitedFood[f] = std::max(0, m_cVisitedFood[f] - 1);
#ifdef PRINTING

      std::cout << "Harvesting time for food  " << f << " on location " << m_cFoodPos[f] << "\n is now " << m_cVisitedFood[f] << std::endl;
#endif
   }

   BaseEvolutionLoopFunctions::PostStep();
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CForagingLoopFunctions, "foraging_loopfunctions" + std::string(TAG))
