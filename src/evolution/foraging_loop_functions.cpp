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

   for (size_t i=0; i < m_unNumberRobots; ++i)
   {
      /* Get handle to foot-bot entity and controller */
      argos::CThymioEntity* cThym = m_pcvecRobot[i];
      ForagingThymioNN &cController = dynamic_cast<ForagingThymioNN &>(cThym->GetControllableEntity().GetController());
      cController.holdingFood = false;
   }
}

std::vector<size_t> CForagingLoopFunctions::priority_robotplacement()
{
   // in base loop functions, simply 0..N-1
   std::vector<size_t> indices;
   for (size_t index = 0; index < m_unNumberRobots; ++index)
   {
      BaseController *cController = get_controller(index);
      if (cController->b_damagedrobot && (cController->FBehavior == BaseController::FaultBehavior::FAULT_SOFTWARE || cController->FBehavior == BaseController::FaultBehavior::FAULT_SOFTWARE_NEIGHBOURHOOD || cController->FBehavior == BaseController::FaultBehavior::FAULT_SOFTWARE_FOOD))
      {
         indices.insert(indices.begin(), index);
      }
      else
      {
         indices.push_back(index);
      }
   }
   return indices;
}

void CForagingLoopFunctions::try_robot_position(CVector3 &Position, CQuaternion &Orientation, const CRange<Real> x_range, const CRange<Real> y_range, const size_t m_unRobot, size_t &num_tries)
{

   CEmbodiedEntity *entity = get_embodied_entity(m_unRobot);
   ForagingThymioNN *cController = dynamic_cast<ForagingThymioNN*>(get_controller(m_unRobot));
   if (cController->b_damagedrobot && cController->FBehavior == BaseController::FaultBehavior::FAULT_SOFTWARE)
   {
      do
      {
         Position = CVector3(nest_x, m_pcRNG->Uniform(CRange<Real>(0.3, 1.8)), 0.0f);
#ifdef PRINTING
         std::cout << "Positioning agent with software" << std::endl;
         std::cout << "Nest x-position is " << nest_x << std::endl;
         std::cout << "Position1 " << Position << std::endl;
#endif

         Orientation.FromEulerAngles(CRadians::PI_OVER_TWO, //orient the agent vertically
                                     CRadians::ZERO,
                                     CRadians::ZERO);
         ++num_tries;
      } while (!entity->MoveTo(
          Position,    // to this position
          Orientation, // with this orientation
          false        // this is not a check, leave the robot there
          ));
   }
   else if (cController->b_damagedrobot && cController->FBehavior == BaseController::FaultBehavior::FAULT_SOFTWARE_FOOD)
   {

      do
      {
         // select a food location
         int i = cController->foodID;
         Position = CVector3(m_cFoodPos[i].GetX(), m_cFoodPos[i].GetY(), 0.0);
#ifdef PRINTING
         std::cout << "Positioning agent with software_food fault" << std::endl;
         std::cout << "Food position is " << m_cFoodPos[i] << std::endl;
         std::cout << "Position1 " << Position << std::endl;
#endif

         Orientation.FromEulerAngles(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                     CRadians::ZERO,
                                     CRadians::ZERO);
         ++num_tries;
      } while (!entity->MoveTo(
          Position,    // to this position
          Orientation, // with this orientation
          false        // this is not a check, leave the robot there
          ));
   }
   else if (cController->b_damagedrobot && cController->FBehavior == BaseController::FaultBehavior::FAULT_SOFTWARE_NEIGHBOURHOOD)
   {
      do
      {
         // select a food location
         int i = cController->foodID;
         // randomly add or subtract a number in (radius, radius + 0.10) for X and Y
         int sign_x = m_pcRNG->Uniform(CRange<int>(0, 2));
         int sign_y = m_pcRNG->Uniform(CRange<int>(0, 2));
         Real radius = std::sqrt(m_fFoodSquareRadius[i]);
         Real added_x = m_pcRNG->Uniform(CRange<Real>(radius, radius + 0.10));
         Real added_y = m_pcRNG->Uniform(CRange<Real>(radius, radius + 0.10));
         if (sign_x == 0)
         {
            added_x = -added_x;
         }
         if (sign_y == 0)
         {
            added_y = -added_y;
         }
         Position = CVector3(m_cFoodPos[i].GetX() + added_x , m_cFoodPos[i].GetY() + added_y, 0.0);
#ifdef PRINTING
         std::cout << "Positioning agent with software_neighbourhood fault" << std::endl;
         std::cout << "Food position is " << m_cFoodPos[i] << std::endl;
         std::cout << "Food radius is " << radius << std::endl;
         std::cout << "Position1 " << Position << std::endl;
#endif

         Orientation.FromEulerAngles(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                     CRadians::ZERO,
                                     CRadians::ZERO);
         ++num_tries;
      } while (!entity->MoveTo(
          Position,    // to this position
          Orientation, // with this orientation
          false        // this is not a check, leave the robot there
          ));
   }
   else
   {
      BaseLoopFunctions::try_robot_position(Position, Orientation, x_range, y_range, m_unRobot, num_tries);
   }
}
//    void CForagingLoopFunctions::reset_agent_positions()
//    {

//       for (size_t m_unRobot = 0; m_unRobot < m_unNumberRobots; ++m_unRobot)
//       {
//          CEmbodiedEntity *entity = get_embodied_entity(m_unRobot);
//          BaseController *cController = get_controller(m_unRobot);
//          CPhysicsModel *model;
//          CVector3 Position;
//          CQuaternion Orientation;
//          if (cController->b_damagedrobot)
//          {
//             if (cController->FBehavior == BaseController::FaultBehavior::FAULT_SOFTWARE)
//             {
//                do
//                {
//                   Position = CVector3(nest_x, m_pcRNG->Uniform(CRange<Real>(0.3, 1.8)), 0.0f);
// #ifdef PRINTING
//                   std::cout << "Position1 " << Position << " trial " << m_unTrial << " time " << GetSpace().GetSimulationClock() << std::endl;
// #endif

//                   Orientation.FromEulerAngles(CRadians::PI_OVER_TWO, //orient the agent vertically
//                                               CRadians::ZERO,
//                                               CRadians::ZERO);
//                } while (!entity->MoveTo(
//                    Position,    // to this position
//                    Orientation, // with this orientation
//                    false        // this is not a check, leave the robot there
//                    ));
//             }
//             else if (cController->FBehavior == BaseController::FaultBehavior::FAULT_SOFTWARE_FOOD)
//             {
//                // 1. randomly select a food
//                size_t i = m_pcRNG->Uniform(CRange<int>(0, num_food));
//                Position = CVector3(m_cFoodPos.GetX(), m_cFoodPos.GetY(), 0.0);
//                // 2. check other agents not on the same location; the ones that are will be moved
//                do
//                {
//                   Position = CVector3(m_cFoodPos, m_pcRNG->Uniform(CRange<Real>(0.3, 1.8)), 0.0f);
// #ifdef PRINTING
//                   std::cout << "Position1 " << Position << " trial " << m_unTrial << " time " << GetSpace().GetSimulationClock() << std::endl;
// #endif

//                   Orientation.FromEulerAngles(CRadians::PI_OVER_TWO, //orient the agent vertically
//                                               CRadians::ZERO,
//                                               CRadians::ZERO);
//                } while (!entity->MoveTo(
//                    Position,    // to this position
//                    Orientation, // with this orientation
//                    false        // this is not a check, leave the robot there
//                    ));
//             }
//             else if (cController->FBehavior == BaseController::FaultBehavior::FAULT_SOFTWARE_NEIGHBOURHOOD)
//             {
//             }
//          }
//          else
//          {

//             if (!entity->MoveTo(
//                     m_vecInitSetup[m_unCurrentTrial][m_unRobot].Position,    // to this position
//                     m_vecInitSetup[m_unCurrentTrial][m_unRobot].Orientation, // with this orientation
//                     false                                                    // this is not a check, leave the robot there
//                     ))
//             {
//                // std::cout << "trial" << m_unCurrentTrial << std::endl;
//                // std::cout << "robot" << m_unRobot << std::endl;
//                // std::cout<<"entity pos "<<entity->GetOriginAnchor().Position << std::endl;
//                // std::cout<<"trial pos " <<m_vecInitSetup[m_unCurrentTrial][m_unRobot].Position<<std::endl;
//             }
//          }

//          // std::cout<<"agent "<<m_unRobot<<std::endl;
//          // std::cout<<m_vecInitSetup[m_unCurrentTrial][m_unRobot].Position<<std::endl;
//          // std::cout<<m_vecInitSetup[m_unCurrentTrial][m_unRobot].Orientation<<std::endl;
//          // for (size_t i=0; i < 4; ++i)
//          // {
//          //     try{
//          //         model = &entity->GetPhysicsModel("dyn2d_"+std::to_string(i));
//          //         //std::cout<<"Found the entity !"<<std::endl;
//          //     }
//          //     catch(argos::CARGoSException e){
//          //         continue;
//          //     }
//          // }

//          old_pos[m_unRobot] = entity->GetOriginAnchor().Position;
//          curr_pos[m_unRobot] = old_pos[m_unRobot];
//          CRadians zAngle = get_orientation(m_unRobot);
//          curr_theta[m_unRobot] = zAngle;
//          old_theta[m_unRobot] = zAngle;
//       }
//    }

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
   for (size_t j=0; j < m_unNumberRobots; ++j)
   {

      /* Get handle to foot-bot entity and controller */
      argos::CThymioEntity* cThym = m_pcvecRobot[j];
      ForagingThymioNN &cController = dynamic_cast<ForagingThymioNN &>(cThym->GetControllableEntity().GetController());
      /* Get the position of the foot-bot on the ground as a CVector2 */
      CVector2 cPos;
      cPos.Set(cThym->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
               cThym->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

      /* The thymio has a food item and does not drop it due to software fault*/
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
            std::cout << cThym->GetId() << " dropped off food. Total collected: " << fitfun->fitness_per_trial[m_unCurrentTrial] << std::endl;
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
                  /* The thymio is now carrying an item, unless it did not pick it up due to software failure */
                  if (cController.b_damagedrobot && cController.FBehavior == BaseController::FaultBehavior::FAULT_SOFTWARE_FOOD)
                  {
                     cController.holdingFood = false;
#ifdef PRINTING
                     std::cout << cThym->GetId() << " could not pick up food due to software_food fault " << std::endl;
#endif
                  }
                  else
                  {
                     cController.holdingFood = true;
#ifdef PRINTING
                     std::cout << cThym->GetId() << " is now holding food " << std::endl;
#endif
                  }
                  /* the food has now been visited */
                  m_cVisitedFood[i] = HARVEST_TIME;
                  /* The floor texture must be updated */
                  m_pcFloor->SetChanged();
/* The floor texture must be updated */
//m_pcFloor->SetChanged();
/* We are done */
#ifdef PRINTING
                  std::cout << cThym->GetId() << " picked up food item " << i << " from location " << m_cFoodPos[i] << std::endl;
#endif
                  bDone = true;
               }
            }
         }
      }
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
