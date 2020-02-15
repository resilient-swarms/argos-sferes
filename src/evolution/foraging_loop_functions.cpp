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
   BaseEvolutionLoopFunctions::Init(t_node);
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Reset()
{
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
   return CColor::GRAY50; // closest match to lab floor (110-160 ~ 127)
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
            std::cout << "thymio" << j << " dropped off food. Total collected: " << fitfun->fitness_per_trial[m_unCurrentTrial]  << std::endl;
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
               if ((cPos - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius[i])
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
      m_cVisitedFood[f] = std::max((size_t)0, m_cVisitedFood[f] - 1);
#ifdef PRINTING

      std::cout << "Harvesting time is now for food  " << f << " on location " << m_cFoodPos[f] << "\n is now" << m_cVisitedFood[f] << std::endl;
#endif
   }
   
   BaseEvolutionLoopFunctions::PostStep();
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CForagingLoopFunctions, "foraging_loopfunctions" + std::string(TAG))
