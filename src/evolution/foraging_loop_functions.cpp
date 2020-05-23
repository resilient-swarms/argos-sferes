#include <src/core/fitness_functions.h>
#include "src/evolution/foraging_loop_functions.h"
#include "src/evolution/foraging_nn_controller.h"
#include <argos3/plugins/robots/thymio/simulator/thymio_entity.h>

/****************************************/
/****************************************/

CForagingLoopFunctions::CForagingLoopFunctions() : m_pcFloor(NULL), virtual_energy(NULL)
{
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Init(TConfigurationNode &t_node)
{
   m_pcFloor = &GetSpace().GetFloorEntity();
   BaseEvolutionLoopFunctions::Init(t_node);
   /* process virtual_energy */
   std::string use_virtual;
   try
   {
      // using virtual energy or no ?
      GetNodeAttribute(t_node, "use_virtual", use_virtual);
      if (use_virtual == "True")
      {
         // init is set to the number of steps to travel 1 m (approx half of the arena)
         // 100 - 6*num_steps = 0
         ForagingThymioNN *cController = dynamic_cast<ForagingThymioNN *>(get_controller(0)); // index 0 because any index will do
         float num_ticks_per_s = 1.0 / tick_time;
         float steps_to_1m = (100.0 / cController->m_sWheelTurningParams.MaxSpeed) * num_ticks_per_s; //maxspeed in cm/s
         virtual_energy = new VirtualEnergy(this->m_unNumberRobots, steps_to_1m);
      }
      // TODO: create some statistics files in this folder

      // using virtual energy or no ?
      std::string track_stats;
      GetNodeAttribute(t_node, "track_stats", track_stats);
      if (track_stats == "True")
      {
         // init is set to the number of steps to travel 1 m (approx half of the arena)
         // 100 - 6*num_steps = 0
         stats = new ForagingStats(output_folder, m_unNumberRobots, m_unNumberTrials);
      }
   }
   catch (CARGoSException &ex)
   {
      THROW_ARGOSEXCEPTION_NESTED("Error in virtual init ", ex);
   }
#if HETEROGENEOUS & !PRINT_NETWORK
   try
   {
      GetNodeAttribute(t_node, "ticks_per_subtrial", ticks_per_subtrial);
   }
   catch (CARGoSException &ex)
   {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing num ticks per subtrial", ex);
   }

   try
   {
      GetNodeAttribute(t_node, "num_subtrials", num_subtrials);
   }
   catch (CARGoSException &ex)
   {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing number of subtrials", ex);
   }
   std::string network_config, network_binary;
   try
   {
      GetNodeAttribute(t_node, "network_config", network_config);
   }
   catch (CARGoSException &ex)
   {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing network config", ex);
   }
   global::argossim_config_name.push_back(network_config);
   try
   {
      GetNodeAttribute(t_node, "network_binary", global::argossim_bin_name);
   }
   catch (CARGoSException &ex)
   {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing networkd binary", ex);
   }

   // select new controller now
   opt.optimize_init<ControllerEval>(state_fun);
   /* initial phase: select controller for one robot and then put others with the same as well */
   argos::CThymioEntity *cThym = m_pcvecRobot[0];
   ForagingThymioNN &cController = dynamic_cast<ForagingThymioNN &>(cThym->GetControllableEntity().GetController());
   cController.worker = ForagingThymioNN::Worker(num_subtrials, 0);
   Eigen::VectorXd result = opt.select_sample<ControllerEval>();
   cController.worker.new_sample = result.head(BEHAV_DIM);
   std::vector<double> bd(result.data(), result.data() + result.rows() * result.cols());
   cController.select_net(bd, num_subtrials, ticks_per_subtrial);
   for (size_t i = 1; i < m_unNumberRobots; ++i)
   {
      argos::CThymioEntity *cThym = m_pcvecRobot[i];
      ForagingThymioNN &cController = dynamic_cast<ForagingThymioNN &>(cThym->GetControllableEntity().GetController());
      cController.worker = ForagingThymioNN::Worker(num_subtrials, i);
      cController.worker.new_sample = result.head(BEHAV_DIM);
      cController.select_net(bd, num_subtrials, ticks_per_subtrial);
      // reset the controller (food_items_collected,)
      //cController.Reset();// will happen automatically
   }
   std::string sim_cmd = "rm BOOST_SERIALIZATION_NVP";
   if (system(sim_cmd.c_str()) != 0)
   {
      std::cerr << "Error removing nvp " << std::endl
                << sim_cmd << std::endl;
      exit(-1);
   }
#endif
}

/****************************************/
/****************************************/
void CForagingLoopFunctions::food_scarcity()
{
   ForagingThymioNN *cController = dynamic_cast<ForagingThymioNN *>(get_controller(0)); // index 0 because any index will do
   if (cController->FBehavior == BaseController::FaultBehavior::FAULT_FOOD_SCARCITY)
   {
      float rad = 0.040 * (float)(cController->foodID + 1);
      m_fFoodSquareRadius = {rad * rad};                        // only one small food item
      float food_x = m_pcRNG->Uniform(CRange<Real>(1.7, 1.85)); // very far from nest_x but not against the border
      float food_y = m_pcRNG->Uniform(CRange<Real>(0.3, 1.8));  // y does not matter so much
      m_cFoodPos = {CVector2(food_x, food_y)};
   }
}
void CForagingLoopFunctions::Reset()
{
   food_scarcity();
   //BaseEvolutionLoopFunctions::Reset();
   reset_agent_positions(forcePositions); //force the positions (unlike BaseEvol/Base LoopFunctions::Reset)
   reset_cylinder_positions();

   m_cVisitedFood.clear();
   for (size_t i = 0; i < m_cFoodPos.size(); i++)
   {
      m_cVisitedFood.push_back(0);
   }

   for (size_t i = 0; i < m_unNumberRobots; ++i)
   {
      /* Get handle to foot-bot entity and controller */
      argos::CThymioEntity *cThym = m_pcvecRobot[i];
      ForagingThymioNN &cController = dynamic_cast<ForagingThymioNN &>(cThym->GetControllableEntity().GetController());
      //cController.Reset(); will happen automatically
   }

   if (virtual_energy != NULL)
   {
      virtual_energy->reset();
   }
}

#if HETEROGENEOUS & !PRINT_NETWORK

void CForagingLoopFunctions::select_new_controller(ForagingThymioNN &cController)
{

   bool all_trials_finished = cController.worker.reset();
   if (cController.worker.initial_phase && all_trials_finished)
   {
      // finish descriptor
      current_robot = cController.worker.index;
      std::vector<float> ident = alltrials_descriptor();
      cController.worker.F = Eigen::VectorXd(ident.size());
      for (size_t i = 0; i < ident.size(); ++i)
      {
         cController.worker.F[i] = ident[i];
      }
      cController.worker.initial_phase = false;
   }

   if (!cController.worker.initial_phase) //update trial info
   {
      Eigen::VectorXd x = cController.worker.get_sample();
      Eigen::VectorXd f = Eigen::VectorXd::Constant(1, cController.worker.fitness(m_unNumberRobots));
      size_t worker_index = cController.worker.index;
      argos::LOG << "worker " << worker_index << std::endl;
      argos::LOG << "all trials finished " << all_trials_finished << std::endl;
      argos::LOG << "initial phase " << cController.worker.initial_phase << std::endl;
      
      argos::LOG.Flush();
      x = opt.optimize_step<ControllerEval>(x, f, worker_index, state_fun, all_trials_finished);
      if (all_trials_finished) // select new sample
      {
         cController.worker.new_sample = x.head(BEHAV_DIM);
         argos::LOG << "new sample" << x << std::endl;
         std::vector<double> bd(cController.worker.new_sample.data(),
                                cController.worker.new_sample.data() + cController.worker.new_sample.rows() * cController.worker.new_sample.cols());
         cController.select_net(bd, num_subtrials, ticks_per_subtrial);
         std::string sim_cmd = "rm BOOST_SERIALIZATION_NVP";
         if (system(sim_cmd.c_str()) != 0)
         {
            std::cerr << "Error removing nvp " << std::endl
                      << sim_cmd << std::endl;
            exit(-1);
         }
      }
   }
   // reset the controller (food_items_collected,)
   cController.Reset();
}
#endif

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

bool CForagingLoopFunctions::try_robot_position(CVector3 &Position, CQuaternion &Orientation, const CRange<Real> x_range, const CRange<Real> y_range, const size_t m_unRobot, size_t &num_tries)
{

   CEmbodiedEntity *entity = get_embodied_entity(m_unRobot);
   ForagingThymioNN *cController = dynamic_cast<ForagingThymioNN *>(get_controller(m_unRobot));
   if (cController->b_damagedrobot && cController->FBehavior == BaseController::FaultBehavior::FAULT_SOFTWARE)
   {
      forcePositions = true;
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
         if (num_tries > 1000)
         {
            std::cout << "could not initialise controller at food item " << std::endl;
            return false;
         }
      } while (!entity->MoveTo(
          Position,    // to this position
          Orientation, // with this orientation
          false        // this is not a check, leave the robot there
          ));
      return true;
   }
   else if (cController->b_damagedrobot && cController->FBehavior == BaseController::FaultBehavior::FAULT_SOFTWARE_FOOD)
   {
      forcePositions = true;
      bool success = true;
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
         if (num_tries > 1000)
         {
            std::cout << "could not initialise controller at food item " << std::endl;
            return false;
         }
      } while (!entity->MoveTo(
          Position,    // to this position
          Orientation, // with this orientation
          false        // this is not a check, leave the robot there
          ));

      std::cout << "successfully initialised at food item " << std::endl;
      return true;
   }
   else if (cController->b_damagedrobot && cController->FBehavior == BaseController::FaultBehavior::FAULT_SOFTWARE_NEIGHBOURHOOD)
   {
      forcePositions = true;
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
         Position = CVector3(m_cFoodPos[i].GetX() + added_x, m_cFoodPos[i].GetY() + added_y, 0.0);
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
         if (num_tries > 1000)
         {
            std::cout << "could not initialise controller around food item " << std::endl;
            return false;
         }
      } while (!entity->MoveTo(
          Position,    // to this position
          Orientation, // with this orientation
          false        // this is not a check, leave the robot there
          ));
      std::cout << "successfully initialised around food item " << std::endl;
      return true;
   }
   else
   {
      return BaseLoopFunctions::try_robot_position(Position, Orientation, x_range, y_range, m_unRobot, num_tries);
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
   for (size_t j = 0; j < m_unNumberRobots; ++j)
   {

      /* Get handle to foot-bot entity and controller */
      argos::CThymioEntity *cThym = m_pcvecRobot[j];
      ForagingThymioNN &cController = dynamic_cast<ForagingThymioNN &>(cThym->GetControllableEntity().GetController());
      /* Get the position of the foot-bot on the ground as a CVector2 */
      CVector2 cPos;
      cPos.Set(cThym->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
               cThym->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      VirtualState virtualState = VirtualState::DEFAULT;
      /* The thymio has a food item and does not drop it due to software fault*/
      if (cController.holdingFood)
      {
         /* Check whether the foot-bot is in the nest */
         if (cPos.GetX() < nest_x)
         {
            /* Drop the food item */
            cController.holdingFood = false;

            virtualState = VirtualState::NEST;

            /* Increase the food count */
            fitfun->fitness_per_trial[m_unCurrentTrial]++;
#ifdef HETEROGENEOUS
            ++cController.worker.numFoodCollected;
#endif
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
               if ((cPos - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius[i]) // on food source
               {
                  if (m_cVisitedFood[i] == 0) // no remaining harvesting time
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
                  else
                  {
                     if (stats != NULL)
                     {
                        stats->count_notharvesting(); // not harvesting but on food source = lost time
                     }
                  }
               }
            }
         }
         else
         {
            if (stats != NULL) //
            {
               stats->count_nestvisitwithoutholdingfood(); // not holding food but still visited nest = waste of time
            }
         }
      }
      // adjust state where needed
      if (cController.holdingFood)
      {
         virtualState = HOLDING_FOOD; //
      }
      else
      {
         if (stats != NULL)
         {
            stats->count_stepswithoutholdingfood();
         }
      }
      if (virtual_energy != NULL)
      {

         virtual_energy->step(j, cThym->GetEmbodiedEntity().IsCollidingWithSomething(), virtualState);
      }

#ifdef HETEROGENEOUS
      // subtract tick; check if trial has finished; if so, get a new sample from BO and initialise new network
      --cController.num_ticks_left;
      if (cController.num_ticks_left == 0)
      {
         cController.worker.finish_trial();
         select_new_controller(cController);
         --cController.num_trials_left;
         cController.num_ticks_left = ticks_per_subtrial;
         if (cController.num_trials_left == 0)
         {
            
            cController.num_trials_left = num_subtrials;
         }
      }

#endif
   }
   for (size_t f = 0; f < m_cVisitedFood.size(); ++f)
   {
      m_cVisitedFood[f] = std::max(0, m_cVisitedFood[f] - 1);
#ifdef PRINTING

      std::cout << "Harvesting time for food  " << f << " on location " << m_cFoodPos[f] << "\n is now " << m_cVisitedFood[f] << std::endl;
#endif
   }

   BaseEvolutionLoopFunctions::PostStep();

   if (virtual_energy != NULL)
   {
      if (virtual_energy->depleted())
      {
#ifdef PRINTING
         std::cout << "out of energy" << std::endl;
#endif

         virtual_energy_finish_trial();
         argos::CSimulator::GetInstance().Terminate();
         stop_eval = true;
      }
   }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CForagingLoopFunctions, "foraging_loopfunctions" + std::string(TAG))
