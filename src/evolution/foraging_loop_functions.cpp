#include <src/core/fitness_functions.h>
#include "src/evolution/foraging_loop_functions.h"
#include "src/evolution/foraging_nn_controller.h"
#include <argos3/plugins/robots/thymio/simulator/thymio_entity.h>

#if HETEROGENEOUS
std::vector<Eigen::VectorXd> Params::busy_samples;
double Params::L;
double Params::M;
size_t Params::count;
std::vector<Params::archiveparams::archive_t> Params::archiveparams::multimap;
std::map<std::vector<double>,bool, Params::archiveparams::classcomp> Params::archiveparams::checked_constraints;// for each constraint, whether or not it was checked

size_t Params::map_index;
#endif

/****************************************/
/****************************************/

CForagingLoopFunctions::CForagingLoopFunctions() : m_pcFloor(NULL), virtual_energy(NULL), stats(NULL)
{
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Init(TConfigurationNode &t_node)
{
   m_pcFloor = &GetSpace().GetFloorEntity();
   BaseEvolutionLoopFunctions::Init(t_node);

   m_numFoodCollected.clear();
   for (size_t i = 0; i < m_unNumberRobots; ++i)
   {
      /* Get handle to foot-bot entity and controller */
      //argos::CThymioEntity *cThym = m_pcvecRobot[i];
      //ForagingThymioNN &cController = dynamic_cast<ForagingThymioNN &>(cThym->GetControllableEntity().GetController());
      //cController.Reset(); will happen automatically
      m_numFoodCollected.push_back(0);
   }

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

#if HETEROGENEOUS
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
      THROW_ARGOSEXCEPTION_NESTED("Error initializing network binary", ex);
   }
   std::vector<double> normal_ID;
#if RECORD_FIT
   normal_ID = {};
#else
   try
   {
      GetNodeAttribute(t_node, "optimisation", optimisation);
   }
   catch (CARGoSException &ex)
   {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing stopping criterion", ex);
   }
   if (optimisation == "BO")
   {
      normal_ID = {0.5, 0.5, 0.5, 0.5};
   }
   else if (optimisation == "BO_noID")
   {
      normal_ID = {};
   }
   else
   {
      normal_ID = {};
   }
   load_ID_map = false;
   try
   {
      GetNodeAttribute(t_node, "load_ID_map", load_ID_map);
   }
   catch (CARGoSException &ex)
   {
      THROW_ARGOSEXCEPTION_NESTED("Error loading ID map", ex);
   }

#endif

   Params::archiveparams::archive = load_archive(std::string(global::archive_path) + "/archive_" + std::to_string(global::gen_to_load) + ".dat", normal_ID);

#if RECORD_FIT

   for (size_t i = 0; i < m_unNumberRobots; ++i)
   {
      // get the best bd
      std::string stats_filename = output_folder + "/async_stats_best" + std::to_string(i) + ".dat";
      std::vector<double> bd = get_best_bd(stats_filename);
      argos::CThymioEntity *cThym = m_pcvecRobot[i];
      ForagingThymioNN &cController = dynamic_cast<ForagingThymioNN &>(cThym->GetControllableEntity().GetController());
      cController.select_net(bd);
   }
#else
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

   try
   {
      GetNodeAttribute(t_node, "stop", stop_crit);
   }
   catch (CARGoSException &ex)
   {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing stopping criterion", ex);
   }
   try
   {
      GetNodeAttribute(t_node, "reset", reset);
   }
   catch (CARGoSException &ex)
   {
      THROW_ARGOSEXCEPTION_NESTED("Error resetting ", ex);
   }
   if (optimisation == "BO" || optimisation == "BO_noID")
   {
      init_BO(normal_ID);
   }
   else if (optimisation == "BO_joint")
   {
      init_multiBO(true, normal_ID); // same for initialisation except opt is different type
   }
   else if (optimisation == "BO_multi")
   {
      init_multiBO(false, normal_ID);
   }
   else
   {
      init_randomsearch();
   }
   m_pcRNG = CRandom::CreateRNG("argos"); // reset the state of the RNG to position robots the same as usual runs
   place_robots(num_subtrials);
#endif
#endif
}

#if HETEROGENEOUS & !RECORD_FIT
void CForagingLoopFunctions::init_BO(std::vector<double> normal_ID)
{
   Params::count = 0;
   // select new controller now
   opt.push_back(new Opt_t());
   opt[0]->optimize_init<ControllerEval>(normal_ID.size(), m_unNumberRobots, state_fun, VARIABLE_NOISE);
   Eigen::VectorXd result;
   if (load_ID_map)
   {
      std::cout << "LOADING IDMAP ID_archive_fixed_" + std::to_string(global::gen_to_load) + ".dat" << std::endl;
      auto pair = load_ID_archive(std::string(global::archive_path) + "/ID_archive_fixed_" + std::to_string(global::gen_to_load) + ".dat", 4);
      Params::archiveparams::elem_archive max_el = std::get<0>(pair);
      Params::archiveparams::archive = std::get<1>(pair);
      result = Eigen::VectorXd(global::behav_dim + global::num_ID_features);
      for (size_t i = 0; i < result.size(); ++i)
      {
         result[i] = max_el.behav_descriptor[i];
      }
   }
   else
   {
      Eigen::VectorXd id_vec = Eigen::VectorXd(normal_ID.size());
      for (size_t i = 0; i < normal_ID.size(); ++i)
         id_vec(i) = normal_ID[i];
      result = opt[0]->select_sample<ControllerEval>(id_vec);
      
   }
   Params::archiveparams::checked_constraints = {};
   /* initial phase: select controller for one robot and then put others with the same as well */
   argos::CThymioEntity *cThym = m_pcvecRobot[0];
   ForagingThymioNN &cController = dynamic_cast<ForagingThymioNN &>(cThym->GetControllableEntity().GetController());
   cController.worker = ForagingThymioNN::Worker(num_subtrials, 0);

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
      cController.Reset();
   }
   std::string sim_cmd = "rm " + cController.savefile;
   if (system(sim_cmd.c_str()) != 0)
   {
      std::cerr << "Error removing nvp " << std::endl
                << sim_cmd << std::endl;
      exit(-1);
   }

   if (!load_ID_map)
   {
      Params::archiveparams::old_archive = Params::archiveparams::archive; // this old archive will now just be auxiliary
      Params::archiveparams::archive = {};
   }
}

void CForagingLoopFunctions::init_multiBO(bool single_worker, std::vector<double> normal_ID)
{
   Params::archiveparams::old_archive = Params::archiveparams::archive; // this old archive will now just be auxiliary
   Params::count = 0;
   std::vector<BaseController::FaultBehavior> found_faults;
   std::map<BaseController::FaultBehavior, int> index_vec;
   std::map<BaseController::FaultBehavior, Eigen::VectorXd> sample_vec;
   std::map<BaseController::FaultBehavior, int> count;
   if (single_worker)
   {
      opt.push_back(new Opt_t());
   }
   for (size_t i = 0; i < m_unNumberRobots; ++i)
   {
      argos::CThymioEntity *cThym = m_pcvecRobot[i];
      ForagingThymioNN &cController = dynamic_cast<ForagingThymioNN &>(cThym->GetControllableEntity().GetController());
      auto found = std::find(found_faults.begin(), found_faults.end(), cController.FBehavior);
      if (found == found_faults.end())
      {
         found_faults.push_back(cController.FBehavior);
         if (!single_worker)
            opt.push_back(new Opt_t());
         index_vec[cController.FBehavior] = 0;
         count[cController.FBehavior] = 0;
      }
      else
      {
         int index = index_vec.at(cController.FBehavior);
         ++index;
         cController.worker.index = index;
         index_vec[cController.FBehavior] = index;
      }
   }
   Eigen::VectorXd result;
   for (size_t i = 0; i < opt.size(); ++i)
   {
      if (single_worker)
      {
         //size_t num_ID_features, size_t behav_dim
         opt[i]->optimize_init_joint<ControllerEval>(found_faults.size(), normal_ID.size(), state_fun, VARIABLE_NOISE);
         fill_combinedmap_with_identifier(found_faults.size(), {}, 100); // fill map with combinations of the best 100 solutions
      }
      else
      {
         opt[i]->optimize_init<ControllerEval>(normal_ID.size(), index_vec[found_faults[i]] + 1, state_fun, VARIABLE_NOISE);
         fill_multimap_with_identifier({});
      }
   }
   if (single_worker)
   {
      Eigen::VectorXd new_x = opt[0]->select_sample<ControllerEval>({});
      current_sample = new_x;
      std::vector<double> bd;
      for (size_t i = 0; i < current_sample.size(); ++i)
      {
         bd.push_back(current_sample[i]);
      }
      Params::archiveparams::elem_archive el = Params::archiveparams::archive.at(bd);
      for (size_t i = 0; i < m_unNumberRobots; ++i)
      {
         argos::CThymioEntity *cThym = m_pcvecRobot[i];
         ForagingThymioNN &cController = dynamic_cast<ForagingThymioNN &>(cThym->GetControllableEntity().GetController());
         auto found = std::find(found_faults.begin(), found_faults.end(), cController.FBehavior);
         size_t opt_idx = found - found_faults.begin(); //use as index for the joint bd
         size_t worker_idx = 0;
         cController.worker = ForagingThymioNN::Worker(num_subtrials, worker_idx, opt_idx);
         count[cController.FBehavior] = worker_idx;
         cController.worker.opt_index = opt_idx;
         cController.num_ticks_left = ticks_per_subtrial;
         cController.num_trials_left = num_subtrials;
         cController.select_net(el.joint_controller[opt_idx]);
         std::string sim_cmd = "rm " + cController.savefile;
         if (system(sim_cmd.c_str()) != 0)
         {
            std::cerr << "Error removing nvp " << std::endl
                      << sim_cmd << std::endl;
            exit(-1);
         }
         // reset the controller (food_items_collected,)
         cController.Reset();
      }
   }
   else
   {
      for (size_t i = 0; i < m_unNumberRobots; ++i)
      {
         argos::CThymioEntity *cThym = m_pcvecRobot[i];
         ForagingThymioNN &cController = dynamic_cast<ForagingThymioNN &>(cThym->GetControllableEntity().GetController());
         auto found = std::find(found_faults.begin(), found_faults.end(), cController.FBehavior);
         size_t opt_idx = found - found_faults.begin();
         size_t worker_idx = count.at(cController.FBehavior);
         cController.worker = ForagingThymioNN::Worker(num_subtrials, worker_idx, opt_idx);
         worker_idx++;
         count[cController.FBehavior] = worker_idx;
         cController.worker.opt_index = opt_idx;
         Eigen::VectorXd result = opt[opt_idx]->select_sample<ControllerEval>({});
         cController.worker.new_sample = result.head(BEHAV_DIM);
         std::vector<double> bd(result.data(), result.data() + result.rows() * result.cols());
         cController.select_net(bd, num_subtrials, ticks_per_subtrial);
         std::string sim_cmd = "rm " + cController.savefile;
         if (system(sim_cmd.c_str()) != 0)
         {
            std::cerr << "Error removing nvp " << std::endl
                      << sim_cmd << std::endl;
            exit(-1);
         }
         // reset the controller (food_items_collected,)
         cController.Reset();
      }
   }
}

void CForagingLoopFunctions::init_randomsearch()
{
   opt[0]->optimize_init<ControllerEval>(0, m_unNumberRobots, state_fun); //just to get some useful stats
   for (size_t i = 0; i < m_unNumberRobots; ++i)
   {
      Params::busy_samples.push_back(opt[0]->NULL_VEC);
      argos::CThymioEntity *cThym = m_pcvecRobot[i];
      ForagingThymioNN &cController = dynamic_cast<ForagingThymioNN &>(cThym->GetControllableEntity().GetController());

      proposals.push_back(new Proposal());
      std::string res_dir = opt[0]->res_dir();
      std::cout << "res dir " << res_dir << std::endl;
      proposals[i]->init(res_dir, i);
      std::vector<double> bd = proposals[i]->generate();
      cController.worker.new_sample = Eigen::VectorXd(bd.size());
      for (size_t j = 0; j < bd.size(); ++j)
      {
         cController.worker.new_sample[j] = bd[j];
      }
      cController.worker.index = i;
      proposals[i]->update();
      cController.select_net(bd, num_subtrials, ticks_per_subtrial);
      // reset the controller (food_items_collected,)
      cController.Reset();
      std::string sim_cmd = "rm " + cController.savefile;
      if (system(sim_cmd.c_str()) != 0)
      {
         std::cerr << "Error removing nvp " << std::endl
                   << sim_cmd << std::endl;
         exit(-1);
      }
   }
}

#endif
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

   if (virtual_energy != NULL)
   {
      virtual_energy->reset();
   }
}

#if HETEROGENEOUS & !RECORD_FIT

void CForagingLoopFunctions::check_ID_map(std::vector<float> ident)
{
   std::map<std::vector<double>, unsigned> bds = Params::archiveparams::get_unique_behaviours();
   // check first element in map; if not then we need to fill the map with the ID vec
   std::vector<double> bd = bds.begin()->first;
   for (size_t i = 0; i < ident.size(); ++i)
   {
      bd.push_back(ident[i]);
   }
   if (Params::archiveparams::archive.find(bd) == Params::archiveparams::archive.end())
   {
      // start filling up
      for (auto it = bds.begin(); it != bds.end(); ++it)
      {
         std::vector<double> b = it->first;
         for (size_t i = 0; i < ident.size(); ++i)
         {
            b.push_back(ident[i]);
         }
         Params::archiveparams::elem_archive el;
         el.behav_descriptor = b;
         el.controller = it->second;
         auto pair = Params::get_closest_neighbour_fit(b);
         el.fit = std::get<0>(pair);
         el.fit_var = std::get<1>(pair);
         Params::archiveparams::archive[b] = el;
      }
   }
}

void CForagingLoopFunctions::select_new_controller(ForagingThymioNN &cController, size_t stat_index, bool all_trials_finished, bool multi)
{

   if (cController.worker.initial_phase && all_trials_finished)
   {
      // finish descriptor
      current_robot = cController.worker.index;
      std::vector<float> ident = alltrials_descriptor();
      Params::archiveparams::classcomp::discretise(ident, 16.0);
      cController.worker.F = Eigen::VectorXd(ident.size());
      std::vector<double> checked_vec = cController.worker.new_sample;
      for (size_t i = 0; i < ident.size(); ++i)
      {
         cController.worker.F[i] = ident[i];
         checked_vec.push_back(ident[i]);
      }
      Params::archiveparams::checked_constraints[checked_vec] = true;
      cController.worker.initial_phase = false;
      // fill the map with corresponding identification vector
      if (!multi && !load_ID_map)
      {
         fill_map_with_identifier(ident);
      }
      else
      {
         if (load_ID_map)
         {
            check_ID_map(ident);
         }
      }
      


   }
   size_t index = cController.worker.opt_index;
   opt[index]->push_fitness(cController.worker.index, cController.worker.fitness(m_unNumberRobots));
   opt[index]->push_time((double)cController.worker.total_time, all_trials_finished);
   if (!cController.worker.initial_phase) //update trial info
   {
      Eigen::VectorXd old_x = cController.worker.get_sample();

      size_t worker_index = cController.worker.index;
      argos::LOG << "worker " << worker_index << std::endl;
      argos::LOG << "all trials finished " << all_trials_finished << std::endl;
      argos::LOG << "initial phase " << cController.worker.initial_phase << std::endl;

      argos::LOG.Flush();
      Eigen::VectorXd new_x = opt[index]->optimize_step<ControllerEval>(cController.worker.get_sample(), worker_index, stat_index, state_fun, all_trials_finished);

      if (all_trials_finished) // select new sample
      {
         std::ofstream busy_log("busy_samples.txt", std::ios::app);
         busy_log << "worker_index " << cController.worker.index << std::endl;
         std::cout << "old_x " << old_x.transpose() << std::endl;
         std::cout << "new_x " << new_x.transpose() << std::endl;
         Params::remove_from_busysamples(old_x);
         Params::add_to_busysamples(new_x);
         cController.worker.new_sample = new_x.head(BEHAV_DIM);
         argos::LOG << "new sample" << new_x << std::endl;
         std::vector<double> bd(new_x.data(), new_x.data() + new_x.rows() * new_x.cols());
         cController.select_net(bd);
         std::string sim_cmd = "rm " + cController.savefile;
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

void CForagingLoopFunctions::select_new_controller_random(ForagingThymioNN &cController, bool all_trials_finished)
{
   opt[0]->push_fitness(cController.worker.index, cController.worker.fitness(m_unNumberRobots));
   if (all_trials_finished) // select new sample
   {

      size_t i = cController.worker.index;

      proposals[i]->print_stats(i, (double)cController.worker.total_time, cController.worker.new_sample, opt[0]->get_mean_fitness(i));
      std::vector<double> bd = proposals[i]->generate();
      cController.worker.new_sample = Eigen::VectorXd(bd.size());
      for (size_t j = 0; j < bd.size(); ++j)
      {
         cController.worker.new_sample[j] = bd[j];
      }
      proposals[i]->update();
      cController.select_net(bd);
      std::string sim_cmd = "rm " + cController.savefile;
      if (system(sim_cmd.c_str()) != 0)
      {
         std::cerr << "Error removing nvp " << std::endl
                   << sim_cmd << std::endl;
         exit(-1);
      }
   }
   // reset the controller (food_items_collected,)
   cController.Reset();
}

void CForagingLoopFunctions::select_joint_controller(bool alltrialsfinished)
{
   // push fitness

   double sum_fit = 0.0;
   std::vector<size_t> seen_index;
   double time = 0.0;
   for (size_t i = 0; i < m_unNumberRobots; ++i)
   {
      argos::CThymioEntity *cThym = m_pcvecRobot[i];
      ForagingThymioNN &cController = dynamic_cast<ForagingThymioNN &>(cThym->GetControllableEntity().GetController());
      size_t index = cController.worker.opt_index;
      sum_fit += cController.worker.fitness(m_unNumberRobots);
      // reset the controller (food_items_collected,)
      cController.Reset();
      time += (double)cController.worker.total_time;
   }

   opt[0]->push_fitness(0, sum_fit / m_unNumberRobots);
   opt[0]->push_time(time / m_unNumberRobots, alltrialsfinished);
   //do step for each opt; then set all matching controllers

   Eigen::VectorXd new_x = opt[0]->optimize_step<ControllerEval>(current_sample, 0, 0, state_fun, alltrialsfinished);

   if (alltrialsfinished)
   {

      argos::LOG << "new sample" << new_x << std::endl;
      std::vector<double> bd;
      for (size_t i = 0; i < new_x.size(); ++i)
      {
         bd.push_back(new_x[i]);
      }
      Params::archiveparams::elem_archive el = Params::archiveparams::archive.at(bd);
      for (size_t j = 0; j < m_unNumberRobots; ++j)
      {
         argos::CThymioEntity *cThym = m_pcvecRobot[j];
         ForagingThymioNN &cController = dynamic_cast<ForagingThymioNN &>(cThym->GetControllableEntity().GetController());
         size_t index = cController.worker.opt_index;

         cController.select_net(el.joint_controller[index]);
         std::string sim_cmd = "rm " + cController.savefile;
         if (system(sim_cmd.c_str()) != 0)
         {
            std::cerr << "Error removing nvp " << std::endl
                      << sim_cmd << std::endl;
            exit(-1);
         }
      }
      current_sample = new_x;
   }
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
#if HETEROGENEOUS & !RECORD_FIT
void CForagingLoopFunctions::reset_controller(size_t j, bool reset, bool alltrialsfinished)
{
   argos::CThymioEntity *cThym = m_pcvecRobot[j];
   ForagingThymioNN &cController = dynamic_cast<ForagingThymioNN &>(cThym->GetControllableEntity().GetController());
   cController.num_ticks_left = ticks_per_subtrial;
   if (reset)
   {
      CPhysicsModel *model;
      //model = &entity->GetPhysicsModel("dyn2d_0");
      //model->UpdateEntityStatus();
      if (!cThym->GetEmbodiedEntity().MoveTo(
              m_vecInitSetup[cController.trial][j].Position,    // to this position
              m_vecInitSetup[cController.trial][j].Orientation, // with this orientation
              false                                             // this is not a check, leave the robot there
              ))
      {
      }
   }
   if (alltrialsfinished)
   {
      cController.num_trials_left = num_subtrials;
      cController.reset_stopvals();
   }
}
#endif
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

#if HETEROGENEOUS & !RECORD_FIT
      float collided = 0.0f;
      if (cThym->GetEmbodiedEntity().IsCollidingWithSomething())
      {
         collided = 1.0f;
      }
      cController.collision_value = 0.99 * cController.collision_value + 0.01 * collided;
#endif
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
#if HETEROGENEOUS & !RECORD_FIT
            ++cController.worker.numFoodCollected;
#endif
            ++m_numFoodCollected[j];
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

#if HETEROGENEOUS & !RECORD_FIT
      // subtract tick; check if trial has finished; if so, get a new sample from BO and initialise new network
      --cController.num_ticks_left;
      ++cController.worker.total_time;
      bool stop = stop_criterion(cController);
      if (stop || cController.num_ticks_left == 0)
      {
         --cController.num_trials_left;
         ++cController.trial;
         if (stop)
         {
            cController.worker.numFoodCollected = 0; // new fitness estimate will be 0
            size_t index = cController.worker.opt_index;
            opt[index]->clear_fitness(cController.worker.index); //clear previous estimates --> after one push of 0 will stop with mean=0 and sd=0
         }
         bool alltrialsfinished = stop || cController.num_trials_left == 0;
         if (optimisation == "BO" || optimisation == "BO_noID")
         {
            select_new_controller(cController, j, alltrialsfinished, false);
         }
         else if (optimisation == "BO_multi")
         {
            select_new_controller(cController, j, alltrialsfinished, true);
         }
         else if (optimisation == "random")
         {
            select_new_controller_random(cController, alltrialsfinished);
         }
         else if (optimisation == "BO_joint")
         {
            if (j == m_unNumberRobots - 1)
            {
               select_joint_controller(alltrialsfinished);
               for (size_t i = 0; i < m_unNumberRobots; ++i)
               {
                  reset_controller(i, reset, alltrialsfinished);
               }
            }
            continue; // ignore the reset, not needed
         }
         else
         {
            throw std::runtime_error("which optimisation?" + optimisation);
         }
         reset_controller(j, reset, alltrialsfinished);
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
