

#include <src/core/base_loop_functions.h>
#include <src/core/fitness_functions.h>
#include <src/core/statistics.h>

FloreanoMondada::FloreanoMondada() : FitFun()
{
    num_updates = argos::CSimulator::GetInstance().GetMaxSimulationClock();
}

void FloreanoMondada::after_robotloop(BaseLoopFunctions &cLoopFunctions)
{
    // compute linear speed for fitness function
    float s = cLoopFunctions.get_controller(0)->linear_speed_01(); // in [0,1]
    float ds = cLoopFunctions.get_controller(0)->turn_speed_01();  // in [0,1]
    speed += s;
    float curr_lin_speed = s * (1.0 - sqrt(ds));
    lin_speed += curr_lin_speed;
    num_ds += (ds >= 0.1) ? 1.0 : 0.0;
    float maxIRSensor = -1.0;
    for (size_t i = 0; i < cLoopFunctions.inputs.size() - 1; ++i)
    {
        maxIRSensor = Max(maxIRSensor, cLoopFunctions.inputs[i]);
    }
    nb_coll += (1.0f - maxIRSensor);
}
/*after completing a trial,calc fitness*/
void FloreanoMondada::apply(BaseLoopFunctions &cLoopFunctions)
{
    float fit = lin_speed / num_updates * (Real)nb_coll / num_updates;
    fitness_per_trial.push_back(fit);
};

/*after completing all trials, combine fitness*/
float FloreanoMondada::after_trials()
{
    float minfit = StatFuns::min(fitness_per_trial);
    fitness_per_trial.clear();
    return minfit;
};
/*after completing a trial, print some statistics (if desired)*/
void FloreanoMondada::print_progress(size_t trial)
{

    printf("\n\n fitness in trial %zu is %f", trial, fitness_per_trial[trial]);
    printf("\n\n lin_speed = %f", lin_speed);
    printf("\n\n nb_coll = %f", nb_coll);
}
MeanSpeed::MeanSpeed() : FitFun()
{
    num_updates = argos::CSimulator::GetInstance().GetMaxSimulationClock();
}

void MeanSpeed::after_robotloop(BaseLoopFunctions &cLoopFunctions)
{
    // compute linear speed for fitness function
    float s = cLoopFunctions.get_controller(0)->linear_speed_01(); // in [0,1]
    float ds = cLoopFunctions.get_controller(0)->turn_speed_01();  // in [0,1]
    speed += s;
    float curr_lin_speed = s * (1.0 - sqrt(ds));
    lin_speed += curr_lin_speed;
    num_ds += (ds >= 0.1) ? 1.0 : 0.0;
    float maxIRSensor = -1.0;
    for (size_t i = 0; i < cLoopFunctions.inputs.size() - 1; ++i)
    {
        maxIRSensor = Max(maxIRSensor, cLoopFunctions.inputs[i]);
    }
    nb_coll += (1.0f - maxIRSensor);
}
/*after completing a trial,calc fitness*/
void MeanSpeed::apply(BaseLoopFunctions &cLoopFunctions)
{
    float fit = lin_speed / num_updates * (Real)nb_coll / num_updates;
    fitness_per_trial.push_back(fit);

    // reset
    nb_coll = 0;
    speed = 0.0f;
    lin_speed = 0.0f;
    num_ds = 0.0;
};

/*after completing all trials, combine fitness*/
float MeanSpeed::after_trials()
{
    float meanfit = StatFuns::mean(fitness_per_trial);
    fitness_per_trial.clear();
    return meanfit;
};

/*after completing a trial, print some statistics (if desired)*/
void MeanSpeed::print_progress(size_t trial)
{

    printf("\n\n fitness in trial %zu is %f", trial, fitness_per_trial[trial]);
    printf("\n\n lin_speed = %f", lin_speed);
    printf("\n\n nb_coll = %f", nb_coll);
}
Coverage::Coverage(std::string init_string, BaseLoopFunctions *cLoopFunctions)
{
    if (init_string == "Coverage")
    {
        coverageCalc = new CoverageCalc(cLoopFunctions);
    }
    else if (init_string == "BorderCoverage")
    {
        coverageCalc = new BorderCoverageCalc(cLoopFunctions);
    }
    else
    {
        throw std::runtime_error("init string should be either Coverage or BorderCoverage");
    }
    num_updates = argos::CSimulator::GetInstance().GetMaxSimulationClock();
}

/*after completing trial, calc fitness*/
void Coverage::apply(BaseLoopFunctions &cLoopFunctions)
{
    //coverage
    fitness_per_trial.push_back(coverageCalc->get_coverage());
    coverageCalc->after_trial();
}

/*after a single step of single agent */
void Coverage::after_robotloop(BaseLoopFunctions &cLoopFunctions)
{
    for (size_t robot_index = 0; robot_index < cLoopFunctions.m_unNumberRobots; ++robot_index)
    {
        coverageCalc->update(cLoopFunctions.curr_pos[robot_index]);
    }
}
/*after completing all trials, combine fitness*/
float Coverage::after_trials()
{
    float meanfit = StatFuns::mean(fitness_per_trial);
    fitness_per_trial.clear();

    return meanfit;
}

/*after completing a trial, print some statistics (if desired)*/
void Coverage::print_progress(size_t trial)
{

    printf("\n\n fitness (coverage) in trial %zu is %f", trial, fitness_per_trial[trial]);
}

TrialCoverage::TrialCoverage(std::string init_type, BaseLoopFunctions *cLoopFunctions) : Coverage(init_type, cLoopFunctions)
{
}

/*before trial calc obstacle cells*/
void TrialCoverage::before_trial(BaseLoopFunctions &cLoopFunctions)
{
    coverageCalc->get_num_cells(cLoopFunctions);
}

DecayCoverage::DecayCoverage(std::string init_string, BaseLoopFunctions *cLoopFunctions)
{
    coverageCalc = new DecayCoverageCalc(init_string, cLoopFunctions);
    num_updates = argos::CSimulator::GetInstance().GetMaxSimulationClock(); // one update for the entire swarm
}

/*after completing trial, calc fitness*/
void DecayCoverage::apply(BaseLoopFunctions &cLoopFunctions)
{
    //coverage
    float sum = coverageCalc->accumulator;
    float coverage = sum / ((float)coverageCalc->grid.size() * (float)num_updates);
    if (!StatFuns::in_range(coverage, 0.0f, 1.0f))
    {
        throw std::runtime_error("fitness not in [0,1]");
    }
    fitness_per_trial.push_back(coverage);
    coverageCalc->end_trial();
}

/*after a single step of single agent */
void DecayCoverage::after_robotloop(BaseLoopFunctions &cLoopFunctions)
{
    for (size_t robot_index = 0; robot_index < cLoopFunctions.m_unNumberRobots; ++robot_index)
    {
        coverageCalc->update(cLoopFunctions.curr_pos[robot_index]);
    }
    coverageCalc->get_grid_sum(); // one update for the entire swarm
    coverageCalc->decay();
}
/*after completing all trials, combine fitness*/
float DecayCoverage::after_trials()
{
    float meanfit = StatFuns::mean(fitness_per_trial);
    fitness_per_trial.clear();

    return meanfit;
}

Aggregation::Aggregation(BaseLoopFunctions *cLoopFunctions) : FitFun()
{
    argos::CVector3 max = cLoopFunctions->get_arenasize();
    maxdist = StatFuns::get_minkowski_distance(CVector3(max.GetX(), max.GetY(), 0.0f), argos::CVector3::ZERO);
    num_updates = argos::CSimulator::GetInstance().GetMaxSimulationClock(); // one update for the entire swarm // one update for the entire swarm
}
void Aggregation::after_robotloop(BaseLoopFunctions &cLoopFunctions)
{
    argos::CVector3 cm = cLoopFunctions.centre_of_mass(cLoopFunctions.curr_pos);
    //trial_dist += StatFuns::get_avg_dist(data.first, data.second); 
    trial_dist += (1.0 - StatFuns::get_avg_dist(cLoopFunctions.curr_pos, cm)/maxdist);// doing normalisation now because exit on collisions
}
/*after completing a trial,calc fitness*/
void Aggregation::apply(BaseLoopFunctions &cLoopFunctions)
{
    //The fitness function is inversely
    //proportional to the average distance to the centre of mass over the entire simulation
    float fitness = trial_dist / ((float)num_updates);
    //fitness = 1 - fitness;// this would work if no exits on collision
    if (!StatFuns::in_range(fitness, 0.0f, 1.0f))
    {
        throw std::runtime_error("fitness not in [0,1]");
    }
    fitness_per_trial.push_back(fitness);
    trial_dist = 0.0f;
};

/*after completing all trials, combine fitness*/
float Aggregation::after_trials()
{
    float meanfit = StatFuns::mean(fitness_per_trial);
    fitness_per_trial.clear();
    return meanfit;
};


Dispersion::Dispersion(BaseLoopFunctions *cLoopFunctions)
{
    argos::CVector3 max = cLoopFunctions->get_arenasize();
    maxdist = StatFuns::get_minkowski_distance(CVector3(max.GetX(), max.GetY(), 0.0f), argos::CVector3::ZERO);
    num_updates = argos::CSimulator::GetInstance().GetMaxSimulationClock(); // one update for the entire swarm

}
/*after completing trial, calc fitness*/
void Dispersion::apply(BaseLoopFunctions &cLoopFunctions)
{
    /*The fitness is proportional to the average distance to the nearest neighbour, 
    averaged over the entire simulation.*/
    float normalised_dist = trial_dist / (maxdist * (float)num_updates);
#ifdef PRINTING
    std::cout << "normalised dist " << normalised_dist << std::endl;
#endif
    if (!StatFuns::in_range(normalised_dist, 0.0f, 1.0f))
    {
        throw std::runtime_error("fitness not in [0,1]");
    }
    fitness_per_trial.push_back(normalised_dist);
    trial_dist = 0.0f;
}
/*after completing all trials, combine fitness*/
float Dispersion::after_trials()
{
    float meanfit = StatFuns::mean(fitness_per_trial);
    fitness_per_trial.clear();
    return meanfit;
}
void Dispersion::after_robotloop(BaseLoopFunctions &cLoopFunctions)
{
    trial_dist += avg_min_dist(cLoopFunctions);
}

float Dispersion::avg_min_dist(BaseLoopFunctions &cLoopFunctions)
{

    float avg_min_dist = 0.0f;
    for (size_t i = 0; i < cLoopFunctions.m_unNumberRobots; ++i)
    {
        float min_dist = std::numeric_limits<float>::infinity();

        for (size_t j = 0; j < cLoopFunctions.m_unNumberRobots; ++j)
        {
            if (i == j)
                continue;
            float dist = StatFuns::get_minkowski_distance(cLoopFunctions.curr_pos[i], cLoopFunctions.curr_pos[j]);
            if (dist < min_dist)
            {
                min_dist = dist;
            }
        }
        avg_min_dist += min_dist;
    }
    avg_min_dist /= (float)cLoopFunctions.m_unNumberRobots;
#ifdef PRINTING
    std::cout << "avg min dist: " << avg_min_dist << std::endl;
#endif
    return avg_min_dist;
}

Flocking::Flocking(BaseLoopFunctions *cLoopFunctions)
{
    flocking_range = 0.50f;// in meters, half the usual RAB range + avoid environment_diversity to have different fitness function
    accumulator = 0.0f;
    num_updates = argos::CSimulator::GetInstance().GetMaxSimulationClock(); // one update for the entire swarm
}
void Flocking::after_robotloop(BaseLoopFunctions &cLoopFunctions)
{
    float temp_accum = 0.0f;
    float num_calcs=0.0f;
    for (int i = 0; i < cLoopFunctions.m_unNumberRobots; ++i)
    {
        CVector3 pos_i = cLoopFunctions.curr_pos[i];
        CRadians theta_i = cLoopFunctions.old_theta[i]; 
        for (int j = i + 1; j < cLoopFunctions.m_unNumberRobots; ++j)
        {
            CVector3 pos_j = cLoopFunctions.curr_pos[j];

            float dist = StatFuns::get_minkowski_distance(pos_i,pos_j);
#ifdef PRINTING
            std::cout<<"position agent "<< i <<" "<< pos_i <<std::endl;
            std::cout<<"position agent "<< j <<" "<< pos_j <<std::endl;
            std::cout<<"flocking range "<<flocking_range << std::endl;
            std::cout<<"distance "<<dist<<std::endl;
#endif
            if (dist < flocking_range)
            {
                float angleDifference = argos::NormalizedDifference(theta_i,cLoopFunctions.old_theta[j]).GetValue();
                angleDifference = std::min((2.0f * BOOST_PI) - std::abs(angleDifference), std::abs(angleDifference)); // [0,PI]

                float vi = cLoopFunctions.actual_linear_velocity_signed(i);                                           // [-1,1]
                float vj = cLoopFunctions.actual_linear_velocity_signed(j);                                           // [-1,1]
                // v1 and v2 need the same sign
                // angle differences greater than pi/2 do not count
                temp_accum += (1.0 - std::min(1.0, angleDifference * 2.0 / BOOST_PI)) * std::max(0.0f, vi * vj);
#ifdef PRINTING

                std::cout<<"orientation agent "<< i <<" "<<theta_i <<std::endl;
                std::cout<<"orientation agent "<< j <<" "<< cLoopFunctions.old_theta[j] <<std::endl;
                std::cout <<"angle diff "<< angleDifference << std::endl;
                std::cout<<"signedvelocity "<< i <<" "<< vi <<std::endl;
                std::cout<<"signedvelocity "<< j <<" "<< vj <<std::endl;
                std::cout<<"accumulator "<< temp_accum <<std::endl;

#endif
            }
            num_calcs += 1.0f;
        }
    }
    temp_accum = temp_accum / num_calcs;// divide by the number of robot pairs
    accumulator += temp_accum;
}
/*after completing trial, calc fitness*/
void Flocking::apply(BaseLoopFunctions &cLoopFunctions)
{
    /*The fitness function rewards robots for having an orientation 
   similar to the other robots within a radius of 25 cm (half the robot sensing range), 
   and for moving as fast as possible.*/
    float fitness = accumulator / (float) num_updates;
#ifdef PRINTING
    std::cout << "flocking fitness " << fitness << std::endl;
#endif
    fitness_per_trial.push_back(fitness);
    if (!StatFuns::in_range(fitness, 0.0f, 1.0f))
    {
        throw std::runtime_error("fitness not in [0,1]");
    }
    accumulator = 0.0f;
}
/*after completing all trials, combine fitness*/
float Flocking::after_trials()
{
    float meanfit = StatFuns::mean(fitness_per_trial);
    fitness_per_trial.clear();
    return meanfit;
}


/****************************************************************/


Chaining::Chaining()
{
   num_updates = argos::CSimulator::GetInstance().GetMaxSimulationClock(); // one update for the entire swarm
}

void Chaining::before_trial(BaseLoopFunctions &cLoopFunctions)
{
   cLoopFunctions.m_pcvecRobot[src_robot_id]->GetControllableEntity().SetController("station_controller");
   cLoopFunctions.m_pcvecRobot[dest_robot_id]->GetControllableEntity().SetController("station_controller");

   Real max_sq_dist = -1000.0;
   size_t max_i, max_j;
   for (size_t i = 0; i < cLoopFunctions.m_pcvecRobot.size(); ++i) {
       for (size_t j = i + 1; j < cLoopFunctions.m_pcvecRobot.size(); ++j)
       {
           Real dist = SquareDistance(
                   cLoopFunctions.m_pcvecRobot[i]->GetEmbodiedEntity().GetOriginAnchor().Position,
                   cLoopFunctions.m_pcvecRobot[j]->GetEmbodiedEntity().GetOriginAnchor().Position);
           if (dist > max_sq_dist) {
               max_sq_dist = dist;
               max_i = i;
               max_j = j;
           }
       }
   }

   //std::cout << "src thymio" << src_robot_id << ", dest thymio" << dest_robot_id << ", max_i "<< max_i << ", max_j " << max_j << ": Dist: " << max_sq_dist << std::endl;

   if (max_j != src_robot_id && max_j != dest_robot_id && max_i != src_robot_id && max_i != dest_robot_id)
   {
       // all different swap all
       swap_robots(cLoopFunctions, max_j, src_robot_id);
       swap_robots(cLoopFunctions, max_i, dest_robot_id);
   } else if ((max_j == src_robot_id && max_i == dest_robot_id) || (max_j == dest_robot_id && max_i == src_robot_id)) {
       //both same no swap
       //std::cout << "no swap" << std::endl;
   } else if (max_i == src_robot_id) {
       //std::cout << "swap " << max_j << " and " << dest_robot_id << std::endl;
       swap_robots(cLoopFunctions, max_j, dest_robot_id);
   } else if (max_i == dest_robot_id) {
       //std::cout << "swap " << max_j << " and " << src_robot_id << std::endl;
       swap_robots(cLoopFunctions, max_j, src_robot_id);
   } else if (max_j == src_robot_id) {
       //std::cout << "swap " << max_i << " and " << dest_robot_id << std::endl;
       swap_robots(cLoopFunctions, max_i, dest_robot_id);
   } else if (max_j == dest_robot_id) {
       //std::cout << "swap " << max_i << " and " << src_robot_id << std::endl;
       swap_robots(cLoopFunctions, max_i, src_robot_id);
   }
}

void Chaining::swap_robots (BaseLoopFunctions &cLoopFunctions, size_t robot_a, size_t robot_b)
{
   CVector3 new_position = cLoopFunctions.m_pcvecRobot[robot_a]->GetEmbodiedEntity().GetOriginAnchor().Position;
   CQuaternion new_orientation = cLoopFunctions.m_pcvecRobot[robot_a]->GetEmbodiedEntity().GetOriginAnchor().Orientation;

   CVector3 temp_position = CVector3(1,1,0);

   CVector3 old_position = cLoopFunctions.m_pcvecRobot[robot_b]->GetEmbodiedEntity().GetOriginAnchor().Position;
   CQuaternion old_orientation = cLoopFunctions.m_pcvecRobot[robot_b]->GetEmbodiedEntity().GetOriginAnchor().Orientation;

   CVector3 size = cLoopFunctions.GetSpace().GetArenaSize();
   Real minX = 0.05f; // the 0.05m offset accounts for the wall thickness
   Real maxX = size.GetX() - 0.05f;
   Real minY = 0.05f;
   Real maxY = size.GetY() - 0.05;

   int num_tries = 0;
   // move to temp
   while (!cLoopFunctions.m_pcvecRobot[robot_b]->GetEmbodiedEntity().MoveTo( // move the body of the robot
           temp_position,                                   // to this position
           new_orientation,                                // with this orientation
           false                                       // this is not a check, leave the robot there
   ))
   {
       temp_position = CVector3(cLoopFunctions.m_pcRNG->Uniform(CRange<Real>(minX, maxX)), cLoopFunctions.m_pcRNG->Uniform(CRange<Real>(minY, maxY)), 0.0f);
       if (num_tries > 10000)
       {
           throw std::runtime_error("failed to swap robot positions; too many obstacles?");
       }
       ++num_tries;
   }
   // move to old
   if (!cLoopFunctions.m_pcvecRobot[robot_a]->GetEmbodiedEntity().MoveTo( // move the body of the robot
           old_position,                                   // to this position
           old_orientation,                                // with this orientation
           false                                       // this is not a check, leave the robot there
   )) {
       throw std::runtime_error("failed to swap robot positions");
   }
   // move to new
   if (!cLoopFunctions.m_pcvecRobot[robot_b]->GetEmbodiedEntity().MoveTo( // move the body of the robot
           new_position,                                   // to this position
           new_orientation,                                // with this orientation
           false                                       // this is not a check, leave the robot there
   )) {
       throw std::runtime_error("failed to swap robot positions");
   }
}

void Chaining::after_robotloop(BaseLoopFunctions &cLoopFunctions)
{
   float maxdist = StatFuns::get_minkowski_distance(cLoopFunctions.curr_pos[src_robot_id], cLoopFunctions.curr_pos[dest_robot_id]);
   float mindist = min_connected_dist(cLoopFunctions, maxdist);

#ifdef PRINTING
   std::cout << "Current Dist: " << (1 - (mindist / maxdist)) << std::endl;
#endif
   trial_dist += 1 - (mindist / maxdist);
}

float Chaining::min_connected_dist(BaseLoopFunctions &cLoopFunctions, float maxdist)
{
   std::vector<size_t> src_connected = {src_robot_id};
   std::vector<size_t> closed_set = {};
   std::vector<size_t> open_set = {src_robot_id};

   while (!open_set.empty())
   {
       size_t current = open_set.back();
       open_set.pop_back();
       closed_set.push_back(current);
       BaseController& controller = dynamic_cast<BaseController&>(cLoopFunctions.m_pcvecRobot[current]->GetControllableEntity().GetController());
       auto RAB = controller.m_pcRABS->GetReadings();

       for (int i = 0; i < RAB.size(); ++i) {
           if (RAB[i].Data[1] == dest_robot_id)
               return 0;
           if(std::find(closed_set.begin(), closed_set.end(), RAB[i].Data[1]) != closed_set.end()) {
               continue;
           } else {
               src_connected.push_back(RAB[i].Data[1]);
               open_set.push_back(RAB[i].Data[1]);
           }
       }
   }

   std::vector<size_t> dest_connected = {dest_robot_id};
   closed_set = {};
   open_set = {dest_robot_id};
   while (!open_set.empty())
   {
       size_t current = open_set.back();
       open_set.pop_back();
       closed_set.push_back(current);
       BaseController& controller = dynamic_cast<BaseController&>(cLoopFunctions.m_pcvecRobot[current]->GetControllableEntity().GetController());
       auto RAB = controller.m_pcRABS->GetReadings();

       for (int i = 0; i < RAB.size(); ++i) {
           if (RAB[i].Data[1] == src_robot_id)
               return 0;
           if(std::find(closed_set.begin(), closed_set.end(), RAB[i].Data[1]) != closed_set.end()) {
               continue;
           } else {
               dest_connected.push_back(RAB[i].Data[1]);
               open_set.push_back(RAB[i].Data[1]);
           }
       }
   }

   float min_dist = maxdist;
   for(size_t i = 0; i < src_connected.size(); ++i)
   {
       for (int j = 0; j < dest_connected.size(); ++j)
       {
           float dist = StatFuns::get_minkowski_distance(cLoopFunctions.curr_pos[src_connected[i]], cLoopFunctions.curr_pos[dest_connected[j]]);
           if(min_dist > dist)
           {
               min_dist = dist;
           }
       }
   }
   return min_dist;
}

/*after completing trial, calc fitness*/
void Chaining::apply(BaseLoopFunctions &cLoopFunctions)
{
   // The fitness function is inversely proportional to
   // the average distance between the closest robots connected to the source and destination by RAB.
   // This includes the source and destination robots themselves.
   float fitness = trial_dist / ((float)num_updates);
   if (!StatFuns::in_range(fitness, 0.0f, 1.0f))
   {
       throw std::runtime_error("fitness not in [0,1]");
   }
   fitness_per_trial.push_back(fitness);
   //std::cout << "fitness of trial " << fitness << std::endl;
   trial_dist = 0.0f;
}
/*after completing all trials, combine fitness*/
float Chaining::after_trials()
{
   float meanfit = StatFuns::mean(fitness_per_trial);
   fitness_per_trial.clear();
   //std::cout << "MEAN FITNESS " << meanfit << std::endl;
   return meanfit;
}


/***********************************************************/


void Foraging::before_trial(BaseLoopFunctions &cLoopFunctions)
{

    m_bRobotsHoldingFood.clear();
    for (size_t i=0; i < cLoopFunctions.curr_pos.size(); i++) {
        m_bRobotsHoldingFood.push_back(false);
    }
    m_cVisitedFood.clear();
    for (size_t i=0; i < m_cFoodPos.size(); i++) {
        m_cVisitedFood.push_back(false);
    }
    numfoodCollected = 0;
    num_updates = 0;
    /// Calculate max fitness
}

void Foraging::after_robotloop(BaseLoopFunctions &cLoopFunctions)
{
    for(size_t i = 0; i < cLoopFunctions.curr_pos.size(); i++)
    {
        /* Get the position of the thymio on the ground as a CVector2 */
        CVector3 cPos = cLoopFunctions.curr_pos[i];

        /* The thymio has a food item */
        if(m_bRobotsHoldingFood[i])
        {
            /* Check whether the thymio is in the nest */
            if(cPos.GetX() < nest_x)
            {
                /* Drop the food item */
                m_bRobotsHoldingFood[i] = false;
                /* Increase the food count */
                numfoodCollected++;
#ifdef PRINTING
                std::cout << "thymio" << i << " dropped off food. Total collected: " << numfoodCollected << std::endl;
#endif
            }
        }
        else {
            /* The thymio has no food item */
            /* Check whether the thymio is out of the nest */
            if(cPos.GetX() > nest_x)
            {
                /* Check whether the thymio is on a food item that has not already been collected*/
                bool bDone = false;
                for(size_t f = 0; f < num_food && !bDone; ++f)
                {
                    float dx = pow(cPos.GetX() - m_cFoodPos[f].GetX(), 2);// squared dist
                    float dy = pow(cPos.GetY() - m_cFoodPos[f].GetY(), 2);// squared dist
                    if(dx + dy < m_fFoodSquareRadius[f] && !m_cVisitedFood[f]) // check squared dist smaller than squared radius
                    {
                        /* The thymio is now carrying an item */
                        m_bRobotsHoldingFood[i] = true;
                        /* the food has now been visited */
                        m_cVisitedFood[f] = true;
                        /* We are done */
                        bDone = true;
                        break;
                        //std::cout << "thymio" << i << " picked up food " << f << std::endl;
                    }
                }
            }
        }
    }
    reward = numfoodCollected / (float) num_food;
    trial_performance += reward;
    num_updates++;
}

void Foraging::apply(BaseLoopFunctions &cLoopFunctions)
{
    float fitness = trial_performance / (float(num_updates));
#ifdef PRINTING
    std::cout << trial_performance << " / " << num_updates << std::endl;
#endif
    fitness_per_trial.push_back(fitness);
    trial_performance = 0;
}

float Foraging::after_trials()
{
    float meanfit = StatFuns::mean( fitness_per_trial);
    fitness_per_trial.clear();
    return meanfit;
}