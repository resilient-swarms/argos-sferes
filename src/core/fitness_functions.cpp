

#include <src/core/base_loop_functions.h>
#include <src/core/fitness_functions.h>
#include <src/core/statistics.h>

FloreanoMondada::FloreanoMondada() : FitFun()
{
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
void FloreanoMondada::apply(BaseLoopFunctions &cLoopFunctions, Real time)
{
    float fit = lin_speed / time * (Real)nb_coll / time;
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
void MeanSpeed::apply(BaseLoopFunctions &cLoopFunctions, Real time)
{
    float fit = lin_speed / time * (Real)nb_coll / time;
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
}

/*after completing trial, calc fitness*/
void Coverage::apply(BaseLoopFunctions &cLoopFunctions, Real time)
{
    //coverage
    fitness_per_trial.push_back(coverageCalc->get_coverage());
    num_updates = 0;
    coverageCalc->after_trial();
}

/*after a single step of single agent */
void Coverage::after_robotloop(BaseLoopFunctions &cLoopFunctions)
{
    for (size_t robot_index = 0; robot_index < cLoopFunctions.curr_pos.size(); ++robot_index)
    {
        coverageCalc->update(cLoopFunctions.curr_pos[robot_index]);
        ++num_updates;
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
}

/*after completing trial, calc fitness*/
void DecayCoverage::apply(BaseLoopFunctions &cLoopFunctions, Real time)
{
    //coverage
    float sum = coverageCalc->accumulator;
    float coverage = sum / ((float)coverageCalc->grid.size() * (float)num_updates);
    if (!StatFuns::in_range(coverage, 0.0f, 1.0f))
    {
        throw std::runtime_error("fitness not in [0,1]");
    }
    fitness_per_trial.push_back(coverage);
    num_updates = 0;
    coverageCalc->end_trial();
}

/*after a single step of single agent */
void DecayCoverage::after_robotloop(BaseLoopFunctions &cLoopFunctions)
{
    for (size_t robot_index = 0; robot_index < cLoopFunctions.curr_pos.size(); ++robot_index)
    {
        coverageCalc->update(cLoopFunctions.curr_pos[robot_index]);
    }
    coverageCalc->get_grid_sum();
    ++num_updates; // one update for the entire swarm
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
    argos::CVector3 max = cLoopFunctions->GetSpace().GetArenaSize();
    maxdist = StatFuns::get_minkowski_distance(CVector3(max.GetX(), max.GetY(), 0.0f), argos::CVector3::ZERO);
}
void Aggregation::after_robotloop(BaseLoopFunctions &cLoopFunctions)
{
    std::pair<std::vector<argos::CVector3>, argos::CVector3> data = centre_of_mass(cLoopFunctions);
    trial_dist += StatFuns::get_avg_dist(data.first, data.second);
    ++num_updates;
}
/*after completing a trial,calc fitness*/
void Aggregation::apply(BaseLoopFunctions &cLoopFunctions, Real time)
{
    //The fitness function is inversely
    //proportional to the average distance to the centre of mass over the entire simulation
    float dist = trial_dist / (maxdist * (float)num_updates);
    float fitness = 1 - dist;
    if (!StatFuns::in_range(fitness, 0.0f, 1.0f))
    {
        throw std::runtime_error("fitness not in [0,1]");
    }
    fitness_per_trial.push_back(fitness);
    num_updates = 0;
    trial_dist = 0.0f;
};

/*after completing all trials, combine fitness*/
float Aggregation::after_trials()
{
    float meanfit = StatFuns::mean(fitness_per_trial);
    fitness_per_trial.clear();
    return meanfit;
};
float Aggregation::get_mass(CThymioEntity *robot)
{
    return 1.0f;
}

std::pair<std::vector<argos::CVector3>, argos::CVector3> Aggregation::centre_of_mass(BaseLoopFunctions &cLoopFunctions)
{
    float M = 0.0;
    argos::CVector3 cm = argos::CVector3(0., 0., 0.);
    std::vector<argos::CVector3> positions;

    // for (CThymioEntity *robot : cLoopFunctions.m_pcvecRobot)
    // {

    //     float mass = get_mass(robot);
    //     M += mass;
    //     argos::CVector3 pos = cLoopFunctions.get_position(robot);
    //     cm += mass * pos;
    //     positions.push_back(pos);
    // }
    for (size_t i = 0; i < cLoopFunctions.curr_pos.size(); ++i)
    {

        float mass = 1.0; //get_mass(robot); mass of 1 is used here
        M += mass;
        argos::CVector3 pos = cLoopFunctions.curr_pos[i];
        cm += pos; //mass * pos;
        positions.push_back(pos);
    }
    cm /= M;
#ifdef PRINTING
    std::cout << "centre of mass: " << cm << std::endl;
#endif
    return std::pair<std::vector<argos::CVector3>, argos::CVector3>(positions, cm);
}

Dispersion::Dispersion(BaseLoopFunctions *cLoopFunctions)
{
    argos::CVector3 max = cLoopFunctions->GetSpace().GetArenaSize();
    maxdist = StatFuns::get_minkowski_distance(CVector3(max.GetX(), max.GetY(), 0.0f), argos::CVector3::ZERO);
}
/*after completing trial, calc fitness*/
void Dispersion::apply(BaseLoopFunctions &cLoopFunctions, Real time)
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
    num_updates = 0;
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
    ++num_updates;
}

float Dispersion::avg_min_dist(BaseLoopFunctions &cLoopFunctions)
{

    float avg_min_dist = 0.0f;
    for (size_t i = 0; i < cLoopFunctions.curr_pos.size(); ++i)
    {
        float min_dist = std::numeric_limits<float>::infinity();

        for (size_t j = 0; j < cLoopFunctions.curr_pos.size(); ++j)
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
    avg_min_dist /= (float)cLoopFunctions.curr_pos.size();
#ifdef PRINTING
    std::cout << "avg min dist: " << avg_min_dist << std::endl;
#endif
    return avg_min_dist;
}

Flocking::Flocking(BaseLoopFunctions *cLoopFunctions)
{
    BaseController *ctrl = cLoopFunctions->get_controller(0); // assume all robots have the same RAB range
    flocking_range = (float)ctrl->max_rab_range / (2.0f);       //half the RAB sensor range (in meters)
    accumulator = 0.0f;
}
void Flocking::after_robotloop(BaseLoopFunctions &cLoopFunctions)
{
    float temp_accum = 0.0f;
    float num_calcs=0.0f;
    for (int i = 0; i < cLoopFunctions.m_unNumberRobots; ++i)
    {
        CVector3 pos_i = cLoopFunctions.curr_pos[i];
        float ai = cLoopFunctions.curr_theta[i].GetValue(); 
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

                float aj = cLoopFunctions.curr_theta[j].GetValue();  // [0,2PI]
                float angleDifference = std::min((2.0f * BOOST_PI) - std::abs(ai - aj), std::abs(ai - aj)); // [0,PI]

                float vi = cLoopFunctions.actual_linear_velocity_signed(i);                                           // [-1,1]
                float vj = cLoopFunctions.actual_linear_velocity_signed(j);                                           // [-1,1]
                // v1 and v2 need the same sign
                // angle differences greater than pi/2 do not count
                temp_accum += (1.0 - std::min(1.0, angleDifference * 2.0 / BOOST_PI)) * std::max(0.0f, vi * vj);
#ifdef PRINTING

                std::cout<<"orientation agent "<< i <<" "<<ai <<std::endl;
                std::cout<<"orientation agent "<< j <<" "<< aj <<std::endl;
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
    ++num_updates;
}
/*after completing trial, calc fitness*/
void Flocking::apply(BaseLoopFunctions &cLoopFunctions, Real time)
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
    num_updates = 0;
    accumulator = 0.0f;
}
/*after completing all trials, combine fitness*/
float Flocking::after_trials()
{
    float meanfit = StatFuns::mean(fitness_per_trial);
    fitness_per_trial.clear();
    return meanfit;
}
