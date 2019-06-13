

#include <src/core/base_loop_functions.h>
#include <src/core/fitness_functions.h>
#include <src/core/statistics.h>

FloreanoMondada::FloreanoMondada() : FitFun()
{
}

void FloreanoMondada::after_robotloop(BaseLoopFunctions &cLoopFunctions)
{
    // compute linear speed for fitness function
    float s = (fabs(cLoopFunctions.outf[0]) + fabs(cLoopFunctions.outf[1])) / 20.0; // in [0,1]
    float ds = fabs(cLoopFunctions.outf[0] - cLoopFunctions.outf[1]) / 20.0;        // in [0,1]
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
    float s = (fabs(cLoopFunctions.outf[0]) + fabs(cLoopFunctions.outf[1])) / 20.0; // in [0,1]
    float ds = fabs(cLoopFunctions.outf[0] - cLoopFunctions.outf[1]) / 20.0;        // in [0,1]
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
    if (init_string=="Coverage")
    {
        coverageCalc = new CoverageCalc(cLoopFunctions);
    }
    else if(init_string=="BorderCoverage")
    {
        coverageCalc = new BorderCoverageCalc(cLoopFunctions);
    }
    else{
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
    for (size_t robot_index=0; robot_index < cLoopFunctions.curr_pos.size(); ++robot_index)
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

Aggregation::Aggregation() : FitFun()
{
}
void Aggregation::after_robotloop(BaseLoopFunctions &cLoopFunctions)
{
    std::pair<std::vector<argos::CVector3>, argos::CVector3> data = centre_of_mass(cLoopFunctions);
    trial_dist +=  StatFuns::get_avg_dist(data.first, data.second);
    ++num_updates;
}
/*after completing a trial,calc fitness*/
void Aggregation::apply(BaseLoopFunctions &cLoopFunctions, Real time)
{
    //The fitness function is inversely
    //proportional to the average distance to the centre of mass over the entire simulation
    float dist = trial_dist / (float) num_updates;
    fitness_per_trial.push_back(1.0f / dist);
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
    for (size_t i=0; i < cLoopFunctions.curr_pos.size(); ++i)
    {

        float mass = 1.0; //get_mass(robot);
        M += mass;
        argos::CVector3 pos = cLoopFunctions.curr_pos[i];
        cm += pos;  //mass * pos;
        positions.push_back(pos);
    }
    cm /= M;
#ifdef PRINTING
    std::cout << "centre of mass: " << cm << std::endl;
#endif
    return std::pair<std::vector<argos::CVector3>, argos::CVector3>(positions, cm);
}


Dispersion::Dispersion(BaseLoopFunctions* cLoopFunctions)
{
    argos::CVector3 max = cLoopFunctions->GetSpace().GetArenaSize();
    maxdist = StatFuns::get_minkowski_distance(max,argos::CVector3::ZERO);
}
/*after completing trial, calc fitness*/
void Dispersion::apply(BaseLoopFunctions &cLoopFunctions, Real time)
{
    /*The fitness is proportional to the average distance to the nearest neighbour, 
    averaged over the entire simulation.*/
    float normalised_dist = trial_dist/(maxdist*num_updates);
#ifdef PRINTING
    std::cout<<"normalised dist "<<  normalised dist  <<std::endl;
#endif
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

    float avg_min_dist=0.0f;
    for (size_t i =0 ; i < cLoopFunctions.curr_pos.size(); ++i)
    {
        float min_dist=std::numeric_limits<float>::infinity();

        for (size_t j=0; j < cLoopFunctions.curr_pos.size(); ++j)
        {
            if (i==j)
                continue;
            float dist  = StatFuns::get_minkowski_distance(cLoopFunctions.curr_pos[i],cLoopFunctions.curr_pos[j]);
            if (dist < min_dist)
            {
                min_dist = dist;
            }
        }
        avg_min_dist+=min_dist;
    }
    avg_min_dist/= (float) cLoopFunctions.curr_pos.size();
#ifdef PRINTING
    std::cout << "avg min dist: " << avg_min_dist << std::endl;
#endif
    return avg_min_dist;
}


// Flocking::Flocking(BaseLoopFunctions *cLoopFunctions)
// {

// }
// void Flocking::after_robotloop(BaseLoopFunctions &cLoopFunctions)
// {
//     trial_dist += avg_min_dist(cLoopFunctions);
//     ++num_updates;
// }
// /*after completing trial, calc fitness*/
// void Flocking::apply(BaseLoopFunctions &cLoopFunctions, Real time)
// {
//   /*The fitness function rewards robots for having an orientation 
//    similar to the other robots within a radius of 25 cm (half the robot sensing range), 
//    and for moving as fast as possible.*/

// }
// /*after completing all trials, combine fitness*/
// float Flocking::after_trials()
// {
//     float meanfit = StatFuns::mean(fitness_per_trial);
//     fitness_per_trial.clear();
//     return meanfit;
// }
