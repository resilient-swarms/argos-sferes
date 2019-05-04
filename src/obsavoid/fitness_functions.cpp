

#include <src/obsavoid/evol_loop_functions.h>
#include <src/obsavoid/fitness_functions.h>
#include <src/obsavoid/statistics.h>



FloreanoMondada::FloreanoMondada() : FitFun()
{
}

void FloreanoMondada::after_step(size_t robot, CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
    // compute linear speed for fitness function
    float s = (fabs(cLoopFunctions.outf[0]) + fabs(cLoopFunctions.outf[1])) / 20.0; // in [0,1]
    float ds = fabs(cLoopFunctions.outf[0] - cLoopFunctions.outf[1]) / 20.0;        // in [0,1]
    speed += s;
    float curr_lin_speed = s * (1.0 - sqrt(ds));
    lin_speed += curr_lin_speed;
    num_ds += (ds >= 0.1) ? 1.0 : 0.0;
    nb_coll += (1.0f - cLoopFunctions.maxIRSensor);
}
/*after completing a trial,calc fitness*/
void FloreanoMondada::apply(CObsAvoidEvolLoopFunctions &cLoopFunctions, Real time)
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


void MeanSpeed::after_step(size_t robot, CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
    // compute linear speed for fitness function
    float s = (fabs(cLoopFunctions.outf[0]) + fabs(cLoopFunctions.outf[1])) / 20.0; // in [0,1]
    float ds = fabs(cLoopFunctions.outf[0] - cLoopFunctions.outf[1]) / 20.0;        // in [0,1]
    speed += s;
    float curr_lin_speed = s * (1.0 - sqrt(ds));
    lin_speed += curr_lin_speed;
    num_ds += (ds >= 0.1) ? 1.0 : 0.0;
    nb_coll += (1.0f - cLoopFunctions.maxIRSensor);
}
/*after completing a trial,calc fitness*/
void MeanSpeed::apply(CObsAvoidEvolLoopFunctions &cLoopFunctions, Real time)
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
Coverage::Coverage(CObsAvoidEvolLoopFunctions *cLoopFunctions)
{
    coverageCalc = CoverageCalc(cLoopFunctions);
}

/*after completing trial, calc fitness*/
void Coverage::apply(CObsAvoidEvolLoopFunctions &cLoopFunctions, Real time)
{
    //coverage
    fitness_per_trial.push_back(coverageCalc.get_coverage());
    num_updates = 0;
    coverageCalc.after_trial();
}
/*after a single step of single agent */
void Coverage::after_step(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
    coverageCalc.update(cLoopFunctions.curr_pos);
    ++num_updates;
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


Aggregation::Aggregation() : FitFun()
{
}
/*after completing a trial,calc fitness*/
void Aggregation::apply(CObsAvoidEvolLoopFunctions &cLoopFunctions, Real time)
{
    std::pair<std::vector<argos::CVector3>, argos::CVector3> data = centre_of_mass(cLoopFunctions);
    float dist = StatFuns::get_avg_dist(data.first, data.second);
    fitness_per_trial.push_back(1.0f / dist);
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

std::pair<std::vector<argos::CVector3>, argos::CVector3> Aggregation::centre_of_mass(CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
    float M = 0.0;
    argos::CVector3 cm = argos::CVector3(0., 0., 0.);
    std::vector<argos::CVector3> positions;

    for (CThymioEntity *robot : cLoopFunctions.m_pcvecRobot)
    {

        float mass = get_mass(robot);
        M += mass;
        argos::CVector3 pos = cLoopFunctions.get_position(robot);
        cm += mass * pos;
        positions.push_back(pos);
    }
    cm /= M;
#ifdef PRINTING
    std::cout << "centre of mass: " << cm << std::endl;
#endif
    return std::pair<std::vector<argos::CVector3>, argos::CVector3>(positions, cm);
}
