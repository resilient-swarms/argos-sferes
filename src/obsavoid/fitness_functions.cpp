

#include <src/obsavoid/evol_loop_functions.h>
#include <src/obsavoid/fitness_functions.h>
#include <src/obsavoid/statistics.h>

FloreanoMondada::FloreanoMondada() : FitFun()
{
}
/*after completing a trial,calc fitness*/
void FloreanoMondada::apply(CObsAvoidEvolLoopFunctions &cLoopFunctions, Real time)
{
    float fit = cLoopFunctions.lin_speed / time * (Real)cLoopFunctions.nb_coll / time;
    fitness_per_trial.push_back(fit);
};

/*after completing all trials, combine fitness*/
float FloreanoMondada::after_trials()
{
    float minfit = StatFuns::min(fitness_per_trial);
    fitness_per_trial.clear();
    return minfit;
};
MeanSpeed::MeanSpeed() : FitFun()
{
}

/*after completing a trial,calc fitness*/
void MeanSpeed::apply(CObsAvoidEvolLoopFunctions &cLoopFunctions, Real time)
{
    float fit = cLoopFunctions.lin_speed / time * (Real)cLoopFunctions.nb_coll / time;
    fitness_per_trial.push_back(fit);
};

/*after completing all trials, combine fitness*/
float MeanSpeed::after_trials()
{
    float meanfit = StatFuns::mean(fitness_per_trial);
    fitness_per_trial.clear();
    return meanfit;
};

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
