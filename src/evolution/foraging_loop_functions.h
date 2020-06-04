#ifndef FORAGING_LOOP_FUNCTIONS_H
#define FORAGING_LOOP_FUNCTIONS_H

#include "src/evolution/base_evol_loop_functions.h"
#include <argos3/core/simulator/entity/floor_entity.h>
#include "src/evolution/virtual_energy.h"
#include "src/evolution/foraging_stats.h"

#if HETEROGENEOUS
#include <src/ite/ite_swarms.hpp>
#endif

class CForagingLoopFunctions : public BaseEvolutionLoopFunctions
{

public:
    ForagingStats *stats;
    VirtualEnergy *virtual_energy;
    CFloorEntity *m_pcFloor;
#if HETEROGENEOUS & !RECORD_FIT
    size_t num_subtrials;
    size_t ticks_per_subtrial;
    Opt_t opt;
    ControllerEval state_fun; //state function; just for its parameters
    std::string stop_crit;
    
    //std::string network_config, network_binary, archive_file;
    bool stop_criterion(ForagingThymioNN &cController)
    {
        
        if (stop_crit=="both")
        {
            return  cController.collision_stop() || cController.reward_stop();
        }
        else if(stop_crit=="collision")
        {
            return  cController.collision_stop();
        }
        else if (stop_crit=="reward"){
            return cController.reward_stop();
        }
        else{
            return true;
        }
    }
    void select_new_controller(ForagingThymioNN &cController,bool alltrialsfinished);
#endif
    const float nest_x = 0.32;
    const int HARVEST_TIME = 50; // 50 time steps
    std::vector<float> m_fFoodSquareRadius = {
        0.10 * 0.10, 0.10 * 0.10, 0.20 * 0.20, 0.20 * 0.20, 0.30 * 0.30};
    std::vector<CVector2> m_cFoodPos = {

        CVector2(0.80, 1.20),
        CVector2(0.80, 0.50),
        CVector2(1.3, 1.0),
        CVector2(1.5, 0.5),
        CVector2(1.6, 1.70)

    };
    bool forcePositions = false;
    std::vector<int> m_cVisitedFood = {}; // how much time steps left until harvestable
    std::vector<bool> m_bRobotsHoldingFood = {};

    float avg_final_E = 0.0f;
    float avg_time = 0.0f;
    CForagingLoopFunctions();
    virtual ~CForagingLoopFunctions() {}

    virtual void Init(TConfigurationNode &t_tree);
    virtual void Reset();
    virtual void Destroy();
    virtual CColor GetFloorColor(const CVector2 &c_position_on_plane);
    virtual void PostStep();

    virtual bool try_robot_position(CVector3 &Position, CQuaternion &Orientation, const CRange<Real> x_range, const CRange<Real> y_range, const size_t m_unRobot, size_t &num_tries);
    virtual std::vector<size_t> priority_robotplacement();

#ifdef RECORD_FIT
    /* write fitness to file */
    virtual void write_fitness(float fFitness)
    {
        if (virtual_energy != NULL)
        {
            avg_final_E /= (float)this->m_unNumberTrials;
            avg_time /= (float)this->m_unNumberTrials;
            std::cout << "FINAL: avg_final_E " << avg_final_E << std::endl;
            std::cout << "FINAL: avg_time " << avg_time << std::endl;
            fitness_writer << fFitness << "\t" << avg_final_E << "\t" << avg_time << std::endl;
        }
        else
        {
            fitness_writer << fFitness << std::endl;
        }
        if (stats != NULL)
        {
            stats->write();
        }
    }
#endif
    void food_scarcity();

    void virtual_energy_finish_trial()
    {
#ifdef RECORD_FIT
        std::cout << "Final energy " << virtual_energy->E << std::endl;
        std::cout << "sim clock " << (float)GetSpace().GetSimulationClock() << std::endl;
        avg_final_E += virtual_energy->E + (float)GetSpace().GetSimulationClock();
        std::cout << "avg_final_E " << avg_final_E << std::endl;
        avg_time += (float)GetSpace().GetSimulationClock();
#endif
    }

    virtual void end_trial()
    {
        if (!stop_eval && virtual_energy != NULL) // already updated virtual_energy
        {
            virtual_energy_finish_trial();
        }
        BaseEvolutionLoopFunctions::end_trial();
    }
};

#endif
