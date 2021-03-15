#ifndef FORAGING_LOOP_FUNCTIONS_H
#define FORAGING_LOOP_FUNCTIONS_H

#include "src/evolution/base_evol_loop_functions.h"
#include <argos3/core/simulator/entity/floor_entity.h>
#include "src/evolution/virtual_energy.h"
#include "src/evolution/foraging_stats.h"

#if HETEROGENEOUS
#include <src/ite/ite_swarms.hpp>
#include <src/ite/baselines.hpp>
#endif

class CForagingLoopFunctions : public BaseEvolutionLoopFunctions
{

public:
    ForagingStats *stats;
    VirtualEnergy *virtual_energy;
    CFloorEntity *m_pcFloor;
    float scale = 1.0f; // scale of the arena
#if HETEROGENEOUS
    std::string optimisation;
#if !RECORD_FIT

    bool load_ID_map;
    size_t num_subtrials;
    size_t ticks_per_subtrial;
    bool reset;
    std::vector<Opt_t *> opt;          // in case we want bayesian optimisation
    std::vector<Proposal *> proposals; //in case we want random search
    ControllerEval state_fun;          //state function; just for its parameters
    Eigen::VectorXd current_sample;
    std::string stop_crit;
    bool wait_until_allfinished=false;

    //std::string network_config, network_binary, archive_file;
    bool stop_criterion(ForagingThymioNN &cController)
    {

        if (stop_crit == "both")
        {
            return cController.collision_stop() || cController.reward_stop();
        }
        else if (stop_crit == "collision")
        {
            return cController.collision_stop();
        }
        else if (stop_crit == "reward")
        {
            return cController.reward_stop();
        }
        else
        {
            return false;
        }
    }
    void reset_controller(size_t j, bool reset, bool alltrialsfinished);
    void update_model(ForagingThymioNN &cController, size_t stat_index, bool alltrialsfinished, bool multi);
    void select_new_controller(ForagingThymioNN &cController, bool alltrialsfinished);
    void select_new_joint_controller(bool alltrialsfinished);
    void select_new_controller_random(ForagingThymioNN &cController, bool alltrialsfinished);
    void update_joint_model(bool alltrialsfinished);
    void check_ID_map(std::vector<float> ident);
    void init_BO(std::vector<double> normal_ID);
    void init_randomsearch();
    void init_multiBO(bool single_worker, bool sharing);
    void init_BOjoint();
#endif
#endif
    const float nest_x = 0.32;
#ifdef LARGE_ARENA
    const int HARVEST_TIME = 0; // no delay
    std::vector<float> m_fFoodSquareRadius = {
        0.10 * 0.10, 0.10 * 0.10, 0.20 * 0.20, 0.20 * 0.20, 0.30 * 0.30,
        0.10 * 0.10, 0.10 * 0.10, 0.20 * 0.20, 0.20 * 0.20, 0.30 * 0.30};
    std::vector<CVector2> m_cFoodPos = {

        CVector2(0.80, 1.20),
        CVector2(0.80, 0.50),
        CVector2(1.3, 1.0),
        CVector2(1.5, 0.5),
        CVector2(1.6, 1.70),
        CVector2(0.80, 2.1 + 1.20),
        CVector2(0.80, 2.1 + 0.50),
        CVector2(1.3, 2.1 + 1.0),
        CVector2(1.5, 2.1 + 0.5),
        CVector2(1.6, 2.1 + 1.70)

    };

#else
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
#endif
    bool forcePositions = false;
    std::vector<int> m_cVisitedFood = {}; // how much time steps left until harvestable
    std::vector<bool> m_bRobotsHoldingFood = {};
    std::vector<int> m_numFoodCollected = {};
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
    virtual float get_robot_fitness(size_t i)
    {
        return m_unNumberRobots * m_numFoodCollected[i] / (float)m_unNumberTrials;
    };
    std::vector<int> get_fault_indexmap();
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
