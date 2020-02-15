#ifndef FORAGING_LOOP_FUNCTIONS_H
#define FORAGING_LOOP_FUNCTIONS_H

#include "src/evolution/base_evol_loop_functions.h"
#include <argos3/core/simulator/entity/floor_entity.h>

class CForagingLoopFunctions : public BaseEvolutionLoopFunctions
{

public:
    CFloorEntity *m_pcFloor;
    const float nest_x = 0.32;
    const size_t num_food = 5;
    const int HARVEST_TIME = 50; // 50 time steps
    const std::vector<float> m_fFoodSquareRadius = {
        0.10 * 0.10, 0.10 * 0.10, 0.20 * 0.20, 0.20 * 0.20, 0.30 * 0.30};
    const std::vector<CVector2> m_cFoodPos = {

        CVector2(0.80, 1.20),
        CVector2(0.80, 0.50),
        CVector2(1.3, 1.0),
        CVector2(1.5, 0.5),
        CVector2(1.6, 1.70)

    };
    std::vector<int> m_cVisitedFood = {}; // how much time steps left until harvestable
    std::vector<bool> m_bRobotsHoldingFood = {};
    size_t numfoodCollected = 0;
    CForagingLoopFunctions();
    virtual ~CForagingLoopFunctions() {}

    virtual void Init(TConfigurationNode &t_tree);
    virtual void Reset();
    virtual void Destroy();
    virtual CColor GetFloorColor(const CVector2 &c_position_on_plane);
    virtual void PostStep();
};

#endif
