#include "arena_utils.h"
#include <argos3/plugins/simulator/entities/cylinder_entity.h>



CoverageCalc::CoverageCalc(CObsAvoidEvolLoopFunctions *cLoopFunctions)
{
    // initialise grid (for calculating coverage and uniformity)
    argos::CVector3 max = cLoopFunctions->GetSpace().GetArenaSize();
    

    SBoundingBox bounding_box = cLoopFunctions->m_pcvecRobot[0]->GetEmbodiedEntity().GetBoundingBox();

    float xdim = bounding_box.MaxCorner.GetX() - bounding_box.MinCorner.GetX();
    float ydim = bounding_box.MaxCorner.GetY() - bounding_box.MinCorner.GetY();
    float max_dim_size = Max(xdim,ydim);
    grid_step = max_dim_size;
    
    float obstacle_cells = get_obstacle_area(cLoopFunctions);
    total_size = (max.GetX() * max.GetY()) / grid_step - obstacle_cells;
}

float CoverageCalc::get_obstacle_area(CObsAvoidEvolLoopFunctions *cLoopFunctions)
{
    CSpace::TMapPerType &argos_cylinders = cLoopFunctions->GetSpace().GetEntitiesByType("cylinder");
    float size=0.0f;
    for (CSpace::TMapPerType::iterator it = argos_cylinders.begin(); it != argos_cylinders.end(); ++it) //!TODO: Make sure the CSpace::TMapPerType does not change during a simulation (i.e it is not robot-position specific)
	{
        CCylinderEntity &cylinder = *any_cast<CCylinderEntity *>(it->second);
        //first calculate the area
		float r =  cylinder.GetRadius();
        // exploit relation between circle and square; 
        // cf https://math.stackexchange.com/questions/854535/how-can-i-find-the-smallest-enclosing-circle-for-a-rectangle
        // R = sqrt(2* a^2 )/2 --> 2 a^2 = 4 R^2  ---> area = a^2=2 R ^2
        float area =2*r*r;
        size_t additional_taken = std::floor(area / grid_step);
        size+=additional_taken;
	}
    return size;
}
/* get the actual coverage of a single trial */
float CoverageCalc::get_coverage(size_t num_updates)
{
    //coverage
    float max_visited_positions = total_size;
    return (float)unique_visited_positions.size() / (float)max_visited_positions;
}
/* get bin corresponding to a position in the space */
std::tuple<int, int, int> CoverageCalc::get_bin(argos::CVector3 vec)
{
    int binx = (int)((float)vec.GetX() / grid_step);
    int biny = (int)((float)vec.GetY() / grid_step);
    int binz = (int)((float)vec.GetZ() / grid_step);

    std::tuple<int, int, int> bin(binx, biny, binz);
    // #ifdef PRINTING
    //     std::cout<<"binned "<< vec  <<"into "<<binx<<","<<biny<<","<<binz<<std::endl;
    // #endif
    return bin;
}
 /* updated the visited positions */
void CoverageCalc::update(argos::CVector3 pos)
{
    //count the bin
    std::tuple<int, int, int> bin = get_bin(pos);
    auto find_result = unique_visited_positions.find(bin);
    if (find_result == unique_visited_positions.end())
    {
        unique_visited_positions.insert(std::pair<std::tuple<int, int, int>, size_t>(bin, 1));
    }
    else
    {
        unique_visited_positions[bin] += 1;
    }
}
/* after trial is finished, reset stats */
void CoverageCalc::after_trial()
{
    unique_visited_positions.clear();
}
/* get the visitation probabilities for each bin  */
std::vector<float> CoverageCalc::get_probs(size_t num_updates)
{
    std::vector<float> a;
    for (auto &pair : unique_visited_positions)
    {
        a.push_back(pair.second / (float)num_updates);
#ifdef PRINTING
        std::cout << "total visits" << num_updates << std::endl;
        std::cout << "location " << std::get<0>(pair.first) << "," << std::get<1>(pair.first) << "," << std::get<2>(pair.first) << std::endl;
        std::cout << "visits " << pair.second << std::endl;
        std::cout << "probability " << a.back() << std::endl;
#endif
    }

    return a;
}