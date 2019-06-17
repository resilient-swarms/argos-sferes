#include <src/core/base_loop_functions.h>
#include <src/core/arena_utils.h>

#include <argos3/plugins/simulator/entities/cylinder_entity.h>



CoverageCalc::CoverageCalc(BaseLoopFunctions *cLoopFunctions)
{
    define_grid(cLoopFunctions);
    get_num_cells(*cLoopFunctions);
}

void CoverageCalc::define_grid(BaseLoopFunctions* cLoopFunctions)
{
    // initialise grid (for calculating coverage and uniformity)
    argos::CVector3 max = cLoopFunctions->GetSpace().GetArenaSize();


    SBoundingBox bounding_box = cLoopFunctions->get_embodied_entity(0).GetBoundingBox();

    float xdim = bounding_box.MaxCorner.GetX() - bounding_box.MinCorner.GetX();
    float ydim = bounding_box.MaxCorner.GetY() - bounding_box.MinCorner.GetY();
    float max_dim_size = Max(xdim,ydim);
    grid_step = max_dim_size;

    
}

void CoverageCalc::get_num_cells(BaseLoopFunctions &cLoopFunctions)
{
    CSpace::TMapPerType &argos_cylinders = cLoopFunctions.GetSpace().GetEntitiesByType("cylinder");
    float obstacle_cells=0.0f;
    for (CSpace::TMapPerType::iterator it = argos_cylinders.begin(); it != argos_cylinders.end(); ++it) //!TODO: Make sure the CSpace::TMapPerType does not change during a simulation (i.e it is not robot-position specific)
	{
        CCylinderEntity &cylinder = *any_cast<CCylinderEntity *>(it->second);

        CVector3 pos = cylinder.GetEmbodiedEntity().GetOriginAnchor().Position;

        //identify the enclosed rectangle
		float r =  cylinder.GetRadius();

        float a = sqrt(2)*r*r;
        float half_a_pos = a/2;
        float x_min = pos.GetX() - half_a_pos;
        float x_max = pos.GetX() + half_a_pos;
        float y_min = pos.GetY() - half_a_pos;
        float y_max = pos.GetY() + half_a_pos;


        
        // exploit relation between circle and square; 
        // cf https://math.stackexchange.com/questions/854535/how-can-i-find-the-smallest-enclosing-circle-for-a-rectangle
        // R = sqrt(2* a^2 )/2 --> 2 a^2 = 4 R^2 --> a^2 = 2R^2 -> a = sqrt(2)R  ---> area = a^2=2 R ^2
       
        // now get the number of squares FULLY enclosed by the enclosing rectangle
        // assuming grid [0,grid_step,2*grid_step,...]
        size_t start_bin_x = std::ceil(x_min/grid_step);
        size_t end_bin_x = std::floor(x_max/grid_step);

        size_t start_bin_y = std::ceil(y_min/grid_step);
        size_t end_bin_y = std::floor(y_max/grid_step);

        if (start_bin_x >= end_bin_x || start_bin_y >= end_bin_y)
            continue;
        obstacle_cells+=(end_bin_x - start_bin_x)*(end_bin_y - start_bin_y);
       
        // float area =2*r*r;
        // size_t additional_taken = std::floor(area / (grid_step*grid_step));
        // size+=additional_taken;
	}
    argos::CVector3 max = cLoopFunctions.GetSpace().GetArenaSize();
    float total_size = (max.GetX() * max.GetY()) / (grid_step*grid_step);
    num_cells = std::ceil(total_size - obstacle_cells);
}
/* get the actual coverage of a single trial */
float CoverageCalc::get_coverage() const
{
    return (float)unique_visited_positions.size() / num_cells;
}
/* get bin corresponding to a position in the space */
location_t CoverageCalc::get_bin(argos::CVector3 vec) const
{
    int binx = (int)((float)vec.GetX() / grid_step);
    int biny = (int)((float)vec.GetY() / grid_step);
    int binz = (int)((float)vec.GetZ() / grid_step);

    location_t bin(binx, biny, binz);
    // #ifdef PRINTING
    //     std::cout<<"binned "<< vec  <<"into "<<binx<<","<<biny<<","<<binz<<std::endl;
    // #endif
    return bin;
}
 /* updated the visited positions */
void CoverageCalc::update(argos::CVector3 pos)
{
    //count the bin
    location_t bin = get_bin(pos);
    auto find_result = unique_visited_positions.find(bin);
    if (find_result == unique_visited_positions.end())
    {
        unique_visited_positions.insert(std::pair<location_t, size_t>(bin, 1));
    }
    else
    {
        unique_visited_positions[bin] += 1;
    }

    // #ifdef PRINTING
    // std::cout<<"the map is now"<<std::endl;
    // for(std::map<location_t,size_t>::iterator iter = unique_visited_positions.begin(); iter != unique_visited_positions.end(); ++iter)
    // {
    //     location_t k =  iter->first;
    //     print_xy(k);
    //     //ignore value
    //     //Value v = iter->second;
    // } 
    // std::cout<<"the map has size "<< unique_visited_positions.size() <<std::endl;  
    // #endif
}
/* after trial is finished, reset stats */
void CoverageCalc::after_trial()
{
    unique_visited_positions.clear();
}
/* get the visitation probabilities for each bin  */
std::vector<float> CoverageCalc::get_probs(size_t num_updates) const 
{
    std::vector<float> a;
    for (const auto &pair : unique_visited_positions)
    {
        a.push_back(pair.second / (float)num_updates);
#ifdef PRINTING
        std::cout << "total visits" << num_updates << std::endl;
        print_xy(pair.first);
        std::cout << "visits " << pair.second << std::endl;
        std::cout << "probability " << a.back() << std::endl;
#endif
    }

    return a;
}


/* print xy-location of an element in the visited positions */
void CoverageCalc::print_xy(location_t location) const
{
    std::cout<<"("<<std::get<0>(location)<<","<<std::get<1>(location)<<")"<<std::endl;
}

BorderCoverageCalc::BorderCoverageCalc(BaseLoopFunctions *cLoopFunctions) : CoverageCalc(cLoopFunctions)
{

}
void BorderCoverageCalc::get_num_cells(BaseLoopFunctions &cLoopFunctions)
{
    // note: simplifying assumption that obstacles are not placed on the border
    argos::CVector3 max = cLoopFunctions.GetSpace().GetArenaSize();
    size_t num_x_bins = (int) std::ceil(max.GetX()/grid_step);
    size_t num_y_bins = (int) std::ceil(max.GetX()/grid_step);
    max_bin_x = num_x_bins - 1;
    max_bin_y = num_y_bins - 1;

    num_cells = max_bin_x*2 + max_bin_y*2 - 4;// -4 to remove double counts at the edges
}

/* check whether bin is on the border*/
bool BorderCoverageCalc::is_on_border(location_t bin) const
{
   float x = std::get<0>(bin);
   float y = std::get<1>(bin);
   return x == 0  || y ==0 || x == max_bin_x || y == max_bin_y;
}
/* get bin corresponding to a position in the space */
location_t  BorderCoverageCalc::get_bin(argos::CVector3 vec) const
{
    location_t bin = CoverageCalc::get_bin(vec);
    if (is_on_border(bin) )
    {
        return bin;
    }
    else{
        
        return null_location;
    }
}
 /* updated the visited positions */
void BorderCoverageCalc::update(argos::CVector3 pos)
{
    //count the bin
    location_t bin = get_bin(pos);
    if (bin == null_location)  // binned position not on the border
    {
        return;
    }
    auto find_result = unique_visited_positions.find(bin);
    if (find_result == unique_visited_positions.end())
    {
        unique_visited_positions.insert(std::pair<location_t, size_t>(bin, 1));
    }
    else
    {
        unique_visited_positions[bin] += 1;
    }
}


DecayCoverageCalc::DecayCoverageCalc(std::string init_type, BaseLoopFunctions *cLoopFunctions)
{
    // note: simplifying assumption that obstacles are not placed on the border
    argos::CVector3 max = cLoopFunctions->GetSpace().GetArenaSize();
    grid_step_x = max.GetX()/grid_size_x;
    grid_step_y = max.GetY()/grid_size_y;

    //static argos::CSimulator &cSimulator = argos::CSimulator::GetInstance();
    //CPhysicsEngine p = cSimulator.GetPhysicsEngine("dyn2d");
    Real t = CPhysicsEngine::GetSimulationClockTick();// e.g., tick = 0.10 s
    size_t decay_period = (size_t) (1.0f / (float) t);// e.g. 10 ticks per second
    decay_rate = 0.005 / decay_period;// decay_rate is 0.005 per second or decay_rate/period per tick
    // define the grid
    if (init_type == "DecayCoverage")
    {
        // 10*10 grid
        for (int x=0; x < grid_size_x; ++x)
        {
            for (int y=0; y < grid_size_y; ++y)
            {
                location_t l(x, y, 0);
                grid.insert(std::pair<location_t,float>(l, 0.0f));
            }
        }
    }
    else if (init_type == "DecayBorderCoverage")
    {
        // 10*10 grid but only the borders are counted
        for (int y=0; y < grid_size_y; ++y)
        {
            location_t l(0, y, 0);
            grid.insert(std::pair<location_t,float>(l, 0.0f));

            location_t l2(grid_size_x - 1, y, 0);
            grid.insert(std::pair<location_t,float>(l2, 0.0f));
        }

        for (int x=0;x  < grid_size_x; ++x)
        {
            location_t l(x, 0, 0);
            grid.insert(std::pair<location_t,float>(l, 0.0f));

            location_t l2(x, grid_size_y - 1, 0);
            grid.insert(std::pair<location_t,float>(l2, 0.0f));
        }
    }
    else{
        throw std::runtime_error("init_type "+init_type + " is not suitable for DecayCoverageCalc");
    }
    // set accumulator to zero
    accumulator = 0.0f;
}
/* start the trial */
void DecayCoverageCalc::end_trial()
{
    // reset the grid to zeros
    for(auto it = grid.begin(); it != grid.end(); it++ )
    {
        it->second = 0.0f;
    }
    // reset accumulator to zero
    accumulator = 0.0f;
}
/* get bin corresponding to a position in the space */
location_t DecayCoverageCalc::get_bin(argos::CVector3 vec) const
{
    int binx = (int)((float)vec.GetX() / grid_step_x);
    int biny = (int)((float)vec.GetY() / grid_step_y);
    //int binz = (int)((float)vec.GetZ() / grid_step);

    location_t bin(binx, biny, 0.0f);
    // #ifdef PRINTING
    //     std::cout<<"binned "<< vec  <<"into "<<binx<<","<<biny<<","<<binz<<std::endl;
    // #endif
    return bin;
}

/* update the visited positions */
void DecayCoverageCalc::update(argos::CVector3 pos)
{
    //count the bin
    location_t bin = get_bin(pos);
    auto find_result = grid.find(bin);
    if (find_result != grid.end())
    {
        grid[bin]=1.0;
    }
}

/* at the end of the after robotloop, decay positions */
void DecayCoverageCalc::decay()
{
    for(auto it = grid.begin(); it != grid.end(); it++ )
    {
        it->second = std::max(0.0f,it->second -decay_rate);
    }
}

/* add the summed value across the grid to the acummulator */
void DecayCoverageCalc::get_grid_sum()
{
    for(auto it = grid.begin(); it != grid.end(); it++ )
    {
        accumulator += it->second;
    }
}
// /* print xy-location of an element in the visited positions */
// void DecayCoverageCalc::print_xy(location_t location) const
// {
//     std::cout<<"("<<std::get<0>(location)<<","<<std::get<1>(location)<<")"<<std::endl;
// }