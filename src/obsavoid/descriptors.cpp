

#include <src/obsavoid/evol_loop_functions.h>
#include <src/obsavoid/descriptors.h>
#include <src/obsavoid/statistics.h>
#include <iterator>

#define SENSOR_ACTIVATION_THRESHOLD 0.5


const size_t Descriptor::behav_dim=ParamsDnn::dnn::nb_inputs - 1;

void AverageDescriptor::set_input_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions& cLoopFunctions)
{   

    for(size_t i = 0; i < cLoopFunctions.inputs.size()-1; ++i )
    {
        this->bd[i] += (cLoopFunctions.inputs[i] >= SENSOR_ACTIVATION_THRESHOLD) ? 1.0 : 0.0;
    }

}
std::vector<float> AverageDescriptor::after_trials(Real time,CObsAvoidEvolLoopFunctions& cLoopFunctions)
{
          // BD1 -- characterizes the number of times the robot turns.
        //data.push_back(cLoopFunctions.num_ds / (Real)cSimulator.GetMaxSimulationClock());
        //assert(cLoopFunctions.num_ds / (Real)cSimulator.GetMaxSimulationClock() >= 0.0 && cLoopFunctions.num_ds / (Real)cSimulator.GetMaxSimulationClock() <= 1.0);

        // BD1 - BD7 -- characterizes the number of times the different IR proximity sensors on the robot return a high value
        for(size_t i = 0; i < this->bd.size(); ++i)
            this->bd[i] /= time*(float)cLoopFunctions.m_unNumberTrials;
        return this->bd;
}




IntuitiveHistoryDescriptor::IntuitiveHistoryDescriptor(CLoopFunctions* cLoopFunctions)
{
            bd.resize(behav_dim,0.0f);
            //define member variables
            center = cLoopFunctions->GetSpace().GetArenaCenter();
            
            // initialise grid (for calculating coverage and uniformity)
            argos::CVector3 max =cLoopFunctions->GetSpace().GetArenaSize();
            argos::CVector3 min = center - 0.5*max;
            max_deviation = StatFuns::get_minkowski_distance(max,center);

            assert(m_unNumberRobots==1 && "number of robots should be equal to 1 when choosing IntuitiveHistoryDescriptor");
            total_size = max.GetX()*max.GetY()/grid_step;

}
/* prepare for trials*/
void IntuitiveHistoryDescriptor::before_trials(argos::CSimulator& cSimulator){



}
/*reset BD at the start of a trial*/
void IntuitiveHistoryDescriptor::start_trial()
{
    deviation=0.0;
    visitation_count=0;
    //velocity_stats=RunningStat();  // dropped the velocity stats
}
void IntuitiveHistoryDescriptor::set_input_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions& cLoopFunctions)
{   
    //add to the deviation (to get the mean after all trials have finished)
    CVector3 pos = cLoopFunctions.get_position(cLoopFunctions.m_pcvecRobot[robot_index]);
    deviation+=StatFuns::get_minkowski_distance(pos,center);// assume single robot 

    //count the bin
    std::tuple<int, int, int> bin = get_bin(pos);
    auto find_result =  unique_visited_positions.find(bin);
    if (find_result ==  unique_visited_positions.end())
    {
        unique_visited_positions.insert(std::pair<std::tuple<int, int, int>,size_t>(bin,1));
    }
    else
    {
        unique_visited_positions[bin]+=1;
    }
    visitation_count+=1;



}

void IntuitiveHistoryDescriptor::set_output_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions& cLoopFunctions)
{
    //velocity_stats->push(cLoopFunctions.curr_lin_speed);
}

void IntuitiveHistoryDescriptor::end_trial(CObsAvoidEvolLoopFunctions& cLoopFunctions)
{
    /*add behavioural metrics */

    //uniformity of probabilities
    std::vector<float> probabilities = get_probs();
    float uniformity = StatFuns::uniformity(probabilities);
    this->bd[0] += uniformity;
    //deviation from the center
    float avg_deviation = deviation/(max_deviation*(float)visitation_count);
    this->bd[1] += avg_deviation;
    //coverage
    float max_visited_positions=std::min(total_size,(float)visitation_count);
    float coverage = (float) unique_visited_positions.size()/ (float)max_visited_positions;
    this->bd[2] +=coverage;
    // //variability in the speed]
    // float velocity_sd=velocity_stats.std()/max_velocitysd;
    // this->bd[3] +=velocity_sd;
    //

    
    #ifdef PRINTING
        std::cout<<"uniformity"<<uniformity<<std::endl;
        std::cout<<"Max deviation"<<max_deviation<<std::endl;
        std::cout<<"deviation"<<avg_deviation<<std::endl;
        std::cout<<"coverage"<<coverage<<std::endl;
        //std::cout<<"velocity_sd"<<velocity_sd<<std::endl;
    #endif

    unique_visited_positions.clear();

}
std::vector<float> IntuitiveHistoryDescriptor::after_trials(Real time, CObsAvoidEvolLoopFunctions& cLoopFunctions)
{
    // now normalise all the summed bds
    for (int i=0; i < bd.size(); ++i)
    {
        bd[i]/=cLoopFunctions.m_unNumberTrials;
    }
    return bd;

}





SDBC::SDBC(CLoopFunctions* cLoopFunctions, std::string init_type) : Descriptor()
{
	if(init_type=="sdbc_walls_and_robots")
	{
		init_walls_and_robots(cLoopFunctions);
	}
	else if ("sdbc_robots")
	{
		init_walls_and_robots(cLoopFunctions);
	}
	else{
		throw std::runtime_error("init type "+init_type+ "not found");
	}

	num_groups=entity_groups.size();
}
void SDBC::init_walls_and_robots(CLoopFunctions* cLoopFunctions)
{
	SDBC::init_robots(cLoopFunctions);
    // 4 walls, each with 0 state features, but two constant positional features
	std::vector<Entity> walls;
	for (size_t i=0; i < 4; ++i)
	{
		walls.push_back(Entity());
	}
	std::pair<std::string,Entity_Group> wallpair={"walls" ,Entity_Group(0, 4, 0, walls)};
    entity_groups.insert(wallpair);
    
}
void SDBC::init_robots(CLoopFunctions* cLoopFunctions)
{
	bd.resize(behav_dim,0.0f);
	// robot here has 4 features: x,y,wheelvelocity1,wheelvelocity2
	std::vector<Entity> robots;
	size_t num_robots=static_cast<CObsAvoidEvolLoopFunctions*>(cLoopFunctions)->m_unNumberRobots;
	for (size_t i=0; i < num_robots; ++i)
	{
		robots.push_back(Entity());
	}
	std::pair<std::string,Entity_Group> robotpair={"robots" ,
    									Entity_Group(4, num_robots, 0, robots)};
    entity_groups.insert(robotpair);
}
float SDBC::distance_function(argos::CVector3 e1, argos::CVector3 e2)
{
 	return StatFuns::get_minkowski_distance(e1,e2);
}


/* group sizes are the first BD dimensions*/
void SDBC::add_group_sizes()
{
	for (auto& kv : entity_groups)
	{
		Entity_Group& group = kv.second;
		float size = group.get_size();

		#ifdef PRINTING
			std::cout<<"group: "<<kv.first<<std::endl;
			std::cout<<"size : "<<size<<std::endl;
		#endif
		bd[bd_index]+=size;
		bd_index++;
	}
}

/* group mean attribute vectors, the second set of BD dimensions*/
void SDBC::add_group_meanstates()
{
	for (auto& kv : entity_groups)
	{
		Entity_Group& group = kv.second;
		#ifdef PRINTING
			std::cout<<"group: "<<kv.first<<std::endl;
		#endif
		for (int i=0; i < group.kappa; ++i) 
		{

			float mean_state=group.mean_state_vec(i);

			bd[bd_index]+=mean_state;
			bd_index++;
			#ifdef PRINTING
				std::cout<<"attribute "<<i<<": "<<mean_state<<std::endl;
			#endif

		}
	}
}

/* avg pair-wise distance within groups, the third set of  BD dimensions*/
void SDBC::add_within_group_dispersion()
{
	for (auto& kv : entity_groups)
	{
		Entity_Group& group=kv.second;
		#ifdef PRINTING
			std::cout<<"group: "<<kv.first<<std::endl;
		#endif
		if(group.max_size<=1)
		{
			continue;//ignore
		}
		else if (group.get_absolute_size()<= 1)
		{
			bd[bd_index]+=0.0f;// 0 at the moment
			++bd_index;
			continue;
		}
		else
		{
			for (int i=0; i < group.kappa; ++i) 
			{

			float mean_state=group.mean_state_vec(i);

			bd[bd_index]+=mean_state;
			++bd_index;
			#ifdef PRINTING
				std::cout<<"attribute "<<i<<": "<<mean_state<<std::endl;
			#endif
			}
		}


	}
}

/* avg pair-wise distance between groups, the final BD dimensions*/
void SDBC::add_between_group_dispersion(){

}


/* prepare for trials*/
void SDBC::before_trials(argos::CSimulator& cSimulator){
}
/*reset BD at the start of a trial*/
void SDBC::start_trial()
{
	bd_index=0;
}
/*after getting inputs, can update the descriptor if needed*/
void SDBC::set_input_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions& cLoopFunctions)
{

}
/*after getting outputs, can update the descriptor if needed*/
void SDBC::set_output_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions& cLoopFunctions)
{
	 // here just set the attributes of the entities; since walls do not have attributes, only robots is enough for now
	 CVector3 pos = cLoopFunctions.get_position(cLoopFunctions.m_pcvecRobot[robot_index]);
	 argos::CVector3 max =cLoopFunctions.GetSpace().GetArenaSize();
	 float x = pos.GetX()/max.GetX();
	 float y = pos.GetY()/max.GetY();
	 std::vector<float> new_vec = {x,y,(10.0f+cLoopFunctions.outf[0])/20.0f,(10.0f+cLoopFunctions.outf[1])/20.0f};
	 element_wise_addition<float>(entity_groups["robots"][robot_index].attributes,new_vec);
     


}
/*end the trial*/
void SDBC::end_trial(CObsAvoidEvolLoopFunctions& cLoopFunctions)
{

}
/*summarise BD at the end of trials*/
std::vector<float> SDBC::after_trials(Real time, CObsAvoidEvolLoopFunctions& cLoopFunctions)
{
	// now normalise all the summed bds
    for (int i=0; i < bd.size(); ++i)
    {
        bd[i]/=cLoopFunctions.m_unNumberTrials;
    }
    return bd;
}
