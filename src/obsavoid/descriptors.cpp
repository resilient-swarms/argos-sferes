
#include <src/obsavoid/statistics.h>
#include <src/obsavoid/evol_loop_functions.h>
#include <src/obsavoid/descriptors.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <iterator>

#define SENSOR_ACTIVATION_THRESHOLD 0.5

// #ifdef TWO_D_BEHAV
// const size_t Descriptor::behav_dim = 3;
// #endif
// #ifdef THREE_D_BEHAV
// const size_t Descriptor::behav_dim = 3;
// #endif
// #ifdef SIX_D_BEHAV
// const size_t Descriptor::behav_dim = 6;
// #endif
// #ifdef FOURTYTWO_D_BEHAV
// const size_t Descriptor::behav_dim = 42;
// #endif
// #ifdef HUNDREDFIFTY_D_BEHAV
// const size_t Descriptor::behav_dim = 150;
// #endif
//const size_t Descriptor::behav_dim
// const size_t SDBC::behav_dim=3;
Descriptor::Descriptor()
{
	bd.resize(behav_dim);
}
void Descriptor::before_trials(CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
	for (size_t t = 0; t < behav_dim; ++t)
		bd[t].resize(cLoopFunctions.m_unNumberTrials, 0.0f);
	current_trial = -1; // will add +1 at start of each new trial
}

void Descriptor::start_trial()
{
	num_updates = 0;
	++current_trial;
}
std::vector<float> Descriptor::after_trials(CObsAvoidEvolLoopFunctions &cLoopFunctions)
{

	std::vector<float> final_bd;
	final_bd.resize(behav_dim);
	for (size_t i = 0; i < behav_dim; ++i)
	{
		final_bd[i] = StatFuns::mean(this->bd[i]);
		if (!StatFuns::in_range(final_bd[i], 0.0f, 1.0f))
		{
			throw std::runtime_error("bd not in [0,1]");
		}
	}
	return final_bd;
}

void AverageDescriptor::set_input_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions)
{

	for (size_t i = 0; i < cLoopFunctions.inputs.size() - 1; ++i)
	{
		this->bd[i][current_trial] += (cLoopFunctions.inputs[i] >= SENSOR_ACTIVATION_THRESHOLD) ? 1.0 : 0.0;
	}
	++num_updates;
}

/*end the trial*/
void AverageDescriptor::end_trial(CObsAvoidEvolLoopFunctions &cLoopFunctions)
{

	for (size_t i = 0; i < cLoopFunctions.inputs.size() - 1; ++i)
	{
		this->bd[i][current_trial] /= (float)num_updates;
		if (!StatFuns::in_range(this->bd[i][current_trial], 0.0f, 1.0))
		{
			throw std::runtime_error("bd not in [0,1]");
		};
	}
}

IntuitiveHistoryDescriptor::IntuitiveHistoryDescriptor(CLoopFunctions *cLoopFunctions)
{

	//define member variables
	center = cLoopFunctions->GetSpace().GetArenaCenter();

	// initialise grid (for calculating coverage and uniformity)
	argos::CVector3 max = cLoopFunctions->GetSpace().GetArenaSize();
	argos::CVector3 min = center - 0.5 * max;
	max_deviation = StatFuns::get_minkowski_distance(max, center);
	CObsAvoidEvolLoopFunctions *lf = static_cast<CObsAvoidEvolLoopFunctions *>(cLoopFunctions);
	if (lf->m_unNumberRobots != 1)
	{
		throw std::runtime_error("number of robots should be equal to 1 when choosing IntuitiveHistoryDescriptor");
	}
	coverageCalc = CoverageCalc(lf);
}

/*reset BD at the start of a trial*/
void IntuitiveHistoryDescriptor::start_trial()
{
	Descriptor::start_trial();
	deviation = 0.0f;
	//velocity_stats=RunningStat();  // dropped the velocity stats
}
void IntuitiveHistoryDescriptor::set_input_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
	//add to the deviation (to get the mean after all trials have finished)
	CVector3 pos = cLoopFunctions.get_position(cLoopFunctions.m_pcvecRobot[robot_index]);
	deviation += StatFuns::get_minkowski_distance(pos, center); // assume single robot
	coverageCalc.update(pos);
	++num_updates;
}

void IntuitiveHistoryDescriptor::set_output_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
	//nothing here
}

void IntuitiveHistoryDescriptor::end_trial(CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
	/*add behavioural metrics */

	//uniformity of probabilities
	std::vector<float> probabilities = coverageCalc.get_probs(num_updates);
	float uniformity = StatFuns::uniformity(probabilities);
	this->bd[0][current_trial] = uniformity;
	//deviation from the center
	float avg_deviation = deviation / (max_deviation * (float)num_updates);
	this->bd[1][current_trial] = avg_deviation;

#ifdef THREE_D_BEHAV
	float coverage = coverageCalc.get_coverage();
	this->bd[2][current_trial] = coverage;
#endif

#ifdef PRINTING
	std::cout << "uniformity" << uniformity << std::endl;
	std::cout << "Max deviation" << max_deviation << std::endl;
	std::cout << "deviation" << avg_deviation << std::endl;
#ifdef THREE_D_BEHAV
	std::cout << "coverage" << coverage << std::endl;
#endif
#endif

	coverageCalc.after_trial();
}

float Entity::distance(const Entity e1, const Entity e2)
{
	return StatFuns::get_minkowski_distance(e1.position, e2.position);
}

SDBC::SDBC(CLoopFunctions *cLoopFunctions, std::string init_type) : Descriptor()
{
	if (init_type.find("sdbc_all") == 0)
	{
		init_walls(cLoopFunctions);
		init_robots(cLoopFunctions);
		init_cylindric_obstacles(cLoopFunctions);
	}
	else if (init_type.find("sdbc_walls_and_robots") == 0)
	{
		init_walls(cLoopFunctions);
		init_robots(cLoopFunctions);
	}
	else if (init_type.find("sdbc_robots") == 0)
	{
		init_robots(cLoopFunctions);
	}
	else
	{
		throw std::runtime_error("init type " + init_type + "not found");
	}
	include_std = init_type.find("std") == 0 ? true : false; // will calculate standard deviations as well

	num_groups = entity_groups.size();
	for (auto &kv : entity_groups)
	{
		if (kv.second.max_size > 1 && kv.first == "robots")
		{
			within_comparison_groups.push_back(kv.first); // within-group distance computations
		}

		between_comparison_groups.push_back(kv.first); // between-group distance computations

		if (kv.second.max_size != kv.second.min_size)
		{
			variable_groups.push_back(kv.first);
		}
	}
	argos::CVector3 max = cLoopFunctions->GetSpace().GetArenaSize();
	maxX = max.GetX();
	maxY = max.GetY();
	maxdist = StatFuns::get_minkowski_distance(max, CVector3::ZERO);
}

void SDBC::init_cylindric_obstacles(CLoopFunctions *cLoopFunctions)
{
	std::vector<Entity> obstacles;
	CSpace::TMapPerType &argos_cylinders = cLoopFunctions->GetSpace().GetEntitiesByType("cylinder");
	for (CSpace::TMapPerType::iterator it = argos_cylinders.begin(); it != argos_cylinders.end(); ++it) //!TODO: Make sure the CSpace::TMapPerType does not change during a simulation (i.e it is not robot-position specific)
	{
		CCylinderEntity &cBody = *any_cast<CCylinderEntity*>(it->second);
		CVector3 position = cBody.GetEmbodiedEntity().GetOriginAnchor().Position;
		Entity e = Entity();
		e.position = CVector3(position);
		obstacles.push_back(e);
	}
	std::pair<std::string, Entity_Group> cylinderpair = {"cylinders", Entity_Group(0, obstacles.size(), obstacles.size(), obstacles)};
	entity_groups.insert(cylinderpair);
}


void SDBC::init_walls(CLoopFunctions *cLoopFunctions)
{
	// 4 walls, each with 0 state features, but two constant positional features
	std::vector<Entity> boxes;

	CSpace::TMapPerType &argos_boxes = cLoopFunctions->GetSpace().GetEntitiesByType("box");
	for (CSpace::TMapPerType::iterator it = argos_boxes.begin(); it != argos_boxes.end(); ++it) //!TODO: Make sure the CSpace::TMapPerType does not change during a simulation (i.e it is not robot-position specific)
	{
		CBoxEntity &cBody = *any_cast<CBoxEntity *>(it->second);
		if (cBody.GetId().find("wall") != 0)
		{
			// id does not name it wall
			continue;
		}
		CVector3 position = cBody.GetEmbodiedEntity().GetOriginAnchor().Position;

		Entity e = Entity();
		e.position = CVector3(position);
		boxes.push_back(e);
	}
	std::pair<std::string, Entity_Group> wallpair = {"boxes", Entity_Group(0, boxes.size(), boxes.size(), boxes)};
	entity_groups.insert(wallpair);
}
void SDBC::init_robots(CLoopFunctions *cLoopFunctions)
{
	// robot here has 5 features: x,y,orientation,wheelvelocity1,wheelvelocity2
	// here it is assumed fixed number of robots
	std::vector<Entity> robots;
	size_t num_robots = static_cast<CObsAvoidEvolLoopFunctions *>(cLoopFunctions)->m_unNumberRobots;
	for (size_t i = 0; i < num_robots; ++i)
	{
		robots.push_back(Entity());
	}
	std::pair<std::string, Entity_Group> robotpair = {"robots",
													  Entity_Group(5, num_robots, num_robots, robots)}; // 5 features: x,y,rot,w1,w2
	entity_groups.insert(robotpair);
}

/* group sizes are the first BD dimensions*/
void SDBC::add_group_sizes()
{

	for (std::string key : variable_groups)
	{
		Entity_Group &group = entity_groups[key];
		float size = group.get_size();

#ifdef PRINTING
		std::cout << "group: " << key << std::endl;
		std::cout << "size : " << size << std::endl;
#endif
		bd[bd_index][current_trial] += size; // add to average out later
		bd_index++;
	}
}

/* group mean attribute vectors, the second set of BD dimensions*/
void SDBC::add_group_meanstates()
{
	for (auto &kv : entity_groups)
	{
		Entity_Group &group = kv.second;
#ifdef PRINTING
		std::cout << "group: " << kv.first << std::endl;
#endif
		for (int i = 0; i < group.kappa; ++i)
		{

			float mean_state = group.mean_state_vec(i);

			bd[bd_index][current_trial] += mean_state;
			if (include_std)
			{
				bd_index++;
				float sd_state = group.sd_state_vec(i, mean_state);
				bd[bd_index][current_trial] += sd_state;
			}
			bd_index++;
#ifdef PRINTING
			std::cout << "attribute " << i << ": " << mean_state << std::endl;
#endif
		}
	}
}

/* avg pair-wise distance within groups, the third set of  BD dimensions*/
void SDBC::add_within_group_dispersion()
{
	float sum;
	for (std::string key : within_comparison_groups)
	{

		Entity_Group &group = entity_groups[key];
#ifdef PRINTING
		std::cout << "group: " << key << std::endl;
#endif
		if (group.max_size <= 1)
		{
			continue; //ignore
		}
		else if (group.get_absolute_size() <= 1)
		{
			// only at the moment, may change later
			++bd_index;
			continue;
		}
		else
		{
			sum = 0.0f;
			for (int i = 0; i < group.get_absolute_size(); ++i)
			{
				for (int j = 1; j < group.get_absolute_size() && j != i; ++j)
				{
					sum += Entity::distance(group[i], group[j]);
				}
			}
		}

		this->bd[bd_index][current_trial] += sum / ((float)(group.get_absolute_size() - 1) * (group.get_absolute_size() - 1) * maxdist);
		++bd_index;
	}
}

/* avg pair-wise distance between groups, the final BD dimensions*/
void SDBC::add_between_group_dispersion()
{
	float sum;
	for (size_t i = 0; i < between_comparison_groups.size(); ++i)
	{
		std::string key = between_comparison_groups[i];
		Entity_Group &group = entity_groups[key];
		for (size_t j = 1; j < between_comparison_groups.size() && j != i; ++j)
		{
			std::string key2 = between_comparison_groups[j];
			Entity_Group &group2 = entity_groups[key2];
#ifdef PRINTING
			std::cout << "group1: " << key << std::endl;
			std::cout << "group2: " << key2 << std::endl;
#endif
			if (group.max_size <= 1)
			{
				continue; //ignore
			}
			else if (group.get_absolute_size() <= 1)
			{
				++bd_index;
				continue;
			}
			else
			{
				sum = 0.0f;
				for (Entity e1 : group.entities)
				{
					for (Entity e2 : group2.entities)
					{
						sum += Entity::distance(e1, e2);
					}
				}
			}
			bd[bd_index][current_trial] += sum / ((float)(group.get_absolute_size() * group2.get_absolute_size()) * maxdist); // divide by product of group sizes; add for now, we will average the number of times
			++bd_index;
		}
	}
}

/* prepare for trials*/
void SDBC::before_trials(argos::CSimulator &cSimulator)
{
}
/*reset BD at the start of a trial*/
void SDBC::start_trial()
{
	Descriptor::start_trial();
	bd_index = 0;
}
/*after getting inputs, can update the descriptor if needed*/
void SDBC::set_input_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
}
/*after getting outputs, can update the descriptor if needed*/
void SDBC::set_output_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
	// here just set the attributes of robot at index; let end
	CVector3 pos = cLoopFunctions.get_position(cLoopFunctions.m_pcvecRobot[robot_index]);

	float x = pos.GetX() / maxX;
	float y = pos.GetY() / maxY;
	float theta = cLoopFunctions.curr_theta.UnsignedNormalize() / CRadians::TWO_PI; // normalise radians to [0,1]
	float wheel1 = (10.0f + cLoopFunctions.outf[0]) / 20.0f;
	float wheel2 = (10.0f + cLoopFunctions.outf[1]) / 20.0f;

	std::vector<float> new_vec = {x, y, theta, wheel1, wheel2};
	entity_groups["robots"][robot_index].set_attributes(new_vec, pos);
#ifdef PRINTING
	std::cout << "x,y,theta,w1,w2=" << x << "," << y << "," << theta << "," << wheel1 << "," << wheel2 << std::endl;
#endif
}
/*after the looping over robots*/
void SDBC::after_robotloop(CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
	add_group_sizes();
	add_group_meanstates();
	add_between_group_dispersion();
	add_within_group_dispersion();
	bd_index = 0;
	++num_updates;
}
/*end the trial*/
void SDBC::end_trial(CObsAvoidEvolLoopFunctions &cLoopFunctions)
{

	for (size_t i = 0; i < behav_dim; ++i)
	{
		this->bd[i][current_trial] /= (float)(num_updates + 1);
		if (!StatFuns::in_range(this->bd[i][current_trial], 0.0f, 1.0f))
		{
			throw std::runtime_error("bd" + std::to_string(i) + " not in [0,1]: " + std::to_string(bd[i][current_trial]));
		};
	}
}



CVT_MutualInfo::CVT_MutualInfo()
{
	freqs.resize(num_sensors);
	joint_freqs.resize(num_sensors);
	for (size_t i = 0; i < num_sensors; ++i)
	{
		joint_freqs[i].resize(num_sensors);
	}
}

/* prepare for trials*/
void CVT_MutualInfo::before_trials(CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
	num_updates = 0; //start counting all the updates in all the trials
	/* reset frequencies */

	for (size_t i = 0; i < num_sensors; ++i)
	{
		freqs[i]=std::vector<float>(num_bins,0.0f);;
		for (size_t j = 0; j < num_sensors; ++j)
		{
			joint_freqs[i][j]=std::vector<float>(num_bins*num_bins,0.0f);
		}
	}
}
/*reset BD at the start of a trial*/
void CVT_MutualInfo::start_trial()
{
	// here we don't do anything because  we don't need to reset the number of updates or keep increment the trials
}
// /* get bin for sensory probabilities  */
// size_t CVT_MutualInfo::get_sensory_bin(float activation) const
// {
// 	return StatFuns::get_bin(activation,0.0f,1.0f,num_bins);
// }
/*after getting inputs, can update the descriptor if needed*/
void CVT_MutualInfo::set_input_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
	size_t joint_index = 0;
	// frequency + joint_frequency
	for (size_t i = 0; i < cLoopFunctions.inputs.size() - 1; ++i)
	{
		size_t bin = cLoopFunctions.get_sensory_bin(i, num_bins);

		++freqs[i][bin];
		for (size_t j = 0; j < cLoopFunctions.inputs.size() - 1; ++j)
		{
			if (j == i)
			{
				continue;
			}
			size_t bin2 = cLoopFunctions.get_sensory_bin(j, num_bins);
			size_t joint_bin = bin * num_bins + bin2;
			++joint_freqs[i][j][joint_bin];
		}
	}
	num_updates++;
}

/*after the looping over robots*/
void CVT_MutualInfo::after_robotloop(CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
}

/*end the trial*/
void CVT_MutualInfo::end_trial(CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
}

/*summarise BD at the end of trials*/
std::vector<float> CVT_MutualInfo::after_trials(CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
	/* convert frequencies to probabilities */

	normalise();
	std::vector<float> final_bd = get_bd();
	return final_bd;
}
/* normalise frequencies */
void CVT_MutualInfo::normalise()
{
	for (size_t i = 0; i < num_sensors; ++i)
	{
		StatFuns::normalise(freqs[i], num_updates);

		for (size_t j = 0; j < num_sensors; ++j)
		{
			if (j == i)
			{
				continue;
			}
			StatFuns::normalise(joint_freqs[i][j], num_updates);
		}
	}
}
/* calculate entropies */
std::vector<float> CVT_MutualInfo::get_bd()
{
	std::vector<float> final_bd;
	/* calculate MI */
	for (size_t i = 0; i < num_sensors; ++i)
	{
		for (size_t j = 0; j < num_sensors; ++j)
		{
			if (j == i)
			{
				continue;
			}
			float MI = calc_and_check(i, j);
			final_bd.push_back(MI);
		}
	}
	return final_bd;
}
float CVT_MutualInfo::calc_and_check(size_t i, size_t j)
{
	float mi = StatFuns::mutual_information(joint_freqs[i][j], freqs[i], freqs[j], num_updates);
	float MI = mi / StatFuns::max_entropy(num_bins, EULER);
#ifdef PRINTING
	printf("\n MI_{%zu,%zu} = %f", i, j, MI);
#endif
	if (!StatFuns::in_range(MI, 0.0f, 1.0f))
	{
		throw std::runtime_error("normalised MI should be in [0,1]");
	}
	return MI;
}







CVT_MutualInfoAct::CVT_MutualInfoAct()
{
	freqs.resize(num_sensors);
	act_freqs.resize(num_act);
	joint_freqs.resize(num_sensors);
	for (size_t i = 0; i < num_sensors; ++i)
	{
		joint_freqs[i].resize(num_act);
	}
}


/* prepare for trials*/
void CVT_MutualInfoAct::before_trials(CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
	num_updates = 0; //start counting all the updates in all the trials
	/* reset frequencies */

	for (size_t j = 0; j < num_act; ++j)
	{
		act_freqs[j]=std::vector<float>(num_bins,0.0f);
	}
	for (size_t i = 0; i < num_sensors; ++i)
	{
		freqs[i]=std::vector<float>(num_bins, 0.0f);

		for (size_t j = 0; j < num_act; ++j)
		{
			joint_freqs[i][j]=std::vector<float>(num_bins * num_bins, 0.0f);
		}
	}
}


/* calculate entropies */
float CVT_MutualInfoAct::calc_and_check(size_t i, size_t j)
{
	float mi = StatFuns::mutual_information(joint_freqs[i][j], freqs[i], act_freqs[j], num_updates);
	float MI = mi / StatFuns::max_entropy(num_bins, EULER);
#ifdef PRINTING
	printf("\n MI_{%zu,%zu} = %f", i, j, MI);
#endif
	if (!StatFuns::in_range(MI, 0.0f, 1.0f))
	{
		throw std::runtime_error("normalised MI should be in [0,1]");
	}
	return MI;
}

/* calculate entropies */
std::vector<float> CVT_MutualInfoAct::get_bd()
{
	std::vector<float> final_bd;
	/* calculate MI */
	for (size_t i = 0; i < num_sensors; ++i)
	{
		for (size_t j = 0; j < num_act; ++j)
		{
			float MI = calc_and_check(i, j);
			final_bd.push_back(MI);
		}
	}
	return final_bd;
}

/* normalise frequencies */
void CVT_MutualInfoAct::normalise()
{
	for (size_t k = 0; k < num_act; ++k)
	{
		StatFuns::normalise(act_freqs[k], num_updates);
	}
	for (size_t i = 0; i < num_sensors; ++i)
	{
		StatFuns::normalise(freqs[i], num_updates);

		for (size_t j = 0; j < num_act; ++j)
		{
			StatFuns::normalise(joint_freqs[i][j], num_updates);
		}
	}
}
/*after getting outputs, can update the descriptor if needed*/
void CVT_MutualInfoAct::set_output_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
	for (size_t j = 0; j < cLoopFunctions.outf.size(); ++j)
	{
		size_t bin2 = cLoopFunctions.get_actuator_bin(j, num_bins);
		++act_freqs[j][bin2];
	}
	// frequency + joint_frequency
	for (size_t i = 0; i < cLoopFunctions.inputs.size() - 1; ++i)
	{
		size_t bin = cLoopFunctions.get_sensory_bin(i, num_bins);
		++freqs[i][bin];
		for (size_t j = 0; j < cLoopFunctions.outf.size(); ++j)
		{
			size_t bin2 = cLoopFunctions.get_actuator_bin(j, num_bins);
			size_t joint_bin = bin * num_bins + bin2;
			++joint_freqs[i][j][joint_bin];
		}
	}
	num_updates++;
}



CVT_Spirit::CVT_Spirit()
{
	freqs.resize(num_joint_sensory_bins);
}



/* prepare for trials*/
void CVT_Spirit::before_trials(CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
	freqs.resize(num_joint_sensory_bins);
	for (size_t i = 0; i < num_joint_sensory_bins; ++i)
	{

		freqs[i]=std::vector<float>(num_joint_actuator_bins,0.0f);
		
	}
}
/*reset BD at the start of a trial*/
void CVT_Spirit::start_trial()
{
	// here we don't do anything because  we don't need to reset the number of updates or keep increment the trials
}

/*after the looping over robots*/
void CVT_Spirit::after_robotloop(CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
}

/*end the trial*/
void CVT_Spirit::end_trial(CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
}

/* normalise frequencies */
std::vector<float> CVT_Spirit::get_bd()
{
	std::vector<float> final_bd;
	for (size_t i = 0; i < num_joint_sensory_bins; ++i)
	{
		float total_observations = StatFuns::sum(freqs[i]);
		// use laplace smoothing instead of treating total_observations > 0 radically different than total_observations=0
		for (int j = 0; j < num_joint_actuator_bins; ++j)
		{
			final_bd.push_back(StatFuns::laplace_smoothing(freqs[i][j], total_observations, alpha_smooth, num_joint_actuator_bins));
		}
	}
	return final_bd;
}

/*after getting outputs, can update the descriptor if needed*/
void CVT_Spirit::set_output_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
	size_t sens_bin = cLoopFunctions.get_quadrant_bin();
	size_t act_bin = cLoopFunctions.get_joint_actuator_bin(num_actuator_bins);
	++freqs[sens_bin][act_bin];
}

/*summarise BD at the end of trials*/
std::vector<float> CVT_Spirit::after_trials(CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
	std::vector<float> final_bd = get_bd();
	return final_bd;
}

// NonMarkovianStochasticPolicyInduction::NonMarkovianStochasticPolicyInduction()
// {
// 	bd.resize(behav_dim);
// }

void CVT_Trajectory::before_trials(CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
	final_bd.clear();
}
CVT_Trajectory::CVT_Trajectory(CObsAvoidEvolLoopFunctions &cLoopFunctions, size_t num_steps)
{
	num_chunks = behav_dim / (2 * cLoopFunctions.m_unNumberTrials);

	periodicity = num_steps / (num_chunks);
	argos::CVector3 max = cLoopFunctions.GetSpace().GetArenaSize();
	maxX = max.GetX();
	maxY = max.GetY();
}
/*after getting outputs, can update the descriptor if needed*/
void CVT_Trajectory::set_output_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
	num_updates++;
	if (end_chunk())
	{
		// here just set the attributes of robot at index; let end
		CVector3 pos = cLoopFunctions.get_position(cLoopFunctions.m_pcvecRobot[robot_index]);
		float x = pos.GetX() / maxX;
		float y = pos.GetY() / maxY;
		final_bd.push_back(x);
		final_bd.push_back(y);
	}
}
/*summarise BD at the end of trials*/
std::vector<float> CVT_Trajectory::after_trials(CObsAvoidEvolLoopFunctions &cLoopFunctions)
{
	return final_bd;
}
