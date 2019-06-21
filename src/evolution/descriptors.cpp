
#include <src/core/statistics.h>
#include <src/evolution/evol_loop_functions.h>
#include <src/evolution/descriptors.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <iterator>
#include <random>
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
	geometric_median = false;
	bd.resize(behav_dim);
}
void Descriptor::before_trials(EvolutionLoopFunctions &cLoopFunctions)
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
std::vector<float> Descriptor::after_trials(EvolutionLoopFunctions &cLoopFunctions)
{

	std::vector<float> final_bd;
	final_bd.resize(behav_dim);
	if (geometric_median)
	{
		final_bd = StatFuns::geometric_median(StatFuns::transpose<float>(this->bd));
	}
	else
	{
		for (size_t i = 0; i < behav_dim; ++i)
		{
			final_bd[i] = StatFuns::mean(this->bd[i]);

			if (!StatFuns::in_range(final_bd[i], 0.0f, 1.0f))
			{
				throw std::runtime_error("bd not in [0,1]");
			}
		}
	}

	return final_bd;
}

void AverageDescriptor::set_input_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions)
{

	for (size_t i = 0; i < cLoopFunctions.inputs.size() - 1; ++i)
	{
		this->bd[i][current_trial] += (cLoopFunctions.inputs[i] >= SENSOR_ACTIVATION_THRESHOLD) ? 1.0 : 0.0;
	}
	++num_updates;
}

/*end the trial*/
void AverageDescriptor::end_trial(EvolutionLoopFunctions &cLoopFunctions)
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
	EvolutionLoopFunctions *lf = static_cast<EvolutionLoopFunctions *>(cLoopFunctions);
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
void IntuitiveHistoryDescriptor::set_input_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions)
{
	//add to the deviation (to get the mean after all trials have finished)
	CVector3 pos = cLoopFunctions.get_position(cLoopFunctions.m_pcvecRobot[robot_index]);
	deviation += StatFuns::get_minkowski_distance(pos, center); // assume single robot
	coverageCalc.update(pos);
	++num_updates;
}

void IntuitiveHistoryDescriptor::set_output_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions)
{
	//nothing here
}

void IntuitiveHistoryDescriptor::end_trial(EvolutionLoopFunctions &cLoopFunctions)
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
	size_t num_attr;

	if (init_type.find("Gomes") != std::string::npos)
	{
		include_closest_robot = true; // add closest robot distance
		attribute_setter = new SpeedAttributeSetter(cLoopFunctions);
		num_attr = 2;			 // linear speed and turn speed
		geometric_median = true; //apply geometric median instead of mean
	}
	else
	{
		include_closest_robot = false;
		attribute_setter = new NormalAttributeSetter(cLoopFunctions);
		num_attr = 5; // x,y,theta,w1,w2
	}

	if (init_type.find("sdbc_all") != std::string::npos)
	{
		init_walls(cLoopFunctions);
		init_robots(num_attr, cLoopFunctions);
		init_cylindric_obstacles(cLoopFunctions);
	}
	else if (init_type.find("sdbc_walls_and_robots") != std::string::npos)
	{
		init_walls(cLoopFunctions);
		init_robots(num_attr, cLoopFunctions);
	}
	else if (init_type.find("sdbc_robots") != std::string::npos)
	{
		init_robots(num_attr, cLoopFunctions);
	}
	else
	{
		throw std::runtime_error("init type " + init_type + "not found");
	}

	include_std = init_type.find("std") != std::string::npos ? true : false; // will calculate standard deviations as well]

	if (include_std)
	{
		num_features = behav_dim / 2;
		temp_bd.resize(num_features);
	}
	else{
		temp_bd.resize(behav_dim);
	}
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

	maxdist = StatFuns::get_minkowski_distance(CVector3(attribute_setter->maxX, attribute_setter->maxY, 0), CVector3::ZERO);
}

void SDBC::init_cylindric_obstacles(CLoopFunctions *cLoopFunctions)
{
	std::vector<Entity> obstacles;
	CSpace::TMapPerType &argos_cylinders = cLoopFunctions->GetSpace().GetEntitiesByType("cylinder");
	for (CSpace::TMapPerType::iterator it = argos_cylinders.begin(); it != argos_cylinders.end(); ++it) //!TODO: Make sure the CSpace::TMapPerType does not change during a simulation (i.e it is not robot-position specific)
	{
		CCylinderEntity &cBody = *any_cast<CCylinderEntity *>(it->second);
		CVector3 position = cBody.GetEmbodiedEntity().GetOriginAnchor().Position;
		Entity e = Entity();
		e.position = CVector3(position);
		obstacles.push_back(e);
	}
	bool is_static = false;
	std::pair<std::string, Entity_Group> cylinderpair = {"cylinders",
														 Entity_Group(0, obstacles.size(), obstacles.size(), obstacles, is_static)};
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
	bool is_static = true;
	std::pair<std::string, Entity_Group> wallpair = {"boxes", Entity_Group(0, boxes.size(), boxes.size(), boxes, is_static)};
	entity_groups.insert(wallpair);
}
void SDBC::init_robots(size_t num_features, CLoopFunctions *cLoopFunctions)
{
	// robot here has 5 features: x,y,orientation,wheelvelocity1,wheelvelocity2
	// here it is assumed fixed number of robots
	std::vector<Entity> robots;
	size_t num_robots = static_cast<EvolutionLoopFunctions *>(cLoopFunctions)->m_unNumberRobots;
	for (size_t i = 0; i < num_robots; ++i)
	{
		robots.push_back(Entity());
	}
	bool is_static = false;
	std::pair<std::string, Entity_Group> robotpair = {"robots",
													  Entity_Group(num_features, num_robots, num_robots, robots, is_static)}; // 5 features: x,y,rot,w1,w2
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
		temp_bd[bd_index].push_back(size); // add to average out later
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

			temp_bd[bd_index].push_back(mean_state);
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

		sum = sum / ((float)(group.get_absolute_size() - 1) * (group.get_absolute_size() - 1) * maxdist);
		this->temp_bd[bd_index].push_back(sum);
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
		for (size_t j = i + 1; j < between_comparison_groups.size(); ++j)
		{
			std::string key2 = between_comparison_groups[j];
			Entity_Group &group2 = entity_groups[key2];
#ifdef PRINTING
			std::cout << "group1: " << key << std::endl;
			std::cout << "group2: " << key2 << std::endl;
#endif
			if (group.is_static && group2.is_static)
			{
				continue; //ignore always as the distance will remain constant
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
			sum = sum / ((float)(group.get_absolute_size() * group2.get_absolute_size()) * maxdist); // divide by product of group sizes; add for now, we will average the number of times
			temp_bd[bd_index].push_back(sum);
			++bd_index;
		}
	}
}

/* distance to closest robot */
void SDBC::add_closest_robot_dist(EvolutionLoopFunctions &cLoopFunctions)
{
	float mean_dist = 0.0f;
	// we already have positions available in curr_pos
	for (size_t i = 0; i < cLoopFunctions.curr_pos.size(); ++i)
	{
		float mindist = std::numeric_limits<float>::infinity();
		for (size_t j = 0; j < cLoopFunctions.curr_pos.size(); ++j)
		{
			if (j == i)
			{
				continue;
			}
			// get the distance
			float dist = StatFuns::get_minkowski_distance(cLoopFunctions.curr_pos[i], cLoopFunctions.curr_pos[j]);
			if (dist < mindist)
			{
				mindist = dist;
			}
		}
		mean_dist += mindist;
	}
	mean_dist = mean_dist / (cLoopFunctions.curr_pos.size() * maxdist);
	temp_bd[bd_index].push_back(mean_dist);
}

/* prepare for trials*/
void SDBC::before_trials(EvolutionLoopFunctions &cLoopFunctions)
{
	for (size_t t = 0; t < num_features; ++t)
		temp_bd[t].clear();
	Descriptor::before_trials(cLoopFunctions);
	//current_trial = -1; // will add +1 at start of each new trial
}
/*reset BD at the start of a trial*/
void SDBC::start_trial()
{
	Descriptor::start_trial();
	bd_index = 0;
}
/*after getting inputs, can update the descriptor if needed*/
void SDBC::set_input_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions)
{
}
/*after getting outputs, can update the descriptor if needed*/
void SDBC::set_output_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions)
{
	entity_groups["robots"][robot_index].set_attributes(attribute_setter->get_attributes(robot_index, cLoopFunctions), cLoopFunctions.curr_pos[robot_index]);
}
/*after the looping over robots*/
void SDBC::after_robotloop(EvolutionLoopFunctions &cLoopFunctions)
{
	add_group_sizes();
	add_group_meanstates();
	add_between_group_dispersion();
	add_within_group_dispersion();
	if (include_closest_robot)
	{
		add_closest_robot_dist(cLoopFunctions);
	}
	bd_index = 0;
	++num_updates;
}
/*end the trial*/
// void SDBC::end_trial(EvolutionLoopFunctions &cLoopFunctions)
// {

// 	for (size_t i = 0; i < behav_dim; ++i)
// 	{
// 		this->bd[i][current_trial] /= (float)(num_updates + 1);
// 		if (!StatFuns::in_range(this->bd[i][current_trial], 0.0f, 1.0f))
// 		{
// 			throw std::runtime_error("bd" + std::to_string(i) + " not in [0,1]: " + std::to_string(bd[i][current_trial]));
// 		};
// 	}
// }

/*summarise BD at the end of trials*/
void SDBC::end_trial(EvolutionLoopFunctions &cLoopFunctions)
{
	//After a simulation has ended, the samples obtained for each
	//behaviour feature at each time step are aggregated to assem-ble a fixed-length characterisation vector.
	for (size_t i = 0; i < behav_dim; ++i)
	{
		if (include_std)
		{
			if (i % 2 == 0)
			{
				bd[i][current_trial]=StatFuns::mean(this->temp_bd[i/2]);
			}
			else
			{
				bd[i][current_trial] = StatFuns::standard_dev(this->temp_bd[i / 2]);
			}
		}
		else
		{
			bd[i][current_trial] = StatFuns::mean(this->temp_bd[i]);
		}

		if (!StatFuns::in_range(bd[i][current_trial], 0.0f, 1.0f))
		{
			throw std::runtime_error("bd" + std::to_string(i) + " not in [0,1]: " + std::to_string(bd[i][current_trial]));
		};
	}
}

CVT_MutualInfo::CVT_MutualInfo()
{
	freqs.resize(num_sensors);
	joint_freqs.resize(num_sensors);
	for (size_t i = 0; i < num_sensors - 1; ++i)
	{
		joint_freqs[i].resize(num_sensors - i - 1);
	}
}

/* prepare for trials*/
void CVT_MutualInfo::before_trials(EvolutionLoopFunctions &cLoopFunctions)
{
	num_updates = 0; //start counting all the updates in all the trials
	/* reset frequencies */

	for (size_t i = 0; i < num_sensors; ++i)
	{
		freqs[i] = std::vector<float>(num_bins, 0.0f);
		;
		for (size_t j = i + 1; j < num_sensors; ++j)
		{
			joint_freqs[i][j - i - 1] = std::vector<float>(num_bins * num_bins, 0.0f);
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
void CVT_MutualInfo::set_input_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions)
{
	size_t joint_index = 0;
	// frequency + joint_frequency
	for (size_t i = 0; i < num_sensors; ++i)
	{
		size_t bin = cLoopFunctions.get_sensory_bin(i, num_bins);

		++freqs[i][bin];
		for (size_t j = i + 1; j < num_sensors; ++j)
		{
			size_t bin2 = cLoopFunctions.get_sensory_bin(j, num_bins);
			size_t joint_bin = bin * num_bins + bin2;
			++joint_freqs[i][j - i - 1][joint_bin];
		}
	}
	num_updates++;
}

/*after the looping over robots*/
void CVT_MutualInfo::after_robotloop(EvolutionLoopFunctions &cLoopFunctions)
{
}

/*end the trial*/
void CVT_MutualInfo::end_trial(EvolutionLoopFunctions &cLoopFunctions)
{
}

/*summarise BD at the end of trials*/
std::vector<float> CVT_MutualInfo::after_trials(EvolutionLoopFunctions &cLoopFunctions)
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

		for (size_t j = i + 1; j < num_sensors; ++j)
		{
			StatFuns::normalise(joint_freqs[i][j - i - 1], num_updates);
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
		for (size_t j = i + 1; j < num_sensors; ++j)
		{
			float MI = calc_and_check(i, j);
			final_bd.push_back(MI);
		}
	}
	return final_bd;
}
float CVT_MutualInfo::calc_and_check(size_t i, size_t j)
{
	float mi = StatFuns::mutual_information(joint_freqs[i][j - i - 1], freqs[i], freqs[j], num_updates);
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
void CVT_MutualInfoAct::before_trials(EvolutionLoopFunctions &cLoopFunctions)
{
	num_updates = 0; //start counting all the updates in all the trials
	/* reset frequencies */

	for (size_t j = 0; j < num_act; ++j)
	{
		act_freqs[j] = std::vector<float>(num_act_bins, 0.0f);
	}
	for (size_t i = 0; i < num_sensors; ++i)
	{
		freqs[i] = std::vector<float>(num_bins, 0.0f);

		for (size_t j = 0; j < num_act; ++j)
		{
			joint_freqs[i][j] = std::vector<float>(num_bins * num_act_bins, 0.0f);
		}
	}
}

/* calculate entropies */
float CVT_MutualInfoAct::calc_and_check(size_t i, size_t j)
{
	float mi = StatFuns::mutual_information(joint_freqs[i][j], freqs[i], act_freqs[j], num_updates);
	float MI = mi / StatFuns::max_entropy(std::max(num_bins, num_act_bins), EULER);
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
void CVT_MutualInfoAct::set_output_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions)
{
	for (size_t j = 0; j < num_act; ++j)
	{
		size_t bin2 = cLoopFunctions.get_actuator_bin(j, num_act_bins);
		++act_freqs[j][bin2];
	}
	// frequency + joint_frequency
	for (size_t i = 0; i < num_sensors; ++i)
	{
		size_t bin = cLoopFunctions.get_sensory_bin(i, num_bins);
		++freqs[i][bin];
		for (size_t j = 0; j < num_act; ++j)
		{
			size_t bin2 = cLoopFunctions.get_actuator_bin(j, num_act_bins);
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
void CVT_Spirit::before_trials(EvolutionLoopFunctions &cLoopFunctions)
{
	freqs.resize(num_joint_sensory_bins);
	for (size_t i = 0; i < num_joint_sensory_bins; ++i)
	{

		freqs[i] = std::vector<float>(num_joint_actuator_bins, 0.0f);
	}
}
/*reset BD at the start of a trial*/
void CVT_Spirit::start_trial()
{
	// here we don't do anything because  we don't need to reset the number of updates or keep increment the trials
}

/*after the looping over robots*/
void CVT_Spirit::after_robotloop(EvolutionLoopFunctions &cLoopFunctions)
{
}

/*end the trial*/
void CVT_Spirit::end_trial(EvolutionLoopFunctions &cLoopFunctions)
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
			//final_bd.push_back(StatFuns::laplace_smoothing(freqs[i][j], total_observations, alpha_smooth, num_joint_actuator_bins));
			if (total_observations == 0)
			{
				final_bd.push_back(1.0 / (float)num_joint_actuator_bins);
			}
			else
			{
				final_bd.push_back(freqs[i][j] / total_observations);
			}
#ifdef PRINTING

			std::cout << "prob_{" << i << "," << j << "}=" << final_bd.back() << std::endl;
#endif
		}
	}
	return final_bd;
}

/*after getting outputs, can update the descriptor if needed*/
void CVT_Spirit::set_output_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions)
{
	size_t sens_bin = cLoopFunctions.get_quadrant_bin();
	size_t act_bin = cLoopFunctions.get_joint_actuator_bin(num_actuator_bins);
	++freqs[sens_bin][act_bin];
}

/*summarise BD at the end of trials*/
std::vector<float> CVT_Spirit::after_trials(EvolutionLoopFunctions &cLoopFunctions)
{
	std::vector<float> final_bd = get_bd();
	return final_bd;
}

// NonMarkovianStochasticPolicyInduction::NonMarkovianStochasticPolicyInduction()
// {
// 	bd.resize(behav_dim);
// }

void CVT_Trajectory::before_trials(EvolutionLoopFunctions &cLoopFunctions)
{
	final_bd.clear();
}
CVT_Trajectory::CVT_Trajectory(EvolutionLoopFunctions &cLoopFunctions, size_t num_steps)
{
	num_chunks = behav_dim / (2 * cLoopFunctions.m_unNumberTrials);

	periodicity = num_steps / (num_chunks);
	argos::CVector3 max = cLoopFunctions.GetSpace().GetArenaSize();
	maxX = max.GetX();
	maxY = max.GetY();
}
/*after getting outputs, can update the descriptor if needed*/
void CVT_Trajectory::set_output_descriptor(size_t robot_index, EvolutionLoopFunctions &cLoopFunctions)
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
std::vector<float> CVT_Trajectory::after_trials(EvolutionLoopFunctions &cLoopFunctions)
{
	return final_bd;
}

EnvironmentDiversity::EnvironmentDiversity(std::string path, size_t num_generators)
{
	this->bd.resize(1);
	/* note path is the path without the .argos prefix */
	for (size_t i = 1; i <= num_generators; ++i)
	{
		env_generators.push_back(new ConfigurationBasedGenerator(path + std::to_string(i) + ".argos"));
	}
}

/* before all trials, prepare */
void EnvironmentDiversity::before_trials(EvolutionLoopFunctions &cLoopFunctions)
{
	// select a random generator
	id = cLoopFunctions.m_pcRNG->Uniform(CRange<int>(0, env_generators.size() - 1));
	env_generators[id]->generate(&cLoopFunctions);
}
/*summarise BD at the end of trials*/
std::vector<float> EnvironmentDiversity::after_trials(EvolutionLoopFunctions &cLoopFunctions)
{
	std::vector<float> final_bd(1, (float)id / (float)env_generators.size());
	return final_bd;
}
