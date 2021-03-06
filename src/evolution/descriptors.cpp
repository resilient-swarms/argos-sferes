
#include <src/core/statistics.h>
#include <src/evolution/base_evol_loop_functions.h>
#include <src/evolution/descriptors.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <iterator>
#include <random>
#include <boost/graph/strong_components.hpp>
#define SENSOR_ACTIVATION_THRESHOLD 0.5

void write_individual(std::vector<float> bd, float fitness, size_t individual, std::string filename)
{

	std::ofstream ofs;
	ofs.open(filename.c_str(), std::ios::app);

	ofs << individual << "    ";
	size_t offset = 0;
	for (size_t dim = 0; dim < bd.size(); ++dim)
		ofs << bd[dim] << " ";
	ofs << " " << fitness << " ";
	ofs << std::endl;
	ofs.flush();
	ofs.close();
}

/***********************************************/

Descriptor::Descriptor(size_t dims) : behav_dim(dims)
{
	geometric_median = false;
	bd.resize(behav_dim);
}
void Descriptor::before_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
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
std::vector<float> Descriptor::after_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
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
				std::cout << "bd" << i << " not in [0,1]:" << final_bd[i] << std::endl;
				StatFuns::clip(final_bd[i], 0.0f, 1.0f);
			}
		}
	}

	return final_bd;
}

/**********************************************************************/

void AverageDescriptor::set_input_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
{

	for (size_t i = 0; i < cLoopFunctions.inputs.size() - 1; ++i)
	{
		this->bd[i][current_trial] += (cLoopFunctions.inputs[i] >= SENSOR_ACTIVATION_THRESHOLD) ? 1.0 : 0.0;
	}
	++num_updates;
}

/*end the trial*/
void AverageDescriptor::end_trial(BaseEvolutionLoopFunctions &cLoopFunctions)
{

	for (size_t i = 0; i < cLoopFunctions.inputs.size() - 1; ++i)
	{
		this->bd[i][current_trial] /= (float)num_updates;
		if (!StatFuns::in_range(this->bd[i][current_trial], 0.0f, 1.0))
		{
			std::cout << "bd" << i << " not in [0,1]:" << bd[i][current_trial] << std::endl;
			StatFuns::clip(bd[i][current_trial], 0.0f, 1.0f);
		};
	}
}
/*********************************************************************************/

NeuralDescriptor::NeuralDescriptor()
{
	nb_input_output = ParamsDnn::dnn::nb_inputs + ParamsDnn::dnn::nb_outputs;
	max_nb_neurons = ParamsDnn::dnn::max_nb_neurons * 2;
	max_nb_connections = ParamsDnn::dnn::max_nb_conns * 2;
}

void NeuralDescriptor::end_trial(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	//nb connections
	unsigned nb_connections = cLoopFunctions.m_pcvecController[0]->nn.get_nb_connections();
	if (nb_connections > max_nb_connections) // cap the bd so at least the evolution will not throw an exception
	{
		// a bd of 1 now means the nb_connections is >= max_nb_connections
		this->bd[0][current_trial] = 1;
	}
	else
	{
		float prc_connections = nb_connections / max_nb_connections;
		this->bd[0][current_trial] = prc_connections;
	}

	//nb neurons
	unsigned nb_neurons = cLoopFunctions.m_pcvecController[0]->nn.get_nb_neurons() - nb_input_output;
	if (nb_neurons > max_nb_neurons) // cap the bd so at least the evolution will not throw an exception
	{
		// a bd of 1 now means the nb_neurons is >= max_nb_neurons
		this->bd[1][current_trial] = 1;
	}
	else
	{
		float prc_neurons = nb_neurons / max_nb_neurons;
		this->bd[1][current_trial] = prc_neurons;
	}
}

void NeuralCyclesDescriptor::end_trial(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	this->bd[0][current_trial] = strongly_connected(cLoopFunctions);

	// Degree ditribution of all the input nodes when k=0.
	// i.e. what proportion of the input nodes have 0 outgoing connections
	auto g = cLoopFunctions.m_pcvecController[0]->nn.get_graph();
	auto input_nodes = cLoopFunctions.m_pcvecController[0]->nn.get_inputs();
	int k_degree = 0;
	for (int i = 0; i < input_nodes.size(); i++)
	{
		if (g.out_edge_list(input_nodes[i]).size() == 0)
		{
			k_degree++;
		}
	}
#ifdef PRINTING
	std::cout << "Total number of input nodes: " << input_nodes.size() << std::endl;
	std::cout << "Total number of input nodes with degree 0: " << k_degree << std::endl;
#endif
	float prc = k_degree / (float)input_nodes.size();
	this->bd[1][current_trial] = prc;
}

float NeuralCyclesDescriptor::strongly_connected(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	using namespace boost;

	auto g = cLoopFunctions.m_pcvecController[0]->nn.get_graph();
	typedef decltype(g) Graph;
	typedef Graph::vertex_iterator VertexIterator;
	typedef Graph::vertex_descriptor VertexDesc;
	typedef std::map<VertexDesc, size_t> VertexDescMap;

	// make vertex index map
	VertexDescMap idxMap;
	associative_property_map<VertexDescMap> indexMap(idxMap);
	VertexIterator di, dj;
	tie(di, dj) = vertices(g);
	for (int i = 0; di != dj; ++di, ++i)
	{
		put(indexMap, (*di), i);
	}

	// Calculate the strongly connected subgraph
	// potential alternative implimentation: hawick_circuits
	std::map<VertexDesc, size_t> compMap;
	associative_property_map<VertexDescMap> componentMap(compMap);
	int num = strong_components(g, componentMap, vertex_index_map(indexMap));
	// Note: neurons not in a cycle are represented as a subgraph of size = 1
	// this is unfortunately the same as a recurrent connection.

	// Get the size of each subgraph
	std::vector<int> mean_comp_size(num);
	for (std::map<VertexDesc, size_t>::iterator it = compMap.begin(); it != compMap.end(); ++it)
	{
		mean_comp_size[it->second]++;
	}

	unsigned nb_comps = 0;		  // number of cycles
	unsigned nb_neurons_comp = 0; // total number of neurons in a cycle
	for (int i = 0; i < mean_comp_size.size(); i++)
	{
		if (mean_comp_size[i] > 1)
		{
			nb_comps++;
			nb_neurons_comp += mean_comp_size[i];
		}
	}
	// get all recurrent connections of the graph separately and add them to the totals
	Graph::edge_iterator ei, ei_end;
	for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
	{
		if (source(*ei, g) == target(*ei, g))
		{
			nb_comps++;
			nb_neurons_comp++;
		}
	}

	//number of neurons in the graph neurons
	auto nb_neurons = cLoopFunctions.m_pcvecController[0]->nn.get_nb_neurons();

#ifdef PRINTING
	std::cout << "Total number of cycles: " << nb_comps << std::endl;
	std::cout << "Total number of neurons in cycles: " << nb_neurons_comp << std::endl;
	std::cout << "Total number of neurons: " << nb_neurons << std::endl;
#endif

	if (nb_neurons == 0) // cannot divide by 0
	{
		return 0;
	}
	else
	{
		// the proportion of all neurons that are in cycles
		return (float)nb_neurons_comp / (float)nb_neurons;
	}
}

/*********************************************************************************/

IntuitiveHistoryDescriptor::IntuitiveHistoryDescriptor(BaseEvolutionLoopFunctions *cLoopFunctions, size_t behav_dim) : Descriptor(behav_dim)
{

	//define member variables
	center = cLoopFunctions->get_arenacenter();

	// initialise grid (for calculating coverage and uniformity)
	argos::CVector3 max = cLoopFunctions->get_arenasize();
	argos::CVector3 min = center - 0.5 * max;
	max_deviation = StatFuns::get_minkowski_distance(max, center);
	// if (cLoopFunctions->m_unNumberRobots != 1)
	// {
	// 	throw std::runtime_error("number of robots should be equal to 1 when choosing IntuitiveHistoryDescriptor");
	// }
	coverageCalc = CoverageCalc(cLoopFunctions);
}

/*reset BD at the start of a trial*/
void IntuitiveHistoryDescriptor::start_trial()
{
	Descriptor::start_trial();
	deviation = 0.0f;
	//velocity_stats=RunningStat();  // dropped the velocity stats
}
void IntuitiveHistoryDescriptor::set_input_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
{
	//add to the deviation (to get the mean after all trials have finished)
	CVector3 pos = cLoopFunctions.get_position(cLoopFunctions.m_pcvecRobot[robot_index]);
	deviation += StatFuns::get_minkowski_distance(pos, center); // assume single robot
	coverageCalc.update(pos);
	++num_updates;
}

void IntuitiveHistoryDescriptor::set_output_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
{
	//nothing here
}

void IntuitiveHistoryDescriptor::end_trial(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	/*add behavioural metrics */

	//uniformity of probabilities
	std::vector<float> probabilities = coverageCalc.get_probs(num_updates);
	float uniformity = StatFuns::uniformity(probabilities);
	this->bd[0][current_trial] = uniformity;
	//deviation from the center
	float avg_deviation = deviation / (max_deviation * (float)num_updates);
	this->bd[1][current_trial] = avg_deviation;

	// #if BEHAV_DIM == 3
	float coverage = coverageCalc.get_coverage();
	this->bd[2][current_trial] = coverage;
	// #endif

#ifdef PRINTING
	std::cout << "uniformity" << uniformity << std::endl;
	std::cout << "Max deviation" << max_deviation << std::endl;
	std::cout << "deviation" << avg_deviation << std::endl;
	// #if BEHAV_DIM == 3
	std::cout << "coverage" << coverage << std::endl;
// #endif
#endif

	coverageCalc.after_trial();
}

float Entity::distance(const Entity &e1, const Entity &e2)
{
	return StatFuns::get_minkowski_distance(e1.position, e2.position);
}

/******************************************************************/

SDBC::SDBC(BaseEvolutionLoopFunctions *cLoopFunctions, std::string init_type, size_t bd) : Descriptor(bd)
{

	include_std = init_type.find("std") != std::string::npos ? true : false; // will calculate standard deviations as well

	size_t num_attr;

	if (init_type.find("Gomes") != std::string::npos)
	{
		include_closest_robot = true; // add closest robot distance
		attribute_setter = new SpeedAttributeSetter(cLoopFunctions);
		num_attr = 2;			 // linear speed and turn speed
		geometric_median = true; //apply geometric median instead of mean

		// get the average of the closest robot distance in the given uniform robot positioning

		float min = 0; // minimal_robot_distance(cLoopFunctions);
		float max = get_uniform_closestdist(cLoopFunctions);
		// range for the closest robot feature
		maxrange.insert(std::pair<std::string, std::pair<float, float>>("closest_robot", std::pair<float, float>(min, max)));
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

		// range for the boxesrobots feature
		maxrange.insert(std::pair<std::string, std::pair<float, float>>("boxesrobots", get_wallsrobots_range(cLoopFunctions)));
	}
	else if (init_type.find("sdbc_robots") != std::string::npos)
	{
		init_robots(num_attr, cLoopFunctions);
	}
	else
	{
		throw std::runtime_error("init type " + init_type + "not found");
	}

	if (include_std)
	{
		num_features = behav_dim / 2;
		temp_bd.resize(num_features);
	}
	else
	{
		temp_bd.resize(behav_dim);
	}
	num_groups = entity_groups.size();
	for (auto &kv : entity_groups)
	{
		if (kv.second.max_size > 1 && kv.first == "robots")
		{
			within_comparison_groups.push_back(kv.first); // within-group distance computations

			// range for the robots feature (within robot comparison)
			float min = 0; //minimal_robot_distance(cLoopFunctions);
			float max = get_max_avgdist(cLoopFunctions);
			maxrange.insert(std::pair<std::string, std::pair<float, float>>("robots", std::pair<float, float>(min, max)));
		}

		between_comparison_groups.push_back(kv.first); // between-group distance computations

		if (kv.second.max_size != kv.second.min_size)
		{
			variable_groups.push_back(kv.first);
		}
	}

	//maxdist = StatFuns::get_minkowski_distance(CVector3(attribute_setter->maxX, attribute_setter->maxY, 0), CVector3::ZERO);
}

/* minimial robot distance */
float SDBC::minimal_robot_distance(BaseEvolutionLoopFunctions *cLoopFunctions)
{
	SBoundingBox bounding_box = cLoopFunctions->get_embodied_entity(0)->GetBoundingBox();

	return StatFuns::get_minkowski_distance(bounding_box.MaxCorner, bounding_box.MinCorner); // at least one robot body
}
/* uniform closest distance as proxy to the maximal avg closest distance */
float SDBC::get_uniform_closestdist(BaseEvolutionLoopFunctions *cLoopFunctions)
{
	/* approximate equation obtained by recursively adding agents at maximum distance to the previous */
	CVector3 max = cLoopFunctions->get_arenasize();
	float maxdist = StatFuns::get_minkowski_distance(max, CVector3::ZERO);
	// 4 robots --> maxSide; 5+ robots --> maxdist/2; 9+ robots: maxSide/2 (seems to work in drawings)
	float maxSide = std::max(max.GetX(), max.GetY());
	if (cLoopFunctions->m_unNumberRobots > 9)
	{
		return maxSide / 2.0f;
	}
	else if (cLoopFunctions->m_unNumberRobots >= 5)
	{
		return maxdist / 2;
	}
	else if (cLoopFunctions->m_unNumberRobots == 4)
	{
		return maxSide;
	}
	else
	{
		return maxdist;
	}
}
/* Divide the max arena distance by the number of robots to get max average robotdist */
float SDBC::get_max_avgdist(BaseEvolutionLoopFunctions *cLoopFunctions)
{
	/* approximate equation obtained by recursively adding agents at maximum distance to the previous */
	CVector3 max = cLoopFunctions->get_arenasize();
	float maxdist = StatFuns::get_minkowski_distance(max, CVector3::ZERO);
	float robot_correction = std::sqrt(cLoopFunctions->m_unNumberRobots / 2.0f);
	// 2 robots --> maxdist; 4 robots --> maxdist/sqrt(2); 8 robots --> maxdist/2 (seems to work in drawings)
	maxdist = maxdist / robot_correction; // NOTE: if number of robots changes during the trial, need to use entity group's max_size (elsewhere too)
	float maxSide = std::max(max.GetX(), max.GetY());
	return std::max(maxdist, maxSide); // maxSide is the minimum possible ( and seems to be Mase's choice for maximum)
}
/* walls robots min and max distance */
std::pair<float, float> SDBC::get_wallsrobots_range(BaseEvolutionLoopFunctions *cLoopFunctions)
{
	// the range depends on the arena; since many robots can be close to each other without affecting this metric
	// here approximate this by filling the XY-grid and calculating the distance, then taking max and min
	// simulating one agent is sufficient this way because maximal distance and minimal distance to walls can be at one point only
	// and agents can easily join together
	CVector3 maxArena = cLoopFunctions->get_arenasize();
	SBoundingBox bounding_box = cLoopFunctions->get_embodied_entity(0)->GetBoundingBox();

	float xdim = bounding_box.MaxCorner.GetX() - bounding_box.MinCorner.GetX();
	float ydim = bounding_box.MaxCorner.GetY() - bounding_box.MinCorner.GetY();
	float dim = std::max(xdim, ydim); //robot body unit

	float max = 0;
	float min = std::numeric_limits<float>::infinity();

	Entity_Group walls = entity_groups["boxes"];
	// define grid based
	for (float x = dim; x <= maxArena.GetX() - dim; x += dim)
	{
		for (float y = dim; y <= maxArena.GetY() - dim; y += dim)
		{
			CVector3 pos = CVector3(x, y, 0.0f);
			float avg_dist = 0.0f;
			for (size_t i = 0; i < walls.get_absolute_size(); ++i)
			{
				float dist = StatFuns::get_minkowski_distance(pos, walls[i].position);
				avg_dist += dist;
			}
			avg_dist /= (float)walls.get_absolute_size();
			if (avg_dist > max)
			{
				max = avg_dist;
			}
			if (avg_dist < min)
			{
				min = avg_dist;
			}
		}
	}
	return std::pair<float, float>(min, max);
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
		CVector3 position = dynamic_cast<BaseLoopFunctions *>(cLoopFunctions)->get_wall_pos(cBody.GetId());
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
	// here it is assumed fixed number of robots
	std::vector<Entity> robots;
	size_t num_robots = static_cast<BaseEvolutionLoopFunctions *>(cLoopFunctions)->m_unNumberRobots;
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
	for (std::string &key : within_comparison_groups)
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
			float num_calcs = 0.0f;
			for (int i = 0; i < group.get_absolute_size(); ++i)
			{
				for (int j = 1; j < group.get_absolute_size(); ++j)
				{
					sum += Entity::distance(group[i], group[j]);
					num_calcs += 1.0f;
				}
			}
			sum = sum / num_calcs;
			sum = normalise(sum, key);
			this->temp_bd[bd_index].push_back(sum);
			++bd_index;
		}
	}
}
/* normalise to get full range across [0,1] */
float SDBC::normalise(float number, std::string key)
{
	std::pair<float, float> range = maxrange[key];
	return (number - std::get<0>(range)) / (std::get<1>(range) - std::get<0>(range));
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
				float num_calcs = 0.0f;
				for (Entity &e1 : group.entities)
				{
					for (Entity &e2 : group2.entities)
					{
						sum += Entity::distance(e1, e2);
						++num_calcs;
					}
				}
				sum = sum / num_calcs; // divide by product of group sizes; add for now, we will average the number of times
				sum = normalise(sum, key + key2);
				temp_bd[bd_index].push_back(sum);
				++bd_index;
			}
		}
	}
}

/* distance to closest robot */
void SDBC::add_closest_robot_dist(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	float mean_dist = 0.0f;
	// we already have positions available in curr_pos
	for (size_t i = 0; i < cLoopFunctions.m_unNumberRobots; ++i)
	{
		float mindist = std::numeric_limits<float>::infinity();
		for (size_t j = 0; j < cLoopFunctions.m_unNumberRobots; ++j)
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
	mean_dist = mean_dist / ((float)cLoopFunctions.m_unNumberRobots);
	mean_dist = normalise(mean_dist, "closest_robot");
	temp_bd[bd_index].push_back(mean_dist);
}

/* prepare for trials*/
void SDBC::before_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
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
void SDBC::set_input_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
{
}
/*after getting outputs, can update the descriptor if needed*/
void SDBC::set_output_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
{
	entity_groups["robots"][robot_index].set_attributes(attribute_setter->get_attributes(robot_index, cLoopFunctions), cLoopFunctions.curr_pos[robot_index]);
}
/*after the looping over robots*/
void SDBC::after_robotloop(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	add_group_sizes();				// in case group sizes are variable, group size is descriptor
	add_group_meanstates();			// for each group calculate mean state (if it has features)
	add_between_group_dispersion(); // calculate distances between groups
	add_within_group_dispersion();	// calculate distances within groups
	if (include_closest_robot)
	{
		add_closest_robot_dist(cLoopFunctions); // calculate closest robot distance
	}
	bd_index = 0;
	++num_updates;
}
/*end the trial*/
// void SDBC::end_trial(BaseEvolutionLoopFunctions &cLoopFunctions)
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
void SDBC::end_trial(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	//After a simulation has ended, the samples obtained for each
	//behaviour feature at each time step are aggregated to assem-ble a fixed-length characterisation vector.
	for (size_t i = 0; i < behav_dim; ++i)
	{
		if (include_std)
		{
			if (i % 2 == 0)
			{
				bd[i][current_trial] = StatFuns::mean(this->temp_bd[i / 2]); // already normalised
			}
			else
			{
				// https://en.wikipedia.org/wiki/Standard_deviation: For a set of N > 4 data spanning a range of values R, an upper bound on the standard deviation s is given by s = 0.6R
				// with R=1 then s = 0.6 maximally --> multiply by 1/0.6 = 1.666666666667
				bd[i][current_trial] = 1.666666666666666666666 * StatFuns::standard_dev(this->temp_bd[i / 2]);
			}
		}
		else
		{
			bd[i][current_trial] = StatFuns::mean(this->temp_bd[i]);
		}

		if (!StatFuns::in_range(bd[i][current_trial], 0.0f, 1.0f))
		{
			std::cout << "bd" << i << " not in [0,1]:" << bd[i][current_trial] << std::endl;
			StatFuns::clip(bd[i][current_trial], 0.0f, 1.0f);
		};
	}
}

/************************************************************************/

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
void CVT_MutualInfo::before_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
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
void CVT_MutualInfo::set_input_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
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
void CVT_MutualInfo::after_robotloop(BaseEvolutionLoopFunctions &cLoopFunctions)
{
}

/*end the trial*/
void CVT_MutualInfo::end_trial(BaseEvolutionLoopFunctions &cLoopFunctions)
{
}

/*summarise BD at the end of trials*/
std::vector<float> CVT_MutualInfo::after_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
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
void CVT_MutualInfoAct::before_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
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
void CVT_MutualInfoAct::set_output_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
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

/*************************************************************************/

CVT_Spirit::CVT_Spirit(size_t behav_dim) : Descriptor(behav_dim)
{
	freqs.resize(num_joint_sensory_bins);
}

/* prepare for trials*/
void CVT_Spirit::before_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
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
void CVT_Spirit::after_robotloop(BaseEvolutionLoopFunctions &cLoopFunctions)
{
}

/*end the trial*/
void CVT_Spirit::end_trial(BaseEvolutionLoopFunctions &cLoopFunctions)
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
void CVT_Spirit::set_output_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
{
	size_t sens_bin = cLoopFunctions.get_quadrant_bin();
	size_t act_bin = cLoopFunctions.get_joint_actuator_bin(num_actuator_bins);
	++freqs[sens_bin][act_bin];
}

/*summarise BD at the end of trials*/
std::vector<float> CVT_Spirit::after_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	std::vector<float> final_bd = get_bd();
	return final_bd;
}

/*************************************************************************/

CVT_RAB_Spirit::CVT_RAB_Spirit(size_t behav_dim) : CVT_Spirit(behav_dim)
{
	/* most of the code remains the same as CVT_Spirit except the meaning of the bins and the number 
	* number of bins 
	/*

	/* num actuator bins */
	num_actuator_bins = 4;

	/* number of joint sensory bins */
	num_joint_sensory_bins = 64; // here 2^2 * 2^4 bins for mean (front-back RAB; quadrant proxi)

	/* number of joint actuator bins simply */
	num_joint_actuator_bins = 16; // 4 bins per wheel

	freqs.resize(num_joint_sensory_bins);
}

/*after getting outputs, can update the descriptor if needed*/
void CVT_RAB_Spirit::set_output_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
{
	size_t sens_bin = cLoopFunctions.get_quadrant_binRAB();
	size_t act_bin = cLoopFunctions.get_joint_actuator_bin(num_actuator_bins);
	++freqs[sens_bin][act_bin];
}


CVT_Ground_Spirit::CVT_Ground_Spirit(size_t behav_dim) : CVT_Spirit(behav_dim)
{
	/* most of the code remains the same as CVT_Spirit except the meaning of the bins and the number 
	* number of bins 
	/*

	/* num actuator bins */
	num_actuator_bins = 4;

	/* number of joint sensory bins */
	num_joint_sensory_bins = 48; // here 3 * 2^4 bins for mean (white-grey-black; 
								// quadrant proxi)

	/* number of joint actuator bins simply */
	num_joint_actuator_bins = 16; // 4 bins per wheel

	freqs.resize(num_joint_sensory_bins);
}
/*after getting outputs, can update the descriptor if needed*/
void CVT_Ground_Spirit::set_output_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
{
	size_t sens_bin = cLoopFunctions.get_binGround();
	size_t act_bin = cLoopFunctions.get_joint_actuator_bin(num_actuator_bins);
	++freqs[sens_bin][act_bin];
}

/*************************************************************************/

MultiAgent_Spirit::MultiAgent_Spirit()
{
	/* most of the code remains the same as CVT_Spirit except the meaning of the bins and the number 
	* number of bins 
	/*

	/* number of joint sensory bins */
	num_joint_sensory_bins = 36; // here 9 (3*3) bins for mean CM of position + 4 (2*2) bins for SD of position

	/* number of joint actuator bins simply */
	num_joint_actuator_bins = 16; // here  2*2 for mean displacement 2*2 for SD displacement

	freqs.resize(num_joint_sensory_bins);
}

/*after getting outputs, can update the descriptor if needed*/
void MultiAgent_Spirit::set_output_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
{
	// nothing here; do in after_robotloop
}
/*after getting outputs, can update the descriptor if needed*/
void MultiAgent_Spirit::after_robotloop(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	size_t sens_bin = cLoopFunctions.get_CM_bin(3, 2);
	size_t act_bin = cLoopFunctions.get_swarmmovement_bin(2, 2);
	++freqs[sens_bin][act_bin];
}
// NonMarkovianStochasticPolicyInduction::NonMarkovianStochasticPolicyInduction()
// {
// 	bd.resize(behav_dim);
// }

/************************************************************************/

void CVT_Trajectory::before_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	final_bd.clear();
}
CVT_Trajectory::CVT_Trajectory(BaseEvolutionLoopFunctions &cLoopFunctions, size_t num_steps)
{
	num_chunks = behav_dim / (2 * cLoopFunctions.m_unNumberTrials);

	periodicity = num_steps / (num_chunks);
	argos::CVector3 max = cLoopFunctions.get_arenasize();
	maxX = max.GetX();
	maxY = max.GetY();
}
/*after getting outputs, can update the descriptor if needed*/
void CVT_Trajectory::set_output_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
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
std::vector<float> CVT_Trajectory::after_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	return final_bd;
}

/***********************************************************************/
// EnvironmentDiversity::EnvironmentDiversity(BaseEvolutionLoopFunctions &cLoopFunctions,std::string path, size_t num_generators)
// {
// 	this->bd.resize(1);
// 	cLoopFunctions.generator = new EnvironmentGenerator(cLoopFunctions.seed);
// }

// /* before all trials, prepare */
// void EnvironmentDiversity::before_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
// {

// }
// /*summarise BD at the end of trials*/
// std::vector<float> EnvironmentDiversity::after_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
// {
// 	std::vector<float> final_bd({0.,0.0});//(1, (float)id / (float)env_generators.size());
// 	return final_bd;
// }

/***********************************************************************/

StaticDescriptor::StaticDescriptor(std::vector<float> bd)
{
	final_bd = bd;
}

/*summarise BD at the end of trials*/
std::vector<float> StaticDescriptor::after_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	return final_bd;
}

/*******************************************************************************/

/* descriptor not used for evolution but for recording state-action trajectories*/
SubjectiveHistoryDescriptor::SubjectiveHistoryDescriptor(const std::string &filename)
{
	// std::ios::out will write a new file
	file_writer.open(filename, std::ios::out); //
}

/* prepare for trials*/
void SubjectiveHistoryDescriptor::before_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
}
/*reset BD at the start of a trial*/
void SubjectiveHistoryDescriptor::start_trial()
{
	if (num_updates % frequency == 0)
	{
		file_writer << std::fixed << "T" << current_trial << ":\n";
	}
}
/*after getting inputs, can update the descriptor if needed*/
void SubjectiveHistoryDescriptor::set_input_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
{
	if (num_updates % frequency == 0)
	{
		for (size_t i = 0; i < cLoopFunctions.inputs.size(); ++i)
		{
			file_writer << std::fixed << std::setprecision(2) << cLoopFunctions.inputs[i] << ",";
		}
	}
}

/*after getting outputs, can update the descriptor if needed*/
void SubjectiveHistoryDescriptor::set_output_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
{
	if (num_updates % frequency == 0)
	{
		for (size_t i = 0; i < cLoopFunctions.outf.size() - 1; ++i)
		{
			file_writer << std::setprecision(2) << cLoopFunctions.outf[i] << ",";
		}
		file_writer << std::setprecision(2) << cLoopFunctions.outf.back() << "# ";
	}
}
/*after the looping over robots*/
void SubjectiveHistoryDescriptor::after_robotloop(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	if (num_updates % frequency == 0)
	{
		file_writer << "\n";
	}
	++num_updates;
}

/*summarise BD at the end of trials*/
std::vector<float> SubjectiveHistoryDescriptor::after_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	file_writer.close();
	return std::vector<float>(0);
}

/*******************************************************************************/

/* descriptor not used for evolution but for recording state-action trajectories*/
ObjectiveHistoryDescriptor::ObjectiveHistoryDescriptor(const std::string &filename)
{
	// std::ios::out will write a new file
	file_writer.open(filename, std::ios::out); //
}

/* prepare for trials*/
void ObjectiveHistoryDescriptor::before_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
}
/*reset BD at the start of a trial*/
void ObjectiveHistoryDescriptor::start_trial()
{
	if (num_updates % frequency == 0)
	{
		file_writer << std::fixed << "T" << current_trial << ":\n";
	}
}
/*after getting inputs, can update the descriptor if needed*/
void ObjectiveHistoryDescriptor::set_input_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
{
	if (num_updates % frequency == 0)
	{
		CVector3 pos = cLoopFunctions.get_position(cLoopFunctions.m_pcvecRobot[robot_index]);
		file_writer << std::fixed << std::setprecision(2) << pos.GetX() << "," << pos.GetY();
		file_writer << "# ";
	}
}

/*after getting outputs, can update the descriptor if needed*/
void ObjectiveHistoryDescriptor::set_output_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
{
}
/*after the looping over robots*/
void ObjectiveHistoryDescriptor::after_robotloop(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	if (num_updates % frequency == 0)
	{
		file_writer << "\n";
	}
	++num_updates;
}
/*summarise BD at the end of trials*/
std::vector<float> ObjectiveHistoryDescriptor::after_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	file_writer.close();
	return std::vector<float>(0);
}

/*******************************************************************************/

AnalysisDescriptor::AnalysisDescriptor(size_t individ, std::string file_n, std::map<std::string, Descriptor *> slaves) : slave_descriptors(slaves),
																														 individual(individ),
																														 file_name(file_n)
{
}

/* prepare for trials*/
void AnalysisDescriptor::before_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	current_trial = 0;
	for (auto const &x : slave_descriptors)
	{
		x.second->before_trials(cLoopFunctions);
	}
}
/*reset BD at the start of a trial*/
void AnalysisDescriptor::start_trial()
{

	for (auto const &x : slave_descriptors)
	{
		x.second->start_trial();
	}
}
/*after getting inputs, can update the descriptor if needed*/
void AnalysisDescriptor::set_input_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
{
	for (auto const &x : slave_descriptors)
	{
		x.second->set_input_descriptor(robot_index, cLoopFunctions);
	}
}

/*after getting outputs, can update the descriptor if needed*/
void AnalysisDescriptor::set_output_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
{
	for (auto const &x : slave_descriptors)
	{
		x.second->set_output_descriptor(robot_index, cLoopFunctions);
	}
}
/*after the looping over robots*/
void AnalysisDescriptor::after_robotloop(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	for (auto const &x : slave_descriptors)
	{
		x.second->after_robotloop(cLoopFunctions);
	}
}

/*summarise BD at the end of trials*/
void AnalysisDescriptor::end_trial(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	for (auto const &x : slave_descriptors)
	{
		x.second->end_trial(cLoopFunctions);
	}
}
/* get the descriptor by its id-string and then print it to file*/
void AnalysisDescriptor::analyse_individual(BaseEvolutionLoopFunctions &cLoopFunctions, float fFitness)
{
	for (auto const &desc : slave_descriptors)
	{
		if (desc.first == "identification" || desc.first == "identification_wheel")
		{
			for (size_t i = 0; i < cLoopFunctions.m_unNumberRobots; ++i)
			{
				cLoopFunctions.current_robot = i;
				write_individual(desc.second->after_trials(cLoopFunctions), cLoopFunctions.get_robot_fitness(i), individual, file_name + desc.first + ".dat");
			}
		}
		else
		{
			std::vector<float> bd = desc.second->after_trials(cLoopFunctions);
			if (!bd.empty())
			{
				write_individual(bd, fFitness, individual, file_name + desc.first + ".dat");
			}
		}
	}
}
#ifdef HETEROGENEOUS
void IdentificationDescriptor::before_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
}

void IdentificationDescriptor::start_trial()
{
}

void IdentificationDescriptor::set_input_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
{
	//the proportion of times the front-left proximity sensors are activated (above 0.5)

	//Pm is the proportion of times the front-mid proximity sensor is activated

	//Pr  is the proportion of times the front-right  proximity sensors are activated
	//Pb is the proportion of times the back proximity sensors are activated
	//Gwhite is the proportion of times the ground sensors are maximal (white)
	//Gblack is the proportion of time the ground sensors are minimal (black)

	//proximity
	std::vector<float> activations = cLoopFunctions.get_inputgroup_activations({1, 2, 4, 6}, 0.00);
	//white
	activations.push_back(cLoopFunctions.get_inputgroup_activations({8}, 0.70, 7)[0]);
	//black
	activations.push_back(cLoopFunctions.get_inputgroup_activations_smaller({8}, -0.70, 7)[0]);

	size_t offset = robot_index * num_features;
	// now add activations to the bd
	for (size_t i = 0; i < num_features; ++i)
	{
		bd_vec[offset + i] += activations[i];
	}
	++updates[robot_index];
}

/*end the trial*/
void IdentificationDescriptor::end_trial(BaseEvolutionLoopFunctions &cLoopFunctions)
{
}

std::vector<float> IdentificationDescriptor::after_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	size_t offset = cLoopFunctions.current_robot * num_features;
	for (size_t i = offset; i < offset + num_features; ++i)
	{
		this->bd_vec[i] /= (float)updates[cLoopFunctions.current_robot];
		if (!StatFuns::in_range(this->bd_vec[i], 0.0f, 1.0))
		{
			std::cout << "bd" << i << " not in [0,1]:" << bd_vec[i] << std::endl;
			StatFuns::clip(bd_vec[i], 0.0f, 1.0f);
		};
	}
	return std::vector<float>(bd_vec.begin() + offset, bd_vec.begin() + offset + num_features);
}

void IdentificationWheelDescriptor::before_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
}

void IdentificationWheelDescriptor::start_trial()
{
}

void IdentificationWheelDescriptor::set_input_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
{
	//the proportion of times the front-left proximity sensors are activated (above 0.5)

	//Pm is the proportion of times the front-mid proximity sensor is activated

	//Pr  is the proportion of times the front-right  proximity sensors are activated
	//Pb is the proportion of times the back proximity sensors are activated
	//Gwhite is the proportion of times the ground sensors are maximal (white)
	//Gblack is the proportion of time the ground sensors are minimal (black)

	//proximity
	std::vector<float> activations = cLoopFunctions.get_inputgroup_activations({4, 6}, 0.00); // front and back
	//white
	activations.push_back(cLoopFunctions.wheel_turn_velocity_01(robot_index));
	//black
	activations.push_back(cLoopFunctions.wheel_linear_velocity_01(robot_index));

	size_t offset = robot_index * num_features;
	// now add activations to the bd
	for (size_t i = 0; i < num_features; ++i)
	{
		bd_vec[offset + i] += activations[i];
	}
	++updates[robot_index];
}

/*end the trial*/
void IdentificationWheelDescriptor::end_trial(BaseEvolutionLoopFunctions &cLoopFunctions)
{
}

std::vector<float> IdentificationWheelDescriptor::after_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	size_t offset = cLoopFunctions.current_robot * num_features;
	for (size_t i = offset; i < offset + num_features; ++i)
	{
		this->bd_vec[i] /= (float)updates[cLoopFunctions.current_robot];
		if (!StatFuns::in_range(this->bd_vec[i], 0.0f, 1.0))
		{
			std::cout << "bd" << i << " not in [0,1]:" << bd_vec[i] << std::endl;
			StatFuns::clip(bd_vec[i], 0.0f, 1.0f);
		};
	}
	return std::vector<float>(bd_vec.begin() + offset, bd_vec.begin() + offset + num_features);
}

void PerfectIdentificationDescriptor::before_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
}

void PerfectIdentificationDescriptor::start_trial()
{
}

void PerfectIdentificationDescriptor::set_input_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
{
}

/*end the trial*/
void PerfectIdentificationDescriptor::end_trial(BaseEvolutionLoopFunctions &cLoopFunctions)
{
}
std::vector<float> PerfectIdentificationDescriptor::after_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	size_t index = cLoopFunctions.current_robot;
	argos::CThymioEntity *cThym = cLoopFunctions.m_pcvecRobot[index];
	ForagingThymioNN &cController = dynamic_cast<ForagingThymioNN &>(cThym->GetControllableEntity().GetController());
	if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_NONE)
	{ // ProxOFFSET, ProxDYN, GroundOFFSET, GroundDYN, Lwheel,Rwheel
		return {0, 0, 0, 0, 0, 0};
	}
	//proximity sensor faults
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_PROXIMITYSENSORS_SETMIN)
	{
		// ProxOFFSET, ProxDYN, GroundOFFSET, GroundDYN, Lwheel,Rwheel
		return {0.5, 0.5, 0, 0, 0, 0};
	}
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_PROXIMITYSENSORS_SETMAX)
	{
		// ProxOFFSET, ProxDYN, GroundOFFSET, GroundDYN, Lwheel,Rwheel
		return {1, 0.5, 0, 0, 0, 0};
	}
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_PROXIMITYSENSORS_SETRANDOM)
	{
		// ProxOFFSET, ProxDYN, GroundOFFSET, GroundDYN, Lwheel,Rwheel
		return {0, 1, 0, 0, 0, 0};
	}
	//ground sensor faults
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_GROUNDSENSORS_SETMIN)
	{
		// ProxOFFSET, ProxDYN, GroundOFFSET, GroundDYN, Lwheel,Rwheel
		return {0, 0, 0.5, 0.5, 0, 0};
	}
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_GROUNDSENSORS_SETMAX)
	{
		// ProxOFFSET, ProxDYN, GroundOFFSET, GroundDYN, Lwheel,Rwheel
		return {0, 0, 1, 0.5, 0, 0};
	}
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_GROUNDSENSORS_SETRANDOM)
	{
		// ProxOFFSET, ProxDYN, GroundOFFSET, GroundDYN, Lwheel,Rwheel
		return {0, 0, 0, 1, 0, 0};
	}
	//actuator faults
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_ACTUATOR_LWHEEL_SETHALF)
	{
		// ProxOFFSET, ProxDYN, GroundOFFSET, GroundDYN, Lwheel,Rwheel
		return {0, 0, 0, 0, 1, 0};
	}
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_ACTUATOR_RWHEEL_SETHALF)
	{
		// ProxOFFSET, ProxDYN, GroundOFFSET, GroundDYN, Lwheel,Rwheel
		return {0, 0, 0, 0, 0, 1};
	}
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_ACTUATOR_BWHEELS_SETHALF)
	{
		// ProxOFFSET, ProxDYN, GroundOFFSET, GroundDYN, Lwheel,Rwheel
		return {0, 0, 0, 0, 1, 1};
	}
	else
	{
		throw std::runtime_error("fault identification vector not implemented");
		return {};
	}
}

void RandomIdentificationDescriptor::before_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
}

void RandomIdentificationDescriptor::start_trial()
{
}

void RandomIdentificationDescriptor::set_input_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
{
}

/*end the trial*/
void RandomIdentificationDescriptor::end_trial(BaseEvolutionLoopFunctions &cLoopFunctions)
{
}
std::vector<float> RandomIdentificationDescriptor::after_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	size_t index = cLoopFunctions.current_robot;
	return bd_vec[index];
}

void PerfectIdentificationDescriptor2::before_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
}

void PerfectIdentificationDescriptor2::start_trial()
{
}

void PerfectIdentificationDescriptor2::set_input_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
{
}

/*end the trial*/
void PerfectIdentificationDescriptor2::end_trial(BaseEvolutionLoopFunctions &cLoopFunctions)
{
}
std::vector<float> PerfectIdentificationDescriptor2::after_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	size_t index = cLoopFunctions.current_robot;
	argos::CThymioEntity *cThym = cLoopFunctions.m_pcvecRobot[index];
	ForagingThymioNN &cController = dynamic_cast<ForagingThymioNN &>(cThym->GetControllableEntity().GetController());
	if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_NONE ||
		cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_FOOD_SCARCITY)
	{ // Prox, Ground, Lwheel,Rwheel, SoftwareNEST, SoftwareFood
		return {0, 0, 0, 0, 0, 0};
	}
	//proximity sensor faults
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_PROXIMITYSENSORS_SETMIN)
	{
		// Prox, Ground, Lwheel,Rwheel, SoftwareNEST, SoftwareFood
		return {0.3333, 0, 0, 0, 0, 0};
	}
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_PROXIMITYSENSORS_SETRANDOM)
	{
		// Prox, Ground, Lwheel,Rwheel, SoftwareNEST, SoftwareFood
		return {0.6666, 0, 0, 0, 0, 0};
	}
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_PROXIMITYSENSORS_SETMAX)
	{
		// Prox, Ground, Lwheel,Rwheel, SoftwareNEST, SoftwareFood
		return {1, 0, 0, 0, 0, 0};
	}

	//ground sensor faults
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_GROUNDSENSORS_SETMIN)
	{
		// Prox, Ground, Lwheel,Rwheel, SoftwareNEST, SoftwareFood
		return {0, 0.3333, 0, 0, 0, 0};
	}
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_GROUNDSENSORS_SETRANDOM)
	{
		// Prox, Ground, Lwheel,Rwheel, SoftwareNEST, SoftwareFood
		return {0, 0.6666, 0, 0, 0, 0};
	}
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_GROUNDSENSORS_SETMAX)
	{
		// Prox, Ground, Lwheel,Rwheel, SoftwareNEST, SoftwareFood
		return {0, 1, 0, 0, 0, 0};
	}

	//actuator faults
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_ACTUATOR_LWHEEL_SETHALF)
	{
		// Prox, Ground, Lwheel,Rwheel, SoftwareNEST, SoftwareFood
		return {0, 0, 1, 0, 0, 0};
	}
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_ACTUATOR_RWHEEL_SETHALF)
	{
		// Prox, Ground, Lwheel,Rwheel, SoftwareNEST, SoftwareFood
		return {0, 0, 0, 1, 0, 0};
	}
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_ACTUATOR_BWHEELS_SETHALF)
	{
		// Prox, Ground, Lwheel,Rwheel, SoftwareNEST, SoftwareFood
		return {0, 0, 1, 1, 0, 0};
	}
	//software-nest
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_SOFTWARE)
	{
		// Prox, Ground, Lwheel,Rwheel, SoftwareNEST, SoftwareFood
		return {0, 0, 0, 0, 1, 0};
	}
	//software-food
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_SOFTWARE_FOOD)
	{
		// Prox, Ground, Lwheel,Rwheel, SoftwareNEST, SoftwareFood
		return {0, 0, 0, 0, 0, 1};
	}
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_SOFTWARE_FOOD)
	{
		// Prox, Ground, Lwheel,Rwheel, SoftwareNEST, SoftwareFood
		return {0, 0, 0, 0, 0, 1};
	}
	else
	{
		throw std::runtime_error("fault identification vector not implemented");
		return {};
	}
}

void PerfectIdentificationDescriptorSorted::before_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
}

void PerfectIdentificationDescriptorSorted::start_trial()
{
}

void PerfectIdentificationDescriptorSorted::set_input_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
{
}

/*end the trial*/
void PerfectIdentificationDescriptorSorted::end_trial(BaseEvolutionLoopFunctions &cLoopFunctions)
{
}
std::vector<float> PerfectIdentificationDescriptorSorted::after_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	size_t index = cLoopFunctions.current_robot;
	argos::CThymioEntity *cThym = cLoopFunctions.m_pcvecRobot[index];
	ForagingThymioNN &cController = dynamic_cast<ForagingThymioNN &>(cThym->GetControllableEntity().GetController());
	//proximity sensor faults
	if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_PROXIMITYSENSORS_SETMIN)
	{
		return {1, 0, 0};
	}
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_PROXIMITYSENSORS_SETRANDOM)
	{
		return {0, 1, 0};
	}
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_PROXIMITYSENSORS_SETMAX)
	{
		return {0, 0, 1};
	}
	// ground sensor faults as separate
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_GROUNDSENSORS_SETMIN)
	{
		return {1, 0, 0};
	}
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_GROUNDSENSORS_SETRANDOM)
	{
		return {0, 1, 0};
	}
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_GROUNDSENSORS_SETMAX)
	{
		return {0, 0, 1};
	}
	// actuator
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_ACTUATOR_LWHEEL_SETHALF)
	{
		return {1, 0, 0};
	}
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_ACTUATOR_RWHEEL_SETHALF)
	{
		return {0, 1, 0};
	}
	else if (cController.FBehavior == ForagingThymioNN::FaultBehavior::FAULT_ACTUATOR_BWHEELS_SETHALF)
	{
		return {0, 0, 1};
	}
	else
	{
		return {1, 1, 1};
	}
}

#endif

void EmptyDescriptor::before_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
}

void EmptyDescriptor::start_trial()
{
}
/*after getting inputs, can update the descriptor if needed*/
void EmptyDescriptor::set_input_descriptor(size_t robot_index, BaseEvolutionLoopFunctions &cLoopFunctions)
{
}

/*end the trial*/
void EmptyDescriptor::end_trial(BaseEvolutionLoopFunctions &cLoopFunctions)
{
}

/*summarise BD at the end of trials*/
std::vector<float> EmptyDescriptor::after_trials(BaseEvolutionLoopFunctions &cLoopFunctions)
{
	return {};
}