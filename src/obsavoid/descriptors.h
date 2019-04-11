


// #ifndef DESCRIPTORS
// #define DESCRIPTORS
/* ARGoS related headers */
/* The NN controller */
#include <tuple>
#include <cmath>

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/thymio/simulator/thymio_entity.h>

/****************************************/
/****************************************/
/****************************************/

class CObsAvoidEvolLoopFunctions;
class RunningStat;


class Descriptor{
public:
    Descriptor(){        
    	    	bd.resize(behav_dim);
    }
    size_t num_updates, current_trial;
    static const size_t behav_dim; // for now all the same
    /* final value of bd*/
    std::vector<std::vector<float>> bd;
        /* prepare for trials*/
    virtual void before_trials(CObsAvoidEvolLoopFunctions& cLoopFunctions);
    /*reset BD at the start of a trial*/
    virtual void start_trial();
    /*after getting inputs, can update the descriptor if needed*/
    virtual void set_input_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions& cLoopFunctions){};

    /*after getting outputs, can update the descriptor if needed*/
    virtual void set_output_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions& cLoopFunctions){};




    /*end the trial*/
    virtual void end_trial(CObsAvoidEvolLoopFunctions& cLoopFunctions)
    {
    }

    /*summarise BD at the end of trials*/
    virtual std::vector<float> after_trials(CObsAvoidEvolLoopFunctions& cLoopFunctions);

};


class AverageDescriptor: public Descriptor{
    /* Get the average sensory readings averaged within and between trial
     */
public:
    AverageDescriptor(){

        
    }
    
    /*after getting inputs, can update the descriptor if needed*/
    virtual void set_input_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions& cLoopFunctions);

    /*end the trial*/
    virtual void end_trial(CObsAvoidEvolLoopFunctions& cLoopFunctions);

};


class IntuitiveHistoryDescriptor: public Descriptor{
    /* 
    *  track observation-action or state-action pairs over time 
    *  after all trials, gather statistics of the observed history
    */
public:
    IntuitiveHistoryDescriptor(CLoopFunctions* cLoopFunctions);
    std::map<std::tuple<int, int, int> , size_t> unique_visited_positions;
    //RunningStat* velocity_stats;
    argos::CVector3 center;
    const float grid_step=0.02;
    const float max_velocitysd = 0.50;// with min,max=0,1 -> at most 0.5 deviation on average
    float total_size;
    float max_deviation, deviation;



 
    /*reset BD at the start of a trial*/
    virtual void start_trial();

    /*after getting inputs, can update the descriptor if needed*/
    virtual void set_input_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions& cLoopFunctions);
    /*after getting outputs, can update the descriptor if needed*/
    virtual void set_output_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions& cLoopFunctions);
    /*end the trial*/
    virtual void end_trial(CObsAvoidEvolLoopFunctions& cLoopFunctions);







    std::tuple<int, int, int> get_bin(argos::CVector3 vec)
    {
        int binx= (int) ((float) vec.GetX()/grid_step);
        int biny= (int) ((float) vec.GetY()/grid_step);
        int binz= (int) ((float) vec.GetZ()/grid_step);

       std::tuple<int, int, int> bin(binx,biny,binz);
        // #ifdef PRINTING
        //     std::cout<<"binned "<< vec  <<"into "<<binx<<","<<biny<<","<<binz<<std::endl;
        // #endif
        return bin;

    }
    std::vector<float> get_probs()
    {
        std::vector<float> a;
         for( auto& pair : unique_visited_positions )
        {
            a.push_back(pair.second/(float) num_updates);
            #ifdef PRINTING
                std::cout<<"total visits"<<num_updates<<std::endl;
                std::cout<<"location "<<std::get<0>(pair.first)<<","<<std::get<1>(pair.first)<<","<<std::get<2>(pair.first)<<std::endl;
                std::cout<<"visits " << pair.second<<std::endl;
                std::cout<<"probability " << a.back()<<std::endl;
            #endif

        }

        return a;
    }




};


struct Entity
{
	std::vector<float> attributes;
	CVector3 position;
	Entity(){}
	float& operator[](size_t idx){ return attributes[idx]; }
	static float distance(const Entity e1, const Entity e2);
	void set_attributes(const std::vector<float> new_vec, CVector3 pos)
	{
		attributes=new_vec;
		position=pos;
	}

};



struct Entity_Group
{

	size_t max_size, min_size;
	size_t kappa;// feature_size
	std::vector<Entity> entities;// the attributes of the entities, can be obtained by a key (e.g. location or agent id), makes identifying entities in a group easier


	Entity_Group(){
	}

	Entity_Group(size_t k, size_t M, size_t m, std::vector<Entity> entity_vec) : kappa(k), max_size(M), min_size(m),entities(entity_vec){
	}
	size_t get_absolute_size()
	{
		return entities.size();
	}
	float get_size()
	{
		return ((float) get_absolute_size() - (float) min_size)/((float) max_size - (float) min_size);
	}
	
	float mean_state_vec(size_t feature_index)
	{
		float mean_state=0;
		for (auto& entity: entities)
		{

			mean_state+=entity[feature_index];
		}
		mean_state/=entities.size();
		return mean_state;
	}

	void add_entity(Entity e)
	{
		entities.push_back(e);
	}

	Entity& operator[](size_t idx){ return entities[idx]; }

};





class SDBC: public Descriptor{
    /* 
    *  Systematically Derived Behavioral Characterisation
    */
public:
	size_t bd_index, num_groups;
	std::map<std::string,Entity_Group> entity_groups;
	std::vector<std::string> comparison_groups;// contains the keys of groups with max_size > 1
	std::vector<std::string> variable_groups;// contains the keys of groups with variable sizes
    SDBC(CLoopFunctions* cLoopFunctions, std::string init_type);
    void init_walls_and_robots(CLoopFunctions* cLoopFunctions);
	void init_robots(CLoopFunctions* cLoopFunctions);

    /* group sizes are the first BD dimensions*/
    void add_group_sizes();

    /* group mean attribute vectors, the second set of BD dimensions*/
    void add_group_meanstates();

    /* avg pair-wise distance within groups, the third set of  BD dimensions*/
    void add_within_group_dispersion();

    /* avg pair-wise distance between groups, the final BD dimensions*/
    void add_between_group_dispersion();

	/* prepare for trials*/
	virtual void before_trials(argos::CSimulator& cSimulator);
	/*reset BD at the start of a trial*/
	virtual void start_trial();
	
	/*after getting inputs, can update the descriptor if needed*/
	virtual void set_input_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions& cLoopFunctions);
	
	/*after getting outputs, can update the descriptor if needed*/
	virtual void set_output_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions& cLoopFunctions);
	/*end the trial*/
	virtual void end_trial(CObsAvoidEvolLoopFunctions& cLoopFunctions);



};



// class RNNHistoryDescriptor: public HistoryDescriptor{
//     /* 
//     *  
//     *  use RNN to process observed oa-history
//     */
// public:
//     ModusHistoryDescriptor(){

//     }
//     void set_RNN_from_config()
//     {

//     }
//     static const size_t behav_dim=7;
//     /*reset BD at the start of a trial*/
//     virtual void start_trial()
//     {
//         //bd.resize(ParamsDnn::dnn::nb_inputs + ParamsDnn::dnn::nb_outputs - 1, 0.0f); 
//         bd.resize(ParamsDnn::dnn::nb_inputs - 1, 0.0f); 
//     }
//     /*end the trial*/
//     virtual void end_trial()
//     {
//         rnn->forward();
//     }
//     /*summarise BD at the end of trials*/
//     virtual std::vector<float> after_trials(Real time);
// };

class AutoDescriptor: public Descriptor{
public:
    AutoDescriptor(){}

    /*after getting inputs, can update the descriptor if needed*/
    virtual void set_input_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions& cLoopFunctions){};

    /*after getting outputs, can update the descriptor if needed*/
    virtual void set_output_descriptor(size_t robot_index, CObsAvoidEvolLoopFunctions& cLoopFunctions){};

    /*summarise BD at the end of trial*/
    virtual std::vector<float> after_trials(Real time, CObsAvoidEvolLoopFunctions& cLoopFunctions){};
};
/****************************************/
/****************************************/
/* typedef function pointer, useful for fitness functions */
//typedef void (*functionPtr)();




// #endif