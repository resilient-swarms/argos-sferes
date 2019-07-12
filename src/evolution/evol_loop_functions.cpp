/****************************************/
/****************************************/
#include <src/core/statistics.h>
#include <src/evolution/evol_loop_functions.h>
#include <src/core/fitness_functions.h>
#include <src/evolution/descriptors.h>

#include <src/evolution/nn_controller.h>

#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>

/****************************************/
/****************************************/

/****************************************/
/****************************************/

EvolutionLoopFunctions::EvolutionLoopFunctions() : BaseLoopFunctions()

{
    outf.resize(ParamsDnn::dnn::nb_outputs);
}

/****************************************/
/****************************************/

EvolutionLoopFunctions::~EvolutionLoopFunctions()
{
}

/****************************************/
/****************************************/
bool EvolutionLoopFunctions::check_BD_choice(const std::string choice)
{
    if (choice == "history")
    {
        if (BEHAV_DIM != 3 && BEHAV_DIM != 2)
        {
            throw std::runtime_error(choice + " should be 2 or 3-dimensional");
        }
        return true;
    }
    else if (choice == "cvt_mutualinfo")
    {
        if (BEHAV_DIM != 21)
        {
            throw std::runtime_error(choice + " should be 21-dimensional");
        }
        return true;
    }
    else if (choice == "cvt_mutualinfoact")
    {
        if (BEHAV_DIM != 14)
        {
            throw std::runtime_error(choice + " should be 14-dimensional");
        }
        return true;
    }
    else if (choice == "cvt_spirit")
    {
        if (BEHAV_DIM != 400)
        {
            throw std::runtime_error(choice + " should be 400-dimensional");
        }
        return true;
    }
    else
    {
        return true;
    }
}
void EvolutionLoopFunctions::Init(TConfigurationNode &t_node)
{
    BaseLoopFunctions::Init(t_node);
    init_simulation(t_node);
    init_descriptors(t_node);

}

/* get the controller  */
BaseController *EvolutionLoopFunctions::get_controller(size_t robot)
{
    return m_pcvecController[robot];
}
/* Process behavioural descriptor type  */
void EvolutionLoopFunctions::init_descriptors(TConfigurationNode &t_node)
{

    try
    {
        std::string s;
        GetNodeAttribute(t_node, "descriptortype", s);
        if (s == "auto")
        {
            this->descriptor = new AutoDescriptor();
        }
        else if (s == "history")
        {
            this->descriptor = new IntuitiveHistoryDescriptor(this);
        }
        else if (s.find("sdbc") != std::string::npos)
        {
            this->descriptor = new SDBC(this, s);
        }
        else if (s == "average")
        {
            this->descriptor = new AverageDescriptor();
        }
        else if (s == "cvt_trajectory")
        {
            this->descriptor = new CVT_Trajectory(*this, argos::CSimulator::GetInstance().GetMaxSimulationClock());
        }
        else if (s == "cvt_mutualinfo")
        {
            this->descriptor = new CVT_MutualInfo();
        }
        else if (s == "cvt_mutualinfoact")
        {
            this->descriptor = new CVT_MutualInfoAct();
        }
        else if (s == "cvt_spirit")
        {
            this->descriptor = new CVT_Spirit();
        }
        else if (s == "multiagent_spirit")
        {
            this->descriptor = new MultiAgent_Spirit();
        }
        else if (s == "environment_diversity")
        {
            this->descriptor = new EnvironmentDiversity(*this, "experiments/generator", 2);
        }
        else
        {
            throw std::runtime_error("descriptortype " + s + " not found");
        }

        check_BD_choice(s);
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing behaviour descriptor", ex);
    }
}

/* Process initialisation of robots, number of trials, and outputfolder  */
void EvolutionLoopFunctions::init_simulation(TConfigurationNode &t_node)
{
#ifdef CVT
    /* process outputfolder */
    try
    {
        GetNodeAttribute(t_node, "centroids_folder", centroids_folder);
        // TODO: create some statistics files in this folder
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing centroids_folder", ex);
    }
#endif
}
void EvolutionLoopFunctions::init_robots(TConfigurationNode &t_node)
{

    BaseLoopFunctions::init_robots(t_node);
    m_pcvecController.clear();
    for (size_t robotindex = 0; robotindex < m_pcvecRobot.size(); ++robotindex) //!TODO: Make sure the CSpace::TMapPerType does not change during a simulation (i.e it is not robot-position specific)
    {
        m_pcvecController.push_back(&dynamic_cast<CThymioNNController &>(m_pcvecRobot[robotindex]->GetControllableEntity().GetController()));
    }
}


/* add additional agents */
void EvolutionLoopFunctions::create_new_agents()
{
    BaseLoopFunctions::create_new_agents();
    for (size_t robotindex = 0; robotindex < m_unNumberRobots; ++robotindex) //!TODO: Make sure the CSpace::TMapPerType does not change during a simulation (i.e it is not robot-position specific)
    {
        m_pcvecController.push_back(&dynamic_cast<CThymioNNController &>(m_pcvecRobot[robotindex]->GetControllableEntity().GetController()));
        Real max_rab = m_pcvecRobot[robotindex]->GetRABEquippedEntity().GetRange();
        m_pcvecController[robotindex]->max_rab_range = max_rab;
    }
}

/* remove superfluous agents */
void EvolutionLoopFunctions::remove_agents(size_t too_much)
{
    BaseLoopFunctions::remove_agents(too_much);
    for (size_t robotindex = 0; robotindex < too_much; ++robotindex) //!TODO: Make sure the CSpace::TMapPerType does not change during a simulation (i.e it is not robot-position specific)
    {
        m_pcvecController.pop_back();
    }
}
/****************************************/
/****************************************/

/****************************************/
/****************************************/

void EvolutionLoopFunctions::PreStep()
{

//     for (size_t robotindex = 0; robotindex < m_pcvecRobot.size(); ++robotindex) //!TODO: Make sure the CSpace::TMapPerType does not change during a simulation (i.e it is not robot-position specific)
//     {
//         CThymioEntity *cThymio = m_pcvecRobot[robotindex];
//         CThymioNNController *cController = m_pcvecController[robotindex];


//         //TODO uncomment when doing perturbations
//         /* if (cController->b_damagedrobot)
//             cController->damage_actuators(); */

// #ifdef PRINTING
//         std::cout << "current position" << curr_pos[robotindex] << std::endl;
//         std::cout << "old position" << old_pos[robotindex] << std::endl;
//         std::cout << "current orientation" << curr_pos[robotindex] << std::endl;
//         std::cout << "old orientation" << old_pos[robotindex] << std::endl;
// #endif
        
//         if (this->fitfun->quit_on_collision())
//         {
//             stop_eval = cThymio->GetEmbodiedEntity().IsCollidingWithSomething();
//             if (stop_eval) // set stop_eval to true if you want to stop the evaluation (e.g., robot collides or robot is stuck)
//             {
//                 argos::CSimulator::GetInstance().Terminate();
// #ifdef PRINTING
//                 std::cout << "Terminate run permaturely" << std::endl;
// #endif
//             }
//         }
    // }
}
void EvolutionLoopFunctions::PostStep()
{
    for (size_t robotindex = 0; robotindex < m_pcvecRobot.size(); ++robotindex) //!TODO: Make sure the CSpace::TMapPerType does not change during a simulation (i.e it is not robot-position specific)
    {
        CThymioEntity *cThymio = m_pcvecRobot[robotindex];
        // update the position and orientation
        curr_theta[robotindex] = get_orientation(robotindex);
        
        curr_pos[robotindex] = cThymio->GetEmbodiedEntity().GetOriginAnchor().Position;
        outf[0] = m_pcvecController[robotindex]->m_fLeftSpeed;
        outf[1] = m_pcvecController[robotindex]->m_fRightSpeed;
        inputs = m_pcvecController[robotindex]->inputs;
        this->descriptor->set_input_descriptor(robotindex, *this);
        this->descriptor->set_output_descriptor(robotindex, *this);
    }
    descriptor->after_robotloop(*this);
    BaseLoopFunctions::PostStep();
}

void EvolutionLoopFunctions::before_trials(argos::CSimulator &cSimulator)
{
    BaseLoopFunctions::before_trials(cSimulator);
    descriptor->before_trials(*this);
}
void EvolutionLoopFunctions::start_trial(argos::CSimulator &cSimulator)
{

    stop_eval = false;
    descriptor->start_trial();

    BaseLoopFunctions::start_trial(cSimulator);
    /* Configure the controller with the indiv gen */
    //ConfigureFromGenome(ind.nn());
    //_ctrlrob = ind.nn_cpy();
    //_ctrlrob.init(); // a copied nn object needs to be init before use

    for (size_t j = 0; j < m_unNumberRobots; ++j)
       m_pcvecController[j]->nn.init(); // a copied nn object needs to be init before use
}
void EvolutionLoopFunctions::end_trial()
{
    BaseLoopFunctions::end_trial();
    descriptor->end_trial(*this);
}

std::vector<float> EvolutionLoopFunctions::alltrials_descriptor()
{
    return descriptor->after_trials(*this);
}

/* get bin for sensory probabilities  */
size_t EvolutionLoopFunctions::get_sensory_bin(size_t i, size_t num_bins) const
{
    return StatFuns::get_bin(inputs[i], 0.0f, 1.0f, num_bins);
}
/* get bin for sensory probabilities  */
size_t EvolutionLoopFunctions::get_actuator_bin(size_t i, size_t num_bins) const
{
    return StatFuns::get_bin(outf[i], -10.0f, 10.0f, num_bins);
}

/* get activation bin for the activations of each sensory quadrant get joint activation bin for the actuators (used for Spirit)*/
size_t EvolutionLoopFunctions::get_quadrant_bin() const
{
    // quadrant bin e.g. [0,0,0,0] ---> 0  , [0,0,1,0] --> 3
    // assuming quadrants are : left:[0,1],front:[2],right:[3,4],back:[5,6]
    size_t bin = 0;
    for (int i = 0; i <= 1; ++i)
    {
        if (inputs[i] > 0.50)
        {
            bin += 8;
            break;
        }
    }
    if (inputs[2] > 0.50)
    {
        bin += 4;
    }
    for (int i = 3; i <= 4; ++i)
    {
        if (inputs[3] > 0.50)
        {
            bin += 2;
            break;
        }
    }
    for (int i = 5; i <= 6; ++i)
    {
        if (inputs[i] > 0.50)
        {
            bin += 1;
            break;
        }
    }
    return bin;
}
/* get joint activation bin for the actuators (used for Spirit) */
size_t EvolutionLoopFunctions::get_joint_actuator_bin(size_t num_bins) const
{
    // joint bin (e.g. with three bins each): (-10,-10) --> 0  ; (-10,0) --> 1; ... (10,10) --> 9
    // assuming quadrants are : left:[0,1],front:[2],right:[3,4],back:[5,6]
    size_t bin1 = get_actuator_bin(0, num_bins);
    size_t bin2 = get_actuator_bin(1, num_bins);
    return bin1 * num_bins + bin2;
}
/* get bin for the centre of mass of the swarm (used for MultiAgentSpirit) */
size_t EvolutionLoopFunctions::get_CM_bin(size_t num_bins, size_t num_SD_bins)
{   

    CVector3 arena = GetSpace().GetArenaSize();
    CVector3 grid = arena /num_bins;
    argos::CVector3 cm = centre_of_mass(old_pos);
    /* split CM into bins */
    float bin = StatFuns::get_bin(cm.GetX(),0.0f,arena.GetX(),num_bins);// multiplied =1 ( first symbol)
    float multiplier = (float) num_bins;// now multiplier is num_bins
    bin+=multiplier*StatFuns::get_bin(cm.GetY(),0.0f,arena.GetY(),num_bins);

    multiplier *= (float) num_bins;

    /* split SD across positions into bins */
    //for a set of N > 4 data spanning a range of values R, an upper bound on the standard deviation s is given by s = 0.6R
				// with R=1 then s = 0.6
    CVector3 SD_arena = 0.6*arena;
    CVector2 SDs = StatFuns::XY_standard_dev(old_pos);
    float sdx = SDs.GetX();
    bin+=multiplier*StatFuns::get_bin(sdx,0.0f,SD_arena.GetX(),num_SD_bins);

    multiplier *= (float) num_SD_bins;
    float sdy = SDs.GetY();
    bin+=multiplier*StatFuns::get_bin(sdy,0.0f,SD_arena.GetX(),num_SD_bins);
    return bin;
}


/* get bin for the movement of he swarm */
size_t EvolutionLoopFunctions::get_swarmmovement_bin(size_t num_bins, size_t num_SD_bins)
{
    // compute displacement vectors' mean and SD for each dimension
    std::vector<float> displacement_X, displacement_Y;
    for (size_t i=0; i < curr_pos.size(); ++i)
    {
        displacement_X.push_back(curr_pos[i].GetX() - old_pos[i].GetX());
        displacement_Y.push_back(curr_pos[i].GetY() - old_pos[i].GetY());
    }
    float mean_dX = StatFuns::mean(displacement_X);
    float mean_dY = StatFuns::mean(displacement_Y);
    float SD_dx = StatFuns::standard_dev(displacement_X);
    float SD_dy = StatFuns::standard_dev(displacement_Y);

    // now bin them 
    
    /* split CM into bins */
    float max_displacement = get_controller(0)->m_sWheelTurningParams.MaxSpeed*tick_time/(100.0f);
    float bin = StatFuns::get_bin(mean_dX,-max_displacement,max_displacement,num_bins);// multiplied =1 ( first symbol)
    float multiplier = (float) num_bins;// now multiplier is num_bins
    bin+=multiplier*StatFuns::get_bin(mean_dY,-max_displacement,max_displacement,num_bins);

    multiplier *= (float) num_bins;

    /* split SD across positions into bins */
    //for a set of N > 4 data spanning a range of values R, an upper bound on the standard deviation s is given by s = 0.6R
				// with R=1 then s = 0.6
    // multiply by 2 then because range=2*max_displacement
    float max_sd = 1.2*max_displacement;
    bin+=multiplier*StatFuns::get_bin(SD_dx,0.0f,max_sd, num_SD_bins);

    multiplier *= (float) num_SD_bins;
    bin+=multiplier*StatFuns::get_bin(SD_dy,0.0f,max_sd, num_SD_bins);
    return bin;
}

/****************************************/
/****************************************/

//using TemplateEvolutionLoopFunctions = EvolutionLoopFunctions<class NN>;
REGISTER_LOOP_FUNCTIONS(EvolutionLoopFunctions, "evolution_loopfunctions" + std::string(TAG))
