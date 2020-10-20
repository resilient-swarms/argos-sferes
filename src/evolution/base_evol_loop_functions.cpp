/****************************************/
/****************************************/
#include <src/core/statistics.h>
#include <src/evolution/base_evol_loop_functions.h>
#include <src/core/fitness_functions.h>
#include <src/evolution/descriptors.h>

#include <src/evolution/nn_controller.h>

#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>
#ifdef CAFFE_NETS
#include "caffe/sgd_solvers.hpp"
#endif

#define PROXI_SENS_OFFSET

/****************************************/
/****************************************/

/****************************************/
/****************************************/

BaseEvolutionLoopFunctions::BaseEvolutionLoopFunctions() : BaseLoopFunctions()

{
    outf.resize(ParamsDnn::dnn::nb_outputs);
}

/****************************************/
/****************************************/

BaseEvolutionLoopFunctions::~BaseEvolutionLoopFunctions()
{
}

/****************************************/
/****************************************/
bool BaseEvolutionLoopFunctions::check_BD_choice(const std::string choice)
{
    if (choice == "history")
    {
        if (BEHAV_DIM != 3 && BEHAV_DIM != 2)
        {
            throw std::runtime_error(choice + " should be 2 or 3-dimensional");
        }
        return true;
    }
    else if (choice == "neural" || choice == "neuralcycles")
    {
        if (BEHAV_DIM != 2)
        {
            throw std::runtime_error(choice + " should be 2-dimensional");
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
void BaseEvolutionLoopFunctions::Init(TConfigurationNode &t_node)
{
    BaseLoopFunctions::Init(t_node);
    init_simulation(t_node);
    init_descriptors(t_node);
}

/* get the controller  */
BaseController *BaseEvolutionLoopFunctions::get_controller(size_t robot)
{
    return m_pcvecController[robot];
}
/* Process behavioural descriptor type  */
void BaseEvolutionLoopFunctions::init_descriptors(TConfigurationNode &t_node)
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
        else if (s == "neural")
        {
            this->descriptor = new NeuralDescriptor();
        }
        else if (s == "neuralcycles")
        {
            this->descriptor = new NeuralCyclesDescriptor();
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
        else if (s == "cvt_rab_spirit")
        {
            this->descriptor = new CVT_RAB_Spirit();
        }
        else if (s == "cvt_ground_spirit")
        {
            this->descriptor = new CVT_Ground_Spirit();
        }
        else if (s == "multiagent_spirit")
        {
            this->descriptor = new MultiAgent_Spirit();
        }
#ifdef CAFFE_NETS
        else if (s == "cvt_dynamics_nesterov")
        {
            size_t max_num_updates = m_unNumberTrials * (argos::CSimulator::GetInstance().GetMaxSimulationClock() - 1);
            /* process solverparam_file */
            std::string solverparam_file = "src/caffe_nets/DynamicsModel_solver.prototxt";
            this->descriptor = new TransitionDescriptor<caffe::NesterovSolver<float>>(max_num_updates, solverparam_file);
        }
#endif
        else if (s == "environment_diversity")
        {
            // do nothing; wait for initialisation in argosparallelenviron_eval.h
        }
        else if (s == "analysis")
        {
            // wait for manual initialisation in analysis.cpp
        }
#ifdef HETEROGENEOUS
        else if (s == "identification")
        {
            this->descriptor = new IdentificationDescriptor(m_unNumberRobots);
        }
        else if (s == "identification_wheel")
        {
            this->descriptor = new IdentificationWheelDescriptor(m_unNumberRobots);
        }
        else if (s == "perfect_identification")
        {
            this->descriptor = new PerfectIdentificationDescriptor(m_unNumberRobots);
        }
        else if (s == "perfect_identification2")
        {
            this->descriptor = new PerfectIdentificationDescriptor2(m_unNumberRobots);
        }
        else if (s == "perfect_identificationsorted")
        {
            this->descriptor = new PerfectIdentificationDescriptorSorted(m_unNumberRobots);
        }
        else if (s == "random_identification")
        {
            this->descriptor = new RandomIdentificationDescriptor(m_unNumberRobots);
        }
#endif
        else if (s == "empty")
        {
            this->descriptor = new EmptyDescriptor();
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
void BaseEvolutionLoopFunctions::init_simulation(TConfigurationNode &t_node)
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
void BaseEvolutionLoopFunctions::init_robots(TConfigurationNode &t_node)
{

    BaseLoopFunctions::init_robots(t_node);
    m_pcvecController.clear();
    for (size_t robotindex = 0; robotindex < m_pcvecRobot.size(); ++robotindex) //!TODO: Make sure the CSpace::TMapPerType does not change during a simulation (i.e it is not robot-position specific)
    {
        m_pcvecController.push_back(&dynamic_cast<ControllerType &>(m_pcvecRobot[robotindex]->GetControllableEntity().GetController()));
    }
}

/* add additional agents */
void BaseEvolutionLoopFunctions::create_new_agents()
{
    BaseLoopFunctions::create_new_agents();
    for (size_t robotindex = 0; robotindex < m_unNumberRobots; ++robotindex) //!TODO: Make sure the CSpace::TMapPerType does not change during a simulation (i.e it is not robot-position specific)
    {
        m_pcvecController.push_back(&dynamic_cast<ControllerType &>(m_pcvecRobot[robotindex]->GetControllableEntity().GetController()));
        Real max_rab = m_pcvecRobot[robotindex]->GetRABEquippedEntity().GetRange();
        m_pcvecController[robotindex]->max_rab_range = max_rab * 100.0; //convert to cm
    }
}

/* remove superfluous agents */
void BaseEvolutionLoopFunctions::remove_agents(size_t too_much)
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

void BaseEvolutionLoopFunctions::PreStep()
{
    // nothing here for now
}
void BaseEvolutionLoopFunctions::PostStep()
{

    for (size_t robotindex = 0; robotindex < m_pcvecRobot.size(); ++robotindex) //!TODO: Make sure the CSpace::TMapPerType does not change during a simulation (i.e it is not robot-position specific)
    {
        CThymioEntity *cThymio = m_pcvecRobot[robotindex];
#ifdef COLLISION_STOP
        if (cThymio->GetEmbodiedEntity().IsCollidingWithSomething())
        {
            std::cout << "collision stop" << std::endl;
            argos::CSimulator::GetInstance().Terminate();
            stop_eval = true;
            return;
        }
#endif
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

void BaseEvolutionLoopFunctions::before_trials(argos::CSimulator &cSimulator)
{
    BaseLoopFunctions::before_trials(cSimulator);
    descriptor->before_trials(*this);
}

void BaseEvolutionLoopFunctions::start_trial(argos::CSimulator &cSimulator)
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
void BaseEvolutionLoopFunctions::end_trial()
{
    BaseLoopFunctions::end_trial();
    descriptor->end_trial(*this);
}

std::vector<float> BaseEvolutionLoopFunctions::alltrials_descriptor()
{
    if (stop_eval)
    {
        return std::vector<float>(BEHAV_DIM);
    }
    return descriptor->after_trials(*this);
}

/* get bin for sensory probabilities  */
size_t BaseEvolutionLoopFunctions::get_sensory_bin(size_t i, size_t num_bins) const
{
    return StatFuns::get_bin(inputs[i], 0.0f, 1.0f, num_bins);
}
/* get bin for sensory probabilities  */
size_t BaseEvolutionLoopFunctions::get_actuator_bin(size_t i, size_t num_bins) const
{
    return StatFuns::get_bin(outf[i], -10.0f, 10.0f, num_bins);
}

/* get activation bin for the activations of each sensory quadrant get joint activation bin for the actuators (used for Spirit)*/
size_t BaseEvolutionLoopFunctions::get_quadrant_bin() const
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
/* get activation of groups of inputs */
std::vector<float> BaseEvolutionLoopFunctions::get_inputgroup_activations(std::vector<size_t> end_indexes, float thresh, size_t start) const
{
    std::vector<float> activations;
    for (size_t i = 0; i < end_indexes.size(); ++i)
    {
        bool found = false;
        size_t end = end_indexes[i];
        for (size_t j = start; j <= end; ++j)
        {
            if (inputs[j] > thresh)
            {
                found = true;
                break;
            }
        }
        if (found)
        {
            activations.push_back(1.0);
        }
        else
        {
            activations.push_back(0.0);
        }
        start = end + 1;
    }

    return activations;
}

std::vector<float> BaseEvolutionLoopFunctions::get_inputgroup_activations_smaller(std::vector<size_t> end_indexes, float thresh, size_t start) const
{
    std::vector<float> activations;
    for (size_t i = 0; i < end_indexes.size(); ++i)
    {
        bool found = false;
        size_t end = end_indexes[i];
        for (size_t j = start; j <= end; ++j)
        {
            if (inputs[j] < thresh)
            {
                found = true;
                break;
            }
        }
        if (found)
        {
            activations.push_back(1.0);
        }
        else
        {
            activations.push_back(0.0);
        }
        start = end + 1;
    }

    return activations;
}
// /* get activation bin for the activations of each sensory quadrant get joint activation bin for the actuators (used for Spirit)*/
// size_t BaseEvolutionLoopFunctions::get_quadrant_binRAB() const
// {
//     // proximity sensors: front vs back
//     size_t bin = 0;
//     // front:
//     for (int i = 0; i <= 4; ++i)
//     {
//         if (inputs[i] > 0.00)
//         {
//             bin += 32;
//             break;
//         }
//     }
//     //back:
//     for (int i = 5; i <= 6; ++i)
//     {
//         if (inputs[i] > 0.00)
//         {
//             bin += 16;
//             break;
//         }
//     }

//     // now the RAB sensors in quadrants of 90 degrees

//     // 0:90 degrees
//     for (int i = 7; i <= 8; ++i)
//     {
//         if (inputs[i] > 0.00)
//         {
//             bin += 8;
//             break;
//         }
//     }

//     // 90:180 degrees
//     for (int i = 9; i <= 10; ++i)
//     {
//         if (inputs[i] > 0.00)
//         {
//             bin += 4;
//             break;
//         }
//     }

//     // 180:270 degrees
//     for (int i = 11; i <= 12; ++i)
//     {
//         if (inputs[i] > 0.00)
//         {
//             bin += 2;
//             break;
//         }
//     }

//     // 270:360 degrees
//     for (int i = 13; i <= 14; ++i)
//     {
//         if (inputs[i] > 0.00)
//         {
//             bin += 1;
//             break;
//         }
//     }
//     return bin;
// }

/* get activation bin for the activations of each sensory quadrant get joint activation bin for the actuators (used for Spirit)*/
size_t BaseEvolutionLoopFunctions::get_quadrant_binRAB() const
{

    // proximity sensors: (quadrants)
    size_t bin = 0;
    // front left:
    for (int i = 0; i <= 1; ++i)
    {
        if (inputs[i] < 0.00) // note : after normalisation, -1 is maximal sensory firing and 1 is minimal sensory firing, 0.0 represents middle
        {
            //std::cout<<"left proximity"<<std::endl;
            bin += 32;
            break;
        }
    }
    // front center
    if (inputs[2] < 0.00)
    {
        //std::cout<<"front proximity"<<std::endl;
        bin += 16;
    }
    // front right
    for (int i = 3; i <= 4; ++i)
    {
        if (inputs[i] < 0.00)
        {
            //std::cout<<"right proximity"<<std::endl;
            bin += 8;
            break;
        }
    }
    //back:
    for (int i = 5; i <= 6; ++i)
    {
        if (inputs[i] < 0.00)
        {
            //std::cout<<"back proximity"<<std::endl;
            bin += 4;
            break;
        }
    }

    // now the RAB sensors in halfs (180 degrees; front vs back)

    // 0:90 degrees
    bool bin_front = false;
    for (int i = 7; i <= 8; ++i)
    {
        if (inputs[i] < 0.00) // note : after normalisation, -1 is maximal sensory firing and 1 is minimal sensory firing, 0.0 represents middle
        {
            //std::cout<<"RAB [0:90] degrees " <<std::endl;
            bin += 2;
            bin_front = true;
            break;
        }
    }

    // 90:180 degrees
    bool bin_back = false;
    for (int i = 9; i <= 10; ++i)
    {
        if (inputs[i] < 0.00)
        {
            //std::cout<<"RAB [90:180] degrees " <<std::endl;
            bin += 1;
            bin_back = true;
            break;
        }
    }

    // 180:270 degrees
    if (!bin_back)
    {
        for (int i = 11; i <= 12; ++i)
        {

            if (inputs[i] < 0.00)
            {
                //std::cout<<"RAB [180:270] degrees " <<std::endl;
                bin += 1;
                break;
            }
        }
    }

    // 270:360 degrees
    if (!bin_front)
    {
        for (int i = 13; i <= 14; ++i)
        {

            if (inputs[i] < 0.00)
            {
                //std::cout<<"RAB [270:360] degrees "<<std::endl;
                bin += 2;
                break;
            }
        }
    }
    //std::cout<<"bin " << bin<<std::endl;
    return bin;
}

/* get activation bin for the activations of sensors, assuming use proximity+ground sensors */
size_t BaseEvolutionLoopFunctions::get_binGround() const
{
    size_t bin = 0;
    //size_t color = 1; // 0=white; 1=grey; 2=black; grey is most frequent, then white, then black
    // two ground sensors: 1 trit -- grey-white-black; grey on one sensor and white/black on another is set to white/black
    // note: food sources are far from nest, so no need to specify conflict resolution for white vs black sensory readings
    for (int i = 7; i <= 8; ++i)
    {

        if (inputs[i] < -0.70) // note : after normalisation, -1 is black and 1 is white, 0.0 represents grey
        {
            //std::cout<<"left proximity"<<std::endl;
            bin += 16;
            break;
        }
        if (inputs[i] > 0.70) // note : after normalisation, -1 is black and 1 is white, 0.0 represents grey
        {
            //std::cout<<"left proximity"<<std::endl;
            bin += 32;
            break;
        }
    }

    // proximity sensors: (quadrants) -- 4 first bits
    // front left:
    for (int i = 0; i <= 1; ++i)
    {
        if (inputs[i] < 0.00) // note : after normalisation, -1 is maximal sensory firing and 1 is minimal sensory firing, 0.0 represents middle
        {
            //std::cout<<"left proximity"<<std::endl;
            bin += 8;
            break;
        }
    }
    // front center
    if (inputs[2] < 0.00)
    {
        //std::cout<<"front proximity"<<std::endl;
        bin += 4;
    }
    // front right
    for (int i = 3; i <= 4; ++i)
    {
        if (inputs[i] < 0.00)
        {
            //std::cout<<"right proximity"<<std::endl;
            bin += 2;
            break;
        }
    }
    //back:
    for (int i = 5; i <= 6; ++i)
    {
        if (inputs[i] < 0.00)
        {
            //std::cout<<"back proximity"<<std::endl;
            bin += 1;
            break;
        }
    }

    // now the ground sensors

    //std::cout<<"bin " << bin<<std::endl;
    return bin;
}

/* get joint activation bin for the actuators (used for Spirit) */
size_t BaseEvolutionLoopFunctions::get_joint_actuator_bin(size_t num_bins) const
{
    // joint bin (e.g. with three bins each): (-10,-10) --> 0  ; (-10,0) --> 1; ... (10,10) --> 9
    // assuming quadrants are : left:[0,1],front:[2],right:[3,4],back:[5,6]
    size_t bin1 = get_actuator_bin(0, num_bins);
    size_t bin2 = get_actuator_bin(1, num_bins);
    return bin1 * num_bins + bin2;
}
/* get bin for the centre of mass of the swarm (used for MultiAgentSpirit) */
size_t BaseEvolutionLoopFunctions::get_CM_bin(size_t num_bins, size_t num_SD_bins)
{

    CVector3 arena = get_arenasize();
    CVector3 grid = arena / num_bins;
    argos::CVector3 cm = centre_of_mass(old_pos);
    /* split CM into bins */
    float bin = StatFuns::get_bin(cm.GetX(), 0.0f, arena.GetX(), num_bins); // multiplied =1 ( first symbol)
    float multiplier = (float)num_bins;                                     // now multiplier is num_bins
    bin += multiplier * StatFuns::get_bin(cm.GetY(), 0.0f, arena.GetY(), num_bins);

    multiplier *= (float)num_bins;

    /* split SD across positions into bins */
    //for a set of N > 4 data spanning a range of values R, an upper bound on the standard deviation s is given by s = 0.6R
    // with R=1 then s = 0.6
    CVector3 SD_arena = 0.6 * arena;
    CVector2 SDs = StatFuns::XY_standard_dev(old_pos);
    float sdx = SDs.GetX();
    bin += multiplier * StatFuns::get_bin(sdx, 0.0f, SD_arena.GetX(), num_SD_bins);

    multiplier *= (float)num_SD_bins;
    float sdy = SDs.GetY();
    bin += multiplier * StatFuns::get_bin(sdy, 0.0f, SD_arena.GetX(), num_SD_bins);
    return bin;
}

/* get bin for the movement of he swarm */
size_t BaseEvolutionLoopFunctions::get_swarmmovement_bin(size_t num_bins, size_t num_SD_bins)
{
    // compute displacement vectors' mean and SD for each dimension
    std::vector<float> displacement_X, displacement_Y;
    for (size_t i = 0; i < curr_pos.size(); ++i)
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
    float max_displacement = get_controller(0)->m_sWheelTurningParams.MaxSpeed * tick_time / (100.0f);
    float bin = StatFuns::get_bin(mean_dX, -max_displacement, max_displacement, num_bins); // multiplied =1 ( first symbol)
    float multiplier = (float)num_bins;                                                    // now multiplier is num_bins
    bin += multiplier * StatFuns::get_bin(mean_dY, -max_displacement, max_displacement, num_bins);

    multiplier *= (float)num_bins;

    /* split SD across positions into bins */
    //for a set of N > 4 data spanning a range of values R, an upper bound on the standard deviation s is given by s = 0.6R
    // with R=1 then s = 0.6
    // multiply by 2 then because range=2*max_displacement
    float max_sd = 1.2 * max_displacement;
    bin += multiplier * StatFuns::get_bin(SD_dx, 0.0f, max_sd, num_SD_bins);

    multiplier *= (float)num_SD_bins;
    bin += multiplier * StatFuns::get_bin(SD_dy, 0.0f, max_sd, num_SD_bins);
    return bin;
}

void BaseEvolutionLoopFunctions::analyse(float fFitness)
{
    AnalysisDescriptor *descrip = dynamic_cast<AnalysisDescriptor *>(descriptor);
    descrip->analyse_individual(*this, fFitness);
}

/****************************************/
/****************************************/

//using TemplateBaseEvolutionLoopFunctions = BaseEvolutionLoopFunctions<class NN>;
