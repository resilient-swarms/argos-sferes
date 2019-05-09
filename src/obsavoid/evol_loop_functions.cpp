/****************************************/
/****************************************/
#include <src/obsavoid/statistics.h>
#include <src/obsavoid/evol_loop_functions.h>
#include <src/obsavoid/fitness_functions.h>
#include <src/obsavoid/descriptors.h>

/****************************************/
/****************************************/

/****************************************/
/****************************************/

CObsAvoidEvolLoopFunctions::CObsAvoidEvolLoopFunctions() : m_unCurrentTrial(0),
                                                           m_vecInitSetup(0), //arg is number of trials
                                                           //m_pcEPuck(NULL),
                                                           //m_pcController(NULL),
                                                           m_pcRNG(NULL)

{
}

/****************************************/
/****************************************/

CObsAvoidEvolLoopFunctions::~CObsAvoidEvolLoopFunctions()
{
}

/****************************************/
/****************************************/
bool CObsAvoidEvolLoopFunctions::check_BD_choice(const std::string choice)
{
    if(choice=="history")
    {
        if (BEHAV_DIM != 3 || BEHAV_DIM != 2)
        {
             throw std::runtime_error(choice + " should be 2 or 3-dimensional");
        }
        return true;
    }
    else if (choice == "cvt_mutualinfo")
    {
        if (BEHAV_DIM != 42)
        {
            throw std::runtime_error(choice + " should be 42-dimensional");
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
    else{
        return true;
    }
}
void CObsAvoidEvolLoopFunctions::Init(TConfigurationNode &t_node)
{
    /*
    * Create the random number generator
    */
    m_pcRNG = CRandom::CreateRNG("argos");

    // /* Process bd_dims */
    // try
    // {

    //     GetNodeAttribute(t_node, "bd_dims",this->bd_dims);

    // }
    // catch(CARGoSException& ex)
    // {
    //     THROW_ARGOSEXCEPTION_NESTED("Error initializing behaviour descriptor", ex);
    // }

    /*
    * Process number of robots in swarm
    */
    try
    {
        GetNodeAttribute(t_node, "robots", m_unNumberRobots);
        m_pcvecRobot.resize(m_unNumberRobots);
        m_pcvecController.resize(m_unNumberRobots);
        _vecctrlrob.resize(m_unNumberRobots);
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing number of robots", ex);
    }

    for (size_t i = 0; i < m_unNumberRobots; ++i)
    {
        m_pcvecRobot[i] = new CThymioEntity(
            std::string("th") + std::to_string(i), // entity id
            "tnn"                                  // controller id as set in the XML
        );
        AddEntity(*m_pcvecRobot[i]);
        m_pcvecController[i] = &dynamic_cast<CThymioNNController &>(m_pcvecRobot[i]->GetControllableEntity().GetController());
    }

    /* Process fitness function type  */
    try
    {
        std::string s;
        GetNodeAttribute(t_node, "fitfuntype", s);
        if (s == "FloreanoMondada")
        {
            this->fitfun = new FloreanoMondada();
        }
        else if (s == "MeanSpeed")
        {
            this->fitfun = new MeanSpeed();
        }
        else if (s == "Aggregation")
        {
            this->fitfun = new Aggregation();
            assert(m_unNumberRobots > 1 && "number of robots should be > 1 when choosing Aggregation fitnessfunction");
        }
        else if (s == "Coverage")
        {
            this->fitfun = new Coverage(this);
        }
        else
        {
            throw std::runtime_error("fitfuntype " + s + " not found");
        }
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing behaviour descriptor", ex);
    }

    /* Process behavioural descriptor type  */
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
        else if (s.find("sdbc") == 0)
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

    /*
    * Process trial information
    */
    try
    {
        GetNodeAttribute(t_node, "trials", m_unNumberTrials);
        //m_vecInitSetup.resize(m_unNumberTrials);
        //this->fitfun->fitness_per_trial.resize(m_unNumberTrials);
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing number of trials", ex);
    }

    try
    {
        GetNodeAttribute(t_node, "output_folder", output_folder);
        // TODO: create some statistics files in this folder
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing output_folder", ex);
    }

    //m_vecInitSetup.clear();
    CVector3 size = GetSpace().GetArenaSize();
    Real minX = 0.0;
    Real maxX = size.GetX() - 0.0;
    Real minY = 0.0;
    Real maxY = size.GetY() - 0.0;
    for (size_t m_unTrial = 0; m_unTrial < m_unNumberTrials; ++m_unTrial)
    {
        m_vecInitSetup.push_back(std::vector<SInitSetup>(m_unNumberRobots));
        size_t num_tries = 0;
        for (size_t m_unRobot = 0; m_unRobot < m_unNumberRobots; ++m_unRobot)
        {
            // TODO: Set bounds for positions from configuration file
            CVector3 Position = CVector3(m_pcRNG->Uniform(CRange<Real>(minX, maxX)), m_pcRNG->Uniform(CRange<Real>(minY, maxY)), 0.0f);
#ifdef PRINTING
            std::cout << "Position1 " << Position << " trial " << m_unTrial << " time " << GetSpace().GetSimulationClock() << std::endl;
#endif
            CQuaternion Orientation;
            Orientation.FromEulerAngles(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                        CRadians::ZERO,
                                        CRadians::ZERO);
   
            while (!MoveEntity(m_pcvecRobot[m_unRobot]->GetEmbodiedEntity(), // move the body of the robot
                               Position,                                     // to this position
                               Orientation,                                  // with this orientation
                               false                                         // this is not a check, leave the robot there
                               ))
            {
                Position = CVector3(m_pcRNG->Uniform(CRange<Real>(minX, maxX)), m_pcRNG->Uniform(CRange<Real>(minY, maxY)), 0.0f);
                //std::cout << "Position2 " << Position << " trial " << m_unTrial << " time " << GetSpace().GetSimulationClock() << std::endl;
                Orientation.FromEulerAngles(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                            CRadians::ZERO,
                                            CRadians::ZERO);
                if (num_tries > 10000)
                {
                    throw std::runtime_error("failed to initialise robot positions; too many obstacles?");
                }
                ++num_tries;
            }
            m_vecInitSetup[m_unTrial][m_unRobot].Position = Position;
            m_vecInitSetup[m_unTrial][m_unRobot].Orientation = Orientation;
        }
    }

    Reset();
}

/****************************************/
/****************************************/

void CObsAvoidEvolLoopFunctions::Reset()
{
    for (size_t m_unRobot = 0; m_unRobot < m_unNumberRobots; ++m_unRobot)
    {
        MoveEntity(m_pcvecRobot[m_unRobot]->GetEmbodiedEntity(),            // move the body of the robot
                   m_vecInitSetup[m_unCurrentTrial][m_unRobot].Position,    // to this position
                   m_vecInitSetup[m_unCurrentTrial][m_unRobot].Orientation, // with this orientation
                   false                                                    // this is not a check, leave the robot there
        );
#ifdef PRINTING
        std::cout << "position after reset " << get_position(m_pcvecRobot[m_unRobot]);
#endif
    }
}

/****************************************/
/****************************************/

void CObsAvoidEvolLoopFunctions::PreStep()
{
    CSpace::TMapPerType &m_cThymio = GetSpace().GetEntitiesByType("Thymio");

    size_t robotindex = 0;
    for (CSpace::TMapPerType::iterator it = m_cThymio.begin(); it != m_cThymio.end(); ++it) //!TODO: Make sure the CSpace::TMapPerType does not change during a simulation (i.e it is not robot-position specific)
    {
        CThymioEntity &cThymio = *any_cast<CThymioEntity *>(it->second);
        CThymioNNController &cController = dynamic_cast<CThymioNNController &>(cThymio.GetControllableEntity().GetController());

        //assert(cController.m_pcProximity->GetReadings().size() + 1 == Params::dnn::nb_inputs); //proximity sensors + bias  given as input to nn

        inputs.resize(ParamsDnn::dnn::nb_inputs);
        maxIRSensor = -1.0;
        for (size_t i = 0; i < cController.m_pcProximity->GetReadings().size(); ++i)
        {
            inputs[i] = cController.m_pcProximity->GetReadings()[i].Value;
            maxIRSensor = Max(maxIRSensor, (Real)inputs[i]);
        }
        this->descriptor->set_input_descriptor(robotindex, *this);
        inputs[ParamsDnn::dnn::nb_inputs - 1] = +1.0; //Bias input

        //      _ctrlrob.step(inputs);
        _vecctrlrob[robotindex].step(inputs);
        _vecctrlrob[robotindex].get_outf();

        //        outf.resize(_ctrlrob.get_outf().size());
        //        assert(_ctrlrob.get_outf().size() == 2);

        //        for(size_t j = 0; j < _ctrlrob.get_outf().size(); j++)
        //            if(std::isnan(_ctrlrob.get_outf()[j]))
        //                outf[j] = 0.0;
        //            else
        //                outf[j]=10.0f*(2.0f*_ctrlrob.get_outf()[j]-1.0f); // to put nn values in the interval [-10;10] instead of [0;1]

        outf.resize(_vecctrlrob[robotindex].get_outf().size());
        assert(_vecctrlrob[robotindex].get_outf().size() == 2);
        for (size_t j = 0; j < _vecctrlrob[robotindex].get_outf().size(); j++)
            if (std::isnan(_vecctrlrob[robotindex].get_outf()[j]))
                outf[j] = 0.0;
            else
                outf[j] = 10.0f * _vecctrlrob[robotindex].get_outf()[j]; // to put nn values in the interval [-10;10] instead of [-1;1]
                                                                         //outf[j]=10.0f*(2.0f*_vecctrlrob[robotindex].get_outf()[j]-1.0f); // to put nn values in the interval [-10;10] instead of [0;1]

        cController.m_fLeftSpeed = outf[0];
        cController.m_fRightSpeed = outf[1];

        CVector3 axis;
        cThymio.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToAngleAxis(curr_theta, axis);

        curr_pos = cThymio.GetEmbodiedEntity().GetOriginAnchor().Position;
        // #ifdef PRINTING
        //         std::cout << "theta=" << curr_theta << std::endl;
        // #endif
        // *** To save simulation time, we stop evaluation if the robot is stuck for more than 100 time steps ***
        /*
        CRadians c_y, c_x;
        cThymio.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(curr_theta, c_y, c_x);

        if (((old_pos-curr_pos).Length() <0.005))// &&
             //(fabs(old_theta.GetValue()-curr_theta.GetValue())<0.0001))
        {
            stand_still++;
            if (stand_still>100)
            {
                stop_eval=true;
                // We add collisions to be fair and avoid side effects
                if (cFootBot.GetEmbodiedEntity().IsCollidingWithSomething())
                    nb_coll += argos::CSimulator::GetInstance().GetMaxSimulationClock() - argos::CSimulator::GetInstance().GetSpace().GetSimulationClock();
            }
        }
        else
        {
            if (cFootBot.GetEmbodiedEntity().IsCollidingWithSomething())
            {
                nb_coll++;
            }
        }

        old_pos     = curr_pos;
        old_theta   = curr_theta;*/

        old_pos = curr_pos;
        old_theta = curr_theta;
        this->descriptor->set_output_descriptor(robotindex, *this);
        this->fitfun->after_step(robotindex, *this);

        stop_eval = cThymio.GetEmbodiedEntity().IsCollidingWithSomething();
        if (stop_eval) // set stop_eval to true if you want to stop the evaluation (e.g., robot collides or robot is stuck)
        {
            argos::CSimulator::GetInstance().Terminate();
#ifdef PRINTING
            std::cout << "Terminate run permaturely" << std::endl;
#endif
        }
        ++robotindex;
    }
    this->descriptor->after_robotloop(*this);
}

void CObsAvoidEvolLoopFunctions::PostStep()
{
}

void CObsAvoidEvolLoopFunctions::before_trials()
{
    m_unCurrentTrial = -1;
    descriptor->before_trials(*this);
}
void CObsAvoidEvolLoopFunctions::start_trial(CSimulator &cSimulator)
{

    stop_eval = false;
    // stand_still = 0;
    // old_pos   = CVector3(0.0f, 0.0f, 0.0f);
    // old_theta = CRadians(0.0f);
    descriptor->start_trial();
    /* Tell the loop functions to get ready for the i-th trial */
    SetTrial();
    /* Reset the experiment. This internally calls also cLoopFunctions::Reset(). */
    cSimulator.Reset();

    /* Configure the controller with the indiv gen */
    //ConfigureFromGenome(ind.nn());
    //_ctrlrob = ind.nn_cpy();
    //_ctrlrob.init(); // a copied nn object needs to be init before use

    for (size_t j = 0; j < m_unNumberRobots; ++j)
        _vecctrlrob[j].init(); // a copied nn object needs to be init before use
}
void CObsAvoidEvolLoopFunctions::end_trial(Real time)
{
    fitfun->apply(*this, time);
    descriptor->end_trial(*this);
}

void CObsAvoidEvolLoopFunctions::print_progress()
{
    int trial = m_unCurrentTrial;
    fitfun->print_progress(trial);
}

float CObsAvoidEvolLoopFunctions::alltrials_fitness()
{
    return fitfun->after_trials();
}
std::vector<float> CObsAvoidEvolLoopFunctions::alltrials_descriptor()
{
    return descriptor->after_trials(*this);
}

/* get bin for sensory probabilities  */
size_t CObsAvoidEvolLoopFunctions::get_sensory_bin(size_t i, size_t num_bins) const
{
    return StatFuns::get_bin(inputs[i], 0.0f, 1.0f, num_bins);
}
/* get bin for sensory probabilities  */
size_t CObsAvoidEvolLoopFunctions::get_actuator_bin(size_t i, size_t num_bins) const
{
    return StatFuns::get_bin(outf[i], -10.0f, 10.0f, num_bins);
}

/* get activation bin for the activations of each sensory quadrant */
size_t CObsAvoidEvolLoopFunctions::get_quadrant_bin() const
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
/* get joint activation bin for the actuators */
size_t CObsAvoidEvolLoopFunctions::get_joint_actuator_bin(size_t num_bins) const
{
    // joint bin (e.g. with three bins each): (-10,-10) --> 0  ; (-10,0) --> 1; ... (10,10) --> 9
    // assuming quadrants are : left:[0,1],front:[2],right:[3,4],back:[5,6]
    size_t bin1=get_actuator_bin(0,num_bins);
    size_t bin2=get_actuator_bin(1,num_bins);
    return bin1*num_bins + bin2;
}



/****************************************/
/****************************************/

//using TemplateCObsAvoidEvolLoopFunctions = CObsAvoidEvolLoopFunctions<class NN>;
REGISTER_LOOP_FUNCTIONS(CObsAvoidEvolLoopFunctions, "obsavoid_evol_loopfunctions")
