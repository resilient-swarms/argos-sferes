#include <src/core/base_loop_functions.h>
#include <src/core/fitness_functions.h>


BaseLoopFunctions::BaseLoopFunctions() : m_unCurrentTrial(0),m_vecInitSetup(0)
{
     m_pcRNG = CRandom::CreateRNG("argos");
}

void BaseLoopFunctions::init_robots()
{
    m_pcvecRobot.resize(m_unNumberRobots);
    
    

    for (size_t i = 0; i < m_unNumberRobots; ++i)
    {
        m_pcvecRobot[i] = new CThymioEntity(
            std::string("th") + std::to_string(i), // entity id
            get_controller_id()   // controller id as set in the XML
        );
        AddEntity(*m_pcvecRobot[i]);
    }
}

CEmbodiedEntity& BaseLoopFunctions::get_embodied_entity(size_t robot)
{
    return m_pcvecRobot[robot]->GetEmbodiedEntity();
}

void BaseLoopFunctions::place_robots()
{
    init_robots();
    curr_pos.resize(m_unNumberRobots);
    curr_theta.resize(m_unNumberRobots);

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

            while (!MoveEntity(get_embodied_entity(m_unRobot), // move the body of the robot
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


void BaseLoopFunctions::Init(TConfigurationNode &t_node)
{
    /*
    * Create the random number generator
    */
    m_pcRNG = CRandom::CreateRNG("argos");

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

    /*
    * Process number of robots in swarm
    */
    try
    {
        GetNodeAttribute(t_node, "robots", m_unNumberRobots);
        
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing number of robots", ex);
    }
    place_robots();

    init_fitfuns(t_node);
        
    /* process outputfolder */
    try
    {
        GetNodeAttribute(t_node, "output_folder", output_folder);
        // TODO: create some statistics files in this folder
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing output_folder", ex);
    }

#ifdef RECORD_FIT
    // std::ios::app is the open mode "append" meaning
    // new data will be written to the end of the file.
    fitness_writer.open(output_folder + "/fitness", std::ios::app);

#endif
}


/* Process fitness function type  */
void BaseLoopFunctions::init_fitfuns(TConfigurationNode &t_node)
{
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
            this->fitfun = new Coverage(s, this);
        }
        else if (s == "TrialCoverage")
        {
            this->fitfun = new TrialCoverage(s, this);
        }
        else if (s == "BorderCoverage")
        {
            this->fitfun = new Coverage(s, this);
        }
        else if (s == "Dispersion")
        {
            this->fitfun = new Dispersion(this);
        }
        // else if (s == "Flocking")
        // {
        //     this->fitfun = new Flocking(this);
        // }
        else
        {
            throw std::runtime_error("fitfuntype " + s + " not found");
        }
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing behaviour descriptor", ex);
    }
}

void BaseLoopFunctions::Reset()
{
    for (size_t m_unRobot = 0; m_unRobot < m_unNumberRobots; ++m_unRobot)
    {
        MoveEntity(get_embodied_entity(m_unRobot),            // move the body of the robot
                   m_vecInitSetup[m_unCurrentTrial][m_unRobot].Position,    // to this position
                   m_vecInitSetup[m_unCurrentTrial][m_unRobot].Orientation, // with this orientation
                   false                                                    // this is not a check, leave the robot there
        );
#ifdef PRINTING
        std::cout << "position after reset " << get_position(m_pcvecRobot[m_unRobot]);
#endif
    }
}


void BaseLoopFunctions::end_trial(Real time)
{
    fitfun->apply(*this, time);
}
void BaseLoopFunctions::print_progress()
{
    int trial = m_unCurrentTrial;
    fitfun->print_progress(trial);
}





void BaseLoopFunctions::before_trials()
{
    m_unCurrentTrial = -1;
}

void BaseLoopFunctions::start_trial(argos::CSimulator &cSimulator)
{
    /* Tell the loop functions to get ready for the i-th trial */
    SetTrial();
    /* Reset the experiment. This internally calls also cLoopFunctions::Reset(). */
    cSimulator.Reset();
    /* take into account the new settings in the fitness functions */
    fitfun->before_trial(*this);
}





/* these methods are not be overridden */
float BaseLoopFunctions::run_all_trials(argos::CSimulator& cSimulator)
{
    before_trials();
    /*
         * Run x trials and take the worst performance as final value.
        */

    for (size_t i = 0; i < m_unNumberTrials; ++i)
    {
        perform_trial(cSimulator);

#ifdef PRINTING

        print_progress();
#endif
    }
    /****************************************/
    /****************************************/
    float fFitness = alltrials_fitness();
    return fFitness;
}


void BaseLoopFunctions::perform_trial(argos::CSimulator &cSimulator)
{
    start_trial(cSimulator);

    /* Run the experiment */
    cSimulator.Execute();
    Real time = (Real)cSimulator.GetMaxSimulationClock();

    end_trial(time);
}
float BaseLoopFunctions::alltrials_fitness()
{
    return fitfun->after_trials();
}