#include <src/core/base_loop_functions.h>

#include <src/core/statistics.h>
#include <src/core/fitness_functions.h>

#include <src/core/environment_generator.h>

#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>
#include <argos3/plugins/robots/thymio/simulator/thymio_entity.h>


BaseLoopFunctions::BaseLoopFunctions() : m_unCurrentTrial(0),m_vecInitSetup(0)
{
}

void BaseLoopFunctions::init_robots()
{
    m_pcvecRobot.clear();
    CSpace::TMapPerType &m_cThymio = GetSpace().GetEntitiesByType("Thymio");

    size_t robotindex = 0;
    for (CSpace::TMapPerType::iterator it = m_cThymio.begin(); it != m_cThymio.end(); ++it) //!TODO: Make sure the CSpace::TMapPerType does not change during a simulation (i.e it is not robot-position specific)
    {
        m_pcvecRobot.push_back(any_cast<CThymioEntity *>(it->second));
    }
    if(m_unNumberRobots != m_pcvecRobot.size())// we need to make sure the number of robots distributed in the arena match what is specified by the user in the loop function.
    {
        throw std::runtime_error("\n The number of robots distributed in the arena " +std::to_string(m_unNumberRobots) + " does not match what is specified by the user in the loop function " +std::to_string(m_pcvecRobot.size()));
    }
    
}

CEmbodiedEntity* BaseLoopFunctions::get_embodied_entity(size_t robot)
{
    return &m_pcvecRobot[robot]->GetEmbodiedEntity();
}

/* get the controller  */
BaseController* BaseLoopFunctions::get_controller(size_t robot)
{
    return dynamic_cast<BaseController*>(&m_pcvecRobot[robot]->GetControllableEntity().GetController());
}


void BaseLoopFunctions::place_robots()
{
    init_robots();
    // set RAB range
    for (int i=0; i < m_unNumberRobots; ++i)
    {
        // add the RAB range as a parameter to the controller class
        Real max_rab = m_pcvecRobot[i]->GetRABEquippedEntity().GetRange();
        BaseController* ctrl = get_controller(i);
        ctrl->max_rab_range = max_rab;
    }
    curr_pos.resize(m_unNumberRobots);
    curr_theta.resize(m_unNumberRobots);
    old_pos.resize(m_unNumberRobots);
    old_theta.resize(m_unNumberRobots);

    //m_vecInitSetup.clear();
    CVector3 size = GetSpace().GetArenaSize();
    Real minX = 0.05f; // the 0.05m offset accounts for the wall thickness
    Real maxX = size.GetX() - 0.05f;
    Real minY = 0.05f;
    Real maxY = size.GetY() - 0.05;
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

            while (!get_embodied_entity(m_unRobot)->MoveTo( // move the body of the robot
                               Position,                                     // to this position
                               Orientation,                                  // with this orientation
                               false                                        // this is not a check, leave the robot there
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
}


void BaseLoopFunctions::Init(TConfigurationNode &t_node)
{
    /*
    * Create the random number generator
    */
    m_pcRNG = CRandom::CreateRNG("argos");
    tick_time = CPhysicsEngine::GetSimulationClockTick();

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
            this->fitfun = new Aggregation(this);
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
        else if (s == "DecayCoverage" || s == "DecayBorderCoverage")
        {
            this->fitfun = new DecayCoverage(s, this);
        }
        else if (s == "Dispersion")
        {
            this->fitfun = new Dispersion(this);
        }
        else if (s == "Flocking")
        {
            this->fitfun = new Flocking(this);
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
}


void BaseLoopFunctions::reset_agent_positions()
{
    for (size_t m_unRobot = 0; m_unRobot < m_unNumberRobots; ++m_unRobot)
    {   
        CEmbodiedEntity* entity = get_embodied_entity(m_unRobot);
        CPhysicsModel* model;

        
        bool moved = entity->MoveTo(
                   m_vecInitSetup[m_unCurrentTrial][m_unRobot].Position,    // to this position
                   m_vecInitSetup[m_unCurrentTrial][m_unRobot].Orientation, // with this orientation
                   false                                                    // this is not a check, leave the robot there
        );

        for (size_t i=0; i < 4; ++i)
        {
            try{
                model = &entity->GetPhysicsModel("dyn2d_"+std::to_string(i));
                //std::cout<<"Found the entity !"<<std::endl;
            }
            catch(argos::CARGoSException e){
                continue;
            }
        }
        model->UpdateEntityStatus();

        old_pos[m_unRobot] = entity->GetOriginAnchor().Position;
        curr_pos[m_unRobot] = old_pos[m_unRobot];
        CVector3 axis;
        entity->GetOriginAnchor().Orientation.ToAngleAxis(old_theta[m_unRobot], axis);
        curr_theta[m_unRobot] = old_theta[m_unRobot];
        // old_pos[m_unRobot] = m_vecInitSetup[m_unCurrentTrial][m_unRobot].Position;
        // curr_pos[m_unRobot] = old_pos[m_unRobot];
        // CVector3 axis;
        // m_vecInitSetup[m_unCurrentTrial][m_unRobot].Orientation;
        // curr_theta[m_unRobot] = old_theta[m_unRobot];

        
        // std::cout<<"reset"<<std::endl;
        // std::cout<<"old theta"<<m_unRobot<<":"<<old_theta[m_unRobot]<<std::endl;
        // std::cout<<"curr theta"<<m_unRobot<<":"<<curr_theta[m_unRobot]<<std::endl;
        // CVector3 axis2;
        // CRadians theta;
        // .ToAngleAxis(theta, axis);
        // std::cout<<"comparison with thymio "<<m_unRobot<<":"<<theta<<std::endl;
    }
}
void BaseLoopFunctions::Reset()
{
    //NOTE : replaced code from here to reset_agent_positions to allow multiple physics engines

    
    // for (size_t m_unRobot = 0; m_unRobot < m_unNumberRobots; ++m_unRobot)
    // {
    //     old_pos[m_unRobot] =get_embodied_entity(m_unRobot).GetOriginAnchor().Position;
    //     CVector3 axis;
    //     get_embodied_entity(m_unRobot).GetOriginAnchor().Orientation.ToAngleAxis(old_theta[m_unRobot], axis);
    //     old_theta[ m_unRobot].UnsignedNormalize();
    // }
    
    
    //reset_agent_positions();
}

void BaseLoopFunctions::PostStep()
{
    
    fitfun->after_robotloop(*this);
    for (size_t robotindex=0; robotindex < m_unNumberRobots; ++robotindex)
    {
        old_pos[robotindex] = curr_pos[robotindex];
        old_theta[robotindex] = curr_theta[robotindex];
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





void BaseLoopFunctions::before_trials(argos::CSimulator &cSimulator)
{
    m_unCurrentTrial = -1;
}

void BaseLoopFunctions::start_trial(argos::CSimulator &cSimulator)
{
    /* Tell the loop functions to get ready for the i-th trial */
    SetTrial();


 

    /* Reset the experiment. This internally calls also cLoopFunctions::Reset(). */
    cSimulator.Reset();
    reset_agent_positions();
    // comment this line if you want to run without error
    //reset_agent_positions();

    /* take into account the new settings in the fitness functions */
    fitfun->before_trial(*this);
}





/* these methods are not be overridden */
float BaseLoopFunctions::run_all_trials(argos::CSimulator& cSimulator)
{
    
    before_trials(cSimulator);
    /*
    * Run x trials and take the average performance as final value.
    */


    // /* generate a new environment if necessary */
    // if (generator != NULL)
    // {
    //     generator->generate(cSimulator);
    // }
        

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




/* helper functions */

/* linear speed normalised to [0,1], based on the actual movement rather than wheel speed */
float BaseLoopFunctions::actual_linear_velocity_01(size_t robot_index)
{
    if (curr_pos[robot_index] == old_pos[robot_index])
    {
        return 0.5f; //exactly on the middle of the range (equivalent to V=0)
    }
    else
    {
        // calculate velocity w.r.t. old orientation
        CVector3 displacement =  curr_pos[robot_index] - old_pos[robot_index];
        float theta = old_theta[robot_index].GetValue();
        float velocity = displacement.GetX()*std::cos(theta) + displacement.GetY() * std::sin(theta);
        // convert max speed in cm to max_speed in meters
        float max_speed = get_controller(robot_index)->m_sWheelTurningParams.MaxSpeed/100.0f;
        velocity/=(tick_time * max_speed);//in [-1,1] now
        velocity = 0.5f + 0.5f*velocity;// in [0,1] now
        return velocity;
    }
}
/* turn velocity normalised to [0,1], based on the actual orientations rather than wheel speed*/
float BaseLoopFunctions::actual_turn_velocity_01(size_t robot_index)
{
    if (curr_theta[robot_index] == old_theta[robot_index])
    {
        return 0.5f; //exactly on the middle of the range (equivalent to V=0)
    }
    else
    {
        // need to normalise by the max possible angle change; cf. https://www.argos-sim.info/forum/viewtopic.php?t=79
        // maxV = get_controller(robot_index)->m_sWheelTurningParams.MaxSpeed = (A * B) / (2 * T)
        // --> maxA = 2*T*maxV/B
        float B =0.045;// THYMIO's INTERWHEEL DISTANCE (m) ! ( a little bit lower than actual seems to be needed)
        float maxV = get_controller(robot_index)->m_sWheelTurningParams.MaxSpeed/100.0f;// max speed (m/s)
        float theta_curr = curr_theta[robot_index].GetValue();
        float theta_old = old_theta[robot_index].GetValue();
        float diff = std::abs(theta_curr - theta_old);
        float maxA = 2*tick_time*maxV/B;
        int sign;
        float signed_angle;
        // does not seem to be necessary: even without normalisation radians seems to never go out [0,2Pi]
        // if (diff > maxA)
        // {
        //     // check if we passed the periodicity
        //     // if so, add two pi to the lowest
        //     if(theta_curr<theta_old)
        //     {
        //         theta_curr += M_2PI;
        //     }
        //     else{
        //         theta_old += M_2_PI;
        //     }
        // }
        if (theta_curr < theta_old)
        {
            sign = 1.0;
        }
        else
        {
            sign = -1.0;
        }
        

        // at 2pi radians it seems to go back in direction regardless, alternative is to determine sign of the rotation based on wheel speeds
        // if (get_controller(robot_index)->m_fLeftSpeed < get_controller(robot_index)->m_fRightSpeed)
        // {
        //     sign = 1.0;
        // }
        // else
        // {
        //     sign = -1.0;
        // }
        float turn_velocity = sign*(theta_curr - theta_old)/maxA;// in [-1,1]
        turn_velocity =0.5 + 0.5*turn_velocity;
        return turn_velocity;
    }

}
/* linear velocity normalised to [-1,1]*/
float BaseLoopFunctions::actual_linear_velocity_signed(size_t robot_index)
{
    if (curr_pos[robot_index] == old_pos[robot_index])
    {
        return 0.0f; //exactly on the middle of the range (equivalent to V=0)
    }
    else
    {
        // calculate velocity w.r.t. old orientation
        CVector3 displacement =  curr_pos[robot_index] - old_pos[robot_index];
        float theta = old_theta[robot_index].GetValue();
        float velocity = displacement.GetX()*std::cos(theta) + displacement.GetY() * std::sin(theta);
        // convert max speed in cm to max_speed in meters
        float max_speed = get_controller(robot_index)->m_sWheelTurningParams.MaxSpeed/100.0f;
        velocity/=(tick_time * max_speed );//in [-1,1] now
        return velocity;
    }
}


