#include <src/core/base_loop_functions.h>

#include <src/core/statistics.h>
#include <src/core/fitness_functions.h>

#include <src/core/environment_generator.h>

#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>
#include <argos3/plugins/robots/thymio/simulator/thymio_entity.h>

#include <argos3/plugins/robots/thymio/simulator/thymio_measures.h>

BaseLoopFunctions::BaseLoopFunctions() : m_unCurrentTrial(0), m_vecInitSetup(0)
{
}

void BaseLoopFunctions::init_robots(TConfigurationNode &t_node)
{

    if (m_unNumberRobots > 0)
    {
        CSpace::TMapPerType &m_cThymio = GetSpace().GetEntitiesByType("Thymio");
        for (CSpace::TMapPerType::iterator it = m_cThymio.begin(); it != m_cThymio.end(); ++it) //!TODO: Make sure the CSpace::TMapPerType does not change during a simulation (i.e it is not robot-position specific)
        {
            m_pcvecRobot.push_back(any_cast<CThymioEntity *>(it->second));
        }
        if (m_unNumberRobots != m_pcvecRobot.size()) // we need to make sure the number of robots distributed in the arena match what is specified by the user in the loop function.
        {
            throw std::runtime_error("\n The number of robots distributed in the arena " + std::to_string(m_unNumberRobots) + " does not match what is specified by the user in the loop function " + std::to_string(m_pcvecRobot.size()));
        }
    }

    if (m_unNumberCylinders > 0)
    {
        CSpace::TMapPerType &cylinders = GetSpace().GetEntitiesByType("cylinder");
        for (CSpace::TMapPerType::iterator it = cylinders.begin(); it != cylinders.end(); ++it) //!TODO: Make sure the CSpace::TMapPerType does not change during a simulation (i.e it is not robot-position specific)
        {
            m_pcvecCylinder.push_back(any_cast<CCylinderEntity *>(it->second));
        }
        if (m_unNumberCylinders != m_pcvecCylinder.size()) // we need to make sure the number of robots distributed in the arena match what is specified by the user in the loop function.
        {
            throw std::runtime_error("\n The number of cylinders distributed in the arena " + std::to_string(m_unNumberRobots) + " does not match what is specified by the user in the loop function " + std::to_string(m_pcvecCylinder.size()));
        }
    }
}

CEmbodiedEntity *BaseLoopFunctions::get_embodied_entity(size_t robot)
{
    return &m_pcvecRobot[robot]->GetEmbodiedEntity();
}

CEmbodiedEntity *BaseLoopFunctions::get_embodied_cylinder(size_t cylinder)
{
    return &m_pcvecCylinder[cylinder]->GetEmbodiedEntity();
}


/* get the controller  */
BaseController *BaseLoopFunctions::get_controller(size_t robot)
{
    return dynamic_cast<BaseController *>(&m_pcvecRobot[robot]->GetControllableEntity().GetController());
}

void BaseLoopFunctions::place_robots()
{
    // set RAB range
    for (int i = 0; i < m_unNumberRobots; ++i)
    {
        // add the RAB range as a parameter to the controller class
        Real max_rab = m_pcvecRobot[i]->GetRABEquippedEntity().GetRange();
        BaseController *ctrl = get_controller(i);
        ctrl->max_rab_range = max_rab*100.0;//convert to cm
    }
    curr_pos.resize(m_unNumberRobots);
    curr_theta.resize(m_unNumberRobots);
    old_pos.resize(m_unNumberRobots);
    old_theta.resize(m_unNumberRobots);

    //m_vecInitSetup.clear();
    CVector3 size = get_arenasize();
    Real minX = 0.50f; // the 0.05m offset accounts for the wall thickness
    Real maxX = size.GetX() - 0.50f;
    Real minY = 0.50f;
    Real maxY = size.GetY() - 0.50;
    m_vecInitSetup.clear();
    for (size_t m_unTrial = 0; m_unTrial < m_unNumberTrials; ++m_unTrial)
    {
        m_vecInitSetup.push_back(std::vector<SInitSetup>(m_unNumberRobots));
        size_t num_tries = 0;
        for (size_t m_unRobot = 0; m_unRobot < m_unNumberRobots; ++m_unRobot)
        {
            CVector3 Position = CVector3(m_pcRNG->Uniform(CRange<Real>(minX, maxX)), m_pcRNG->Uniform(CRange<Real>(minY, maxY)), 0.0f);
#ifdef PRINTING
            std::cout << "Position1 " << Position << " trial " << m_unTrial << " time " << GetSpace().GetSimulationClock() << std::endl;
#endif
            CQuaternion Orientation;
            Orientation.FromEulerAngles(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                        CRadians::ZERO,
                                        CRadians::ZERO);

            while (!get_embodied_entity(m_unRobot)->MoveTo( // move the body of the robot
                Position,                                   // to this position
                Orientation,                                // with this orientation
                false                                       // this is not a check, leave the robot there
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
            // std::cout << Position << std::endl;
            // std::cout << Orientation << std::endl;
        }



        m_vecInitSetupCylinders.push_back(std::vector<SInitSetup>(m_unNumberCylinders));
        for (size_t cylinder =0 ; cylinder < m_unNumberCylinders; ++cylinder)
        {
            CVector3 Position = CVector3(m_pcRNG->Uniform(CRange<Real>(minX, maxX)), m_pcRNG->Uniform(CRange<Real>(minY, maxY)), 0.0f);
#ifdef PRINTING
            std::cout << "Position1 " << Position << " trial " << m_unTrial << " time " << GetSpace().GetSimulationClock() << std::endl;
#endif
            CQuaternion Orientation;
            Orientation.FromEulerAngles(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                        CRadians::ZERO,
                                        CRadians::ZERO);
            CEmbodiedEntity *entity = get_embodied_cylinder(cylinder);
            while (!entity->MoveTo( // move the body of the robot
                Position,                                   // to this position
                Orientation,                                // with this orientation
                false                                       // this is not a check, leave the robot there
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
            m_vecInitSetupCylinders[m_unTrial][cylinder].Position = Position;
            m_vecInitSetupCylinders[m_unTrial][cylinder].Orientation = Orientation;
            // std::cout << Position << std::endl;
            // std::cout << Orientation << std::endl;
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
    //init_generator(t_node);
    init_robots(t_node);
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
/* add additional agents */
void BaseLoopFunctions::create_new_agents()
{
    size_t start = m_pcvecRobot.size();
    for (size_t i = start ; i < m_unNumberRobots; ++i) // initialise the robots
    {
        m_pcvecRobot.push_back(new CThymioEntity("thymio"+std::to_string(i),
                       get_controller_id()+std::to_string(i),
                        m_vecInitSetupCylinders[m_unCurrentTrial][i].Position,    // to this position
                      m_vecInitSetupCylinders[m_unCurrentTrial][i].Orientation,
                       rab_range,
                       rab_data_size));// TODO: construct properly
        this->AddEntity(*m_pcvecRobot[i]);
        //CPhysicsModel *model = &m_pcvecRobot[i]->GetEmbodiedEntity().GetPhysicsModel("dyn2d_0");
        //model->UpdateEntityStatus();
    }
}

/* add additional agents */
void BaseLoopFunctions::create_new_cylinders()
{
    
    CSpace::TMapPerType &cylinders = GetSpace().GetEntitiesByType("cylinder");
    size_t start = cylinders.size();
    CCylinderEntity *cyl = any_cast<CCylinderEntity*>(cylinders.begin()->second);
    for (size_t i = start; i < m_unNumberCylinders; ++i) // initialise the robots
    {
        
        CCylinderEntity* new_c = new CCylinderEntity("c" + std::to_string(i),
                                            m_vecInitSetupCylinders[m_unCurrentTrial][i].Position,    // to this position
                                            m_vecInitSetupCylinders[m_unCurrentTrial][i].Orientation,
                                                cyl->GetEmbodiedEntity().IsMovable(),
                                                cyl->GetRadius(),
                                                cyl->GetHeight(),
                                                cyl->GetMass());
        // CPhysicsModel *model;
        // model = &new_c->GetEmbodiedEntity().GetPhysicsModel("dyn2d_0");
        // model->UpdateEntityStatus();
        this->AddEntity(*new_c);
        m_pcvecCylinder.push_back(new_c);
    }
}

/* remove superfluous agents */
void BaseLoopFunctions::remove_agents(size_t too_much)
{
    for (size_t i = 0 ; i < too_much; ++i)
    {
        m_pcvecRobot.back()->Destroy();
        this->RemoveEntity(*m_pcvecRobot.back());
        m_pcvecRobot.pop_back();
    }
}

void BaseLoopFunctions::remove_cylinders(size_t too_much)
{

   
    for (size_t i = 0; i < too_much; ++i)
    {
        this->RemoveEntity(*m_pcvecCylinder.back());
        m_pcvecCylinder.pop_back();
    }
}
/* adjust the number of agents */
void BaseLoopFunctions::adjust_number_agents()
{
    // first calculate the difference
    int difference = m_pcvecRobot.size() - m_unNumberRobots;

    curr_pos.resize(m_unNumberRobots);   // strictly not necessary as end indexes not reached
    curr_theta.resize(m_unNumberRobots); //  but less confusing + avoid bugs
    old_pos.resize(m_unNumberRobots);
    old_theta.resize(m_unNumberRobots);

    if (difference > 0)
    {
        remove_agents(difference);
    }
    else if (difference < 0)
    {
        create_new_agents();
    }
    else
    {
        return;
    }
}

/* adjust the number of cylinders */
void BaseLoopFunctions::adjust_number_cylinders()
{
    // first calculate the difference
    int difference = m_pcvecCylinder.size() - m_unNumberCylinders;
    if (difference > 0)
    {
        remove_cylinders(difference);
    }
    else if (difference < 0)
    {
        create_new_cylinders();
    }
    else
    {
        return;
    }
}

void BaseLoopFunctions::reset_agent_positions()
{

    for (size_t m_unRobot = 0; m_unRobot < m_unNumberRobots; ++m_unRobot)
    {
        CEmbodiedEntity *entity = get_embodied_entity(m_unRobot);

        CPhysicsModel *model;
        //model = &entity->GetPhysicsModel("dyn2d_0");
        //model->UpdateEntityStatus();
        if ( !entity->MoveTo(
            m_vecInitSetup[m_unCurrentTrial][m_unRobot].Position,    // to this position
            m_vecInitSetup[m_unCurrentTrial][m_unRobot].Orientation, // with this orientation
            false                                                    // this is not a check, leave the robot there
        ))
        {
            // std::cout << "trial" << m_unCurrentTrial << std::endl;
            // std::cout << "robot" << m_unRobot << std::endl;
            // std::cout<<"entity pos "<<entity->GetOriginAnchor().Position << std::endl;
            // std::cout<<"trial pos " <<m_vecInitSetup[m_unCurrentTrial][m_unRobot].Position<<std::endl;

        }
        // std::cout<<"agent "<<m_unRobot<<std::endl;
        // std::cout<<m_vecInitSetup[m_unCurrentTrial][m_unRobot].Position<<std::endl;
        // std::cout<<m_vecInitSetup[m_unCurrentTrial][m_unRobot].Orientation<<std::endl;
        // for (size_t i=0; i < 4; ++i)
        // {
        //     try{
        //         model = &entity->GetPhysicsModel("dyn2d_"+std::to_string(i));
        //         //std::cout<<"Found the entity !"<<std::endl;
        //     }
        //     catch(argos::CARGoSException e){
        //         continue;
        //     }
        // }
        

        old_pos[m_unRobot] = entity->GetOriginAnchor().Position;
        curr_pos[m_unRobot] = old_pos[m_unRobot];
        CRadians zAngle = get_orientation(m_unRobot);
        curr_theta[m_unRobot] = zAngle;
        old_theta[m_unRobot] = zAngle;
    }

    // for (size_t i=0; i < 4; ++i)
    // {
    //     try{
    //         CSimulator::GetInstance().GetPhysicsEngines()[i]->TransferEntities();

    //     }
    //     catch (argos::CARGoSException e){
    //         continue;
    //     }
    // }
}

void BaseLoopFunctions::reset_cylinder_positions()
{
    for (size_t cylinder=0 ; cylinder < m_pcvecCylinder.size(); ++cylinder)
    {
       
        CEmbodiedEntity *entity = get_embodied_cylinder(cylinder);
        CPhysicsModel *model;
        //model = &entity->GetPhysicsModel("dyn2d_0");
        //model->UpdateEntityStatus();
        bool moved = entity->MoveTo(
            m_vecInitSetupCylinders[m_unCurrentTrial][cylinder].Position,    // to this position
            m_vecInitSetupCylinders[m_unCurrentTrial][cylinder].Orientation, // with this orientation
            false                                                            // this is not a check, leave the robot there
        );
        // std::cout<<"cylinder "<<cylinder<<std::endl;
        // std::cout<<m_vecInitSetup[m_unCurrentTrial][cylinder].Position<<std::endl;
        // std::cout<<m_vecInitSetup[m_unCurrentTrial][cylinder].Orientation<<std::endl;

    }
}
/* before the trials start and Reset happens check 
    whether some settings of the config must be changed */
// void BaseLoopFunctions::generate()
// {
    
//     if (generator != NULL)
//     {
//         generator->generate(this);
//     }
//}
void BaseLoopFunctions::Reset()
{

    reset_agent_positions();
    reset_cylinder_positions();
}

void BaseLoopFunctions::PostStep()
{

    fitfun->after_robotloop(*this);
    for (size_t robotindex = 0; robotindex < m_unNumberRobots; ++robotindex)
    {
        old_pos[robotindex] = curr_pos[robotindex];
        old_theta[robotindex] = curr_theta[robotindex];
    }
}

void BaseLoopFunctions::end_trial()
{
    fitfun->apply(*this);
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

    /* take into account the new settings in the fitness functions */
    fitfun->before_trial(*this);
}

/* these methods are not be overridden */
float BaseLoopFunctions::run_all_trials(argos::CSimulator &cSimulator)
{

    before_trials(cSimulator);
    /*
    * Run x trials and take the average performance as final value.
    */

    for (size_t i = 0; i < m_unNumberTrials; ++i)
    {
        perform_trial(cSimulator);
#ifdef COLLISION_STOP
        if(stop_eval)
        {
            return 0.0f;
        }
#endif
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

    end_trial();
}
float BaseLoopFunctions::alltrials_fitness()
{
    return fitfun->after_trials();
}

/* helper functions */

/*get the orientation of the robot */
CRadians BaseLoopFunctions::get_orientation(size_t robot_index)
{
    CVector3 axis;
    CQuaternion quat = get_embodied_entity(robot_index)->GetOriginAnchor().Orientation;
    CRadians zAngle, yAngle, xAngle;
    quat.ToEulerAngles(zAngle, yAngle, xAngle);
    return zAngle;
}

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
        CVector3 displacement = curr_pos[robot_index] - old_pos[robot_index];
        float theta = old_theta[robot_index].GetValue();
        float velocity = displacement.GetX() * std::cos(theta) + displacement.GetY() * std::sin(theta);
        // convert max speed in cm to max_speed in meters
        float max_speed = get_controller(robot_index)->m_sWheelTurningParams.MaxSpeed / 100.0f;
        velocity /= (tick_time * max_speed); //in [-1,1] now
        velocity = 0.5f + 0.5f * velocity;   // in [0,1] now
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
        float B = Thymio_WHEEL_DISTANCE;                                                   // THYMIO's INTERWHEEL DISTANCE (m)
        float maxV = get_controller(robot_index)->m_sWheelTurningParams.MaxSpeed / 100.0f; // max speed (m/s)
        float angle_difference = argos::NormalizedDifference(curr_theta[robot_index], old_theta[robot_index]).GetValue();
        float maxA = 2 * tick_time * maxV / B;         // this comes at 0.25 which agrees with the value when setting V_l=-Max and V_r=+Max
        float turn_velocity = angle_difference / maxA; // in [-1,1]
        turn_velocity = 0.5 + 0.5 * turn_velocity;
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
        CVector3 displacement = curr_pos[robot_index] - old_pos[robot_index];
        float theta = old_theta[robot_index].GetValue();
        float velocity = displacement.GetX() * std::cos(theta) + displacement.GetY() * std::sin(theta);
        // convert max speed in cm to max_speed in meters
        float max_speed = get_controller(robot_index)->m_sWheelTurningParams.MaxSpeed / 100.0f;
        velocity /= (tick_time * max_speed); //in [-1,1] now
        return velocity;
    }
}

/* get the mass */
float BaseLoopFunctions::get_mass(CThymioEntity *robot)
{
    return 1.0f;
}
/* get the centre of mass of the swarm */
argos::CVector3 BaseLoopFunctions::centre_of_mass(const std::vector<CVector3> &positions)
{
    float M = 0.0;
    argos::CVector3 cm = argos::CVector3(0., 0., 0.);
    for (size_t i = 0; i < positions.size(); ++i)
    {
        float mass = 1.0; //get_mass(robot);// mass of 1 is used here, else use this code
        M += mass;
        cm += positions[i]; //mass * pos;// use this code if you implement real mass
    }
    cm /= M;
#ifdef PRINTING
    std::cout << "centre of mass: " << cm << std::endl;
#endif
    return cm;
}
