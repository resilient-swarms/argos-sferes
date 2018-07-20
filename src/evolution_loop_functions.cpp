/****************************************/
/****************************************/

#include "evolution_loop_functions.h"

/****************************************/
/****************************************/

CEvolutionLoopFunctions::CEvolutionLoopFunctions() :
    m_unCurrentTrial(0),
    m_vecInitSetup(0), //arg is number of trials
    //m_pcEPuck(NULL),
    //m_pcController(NULL),
    m_pcRNG(NULL)
{
}

/****************************************/
/****************************************/

CEvolutionLoopFunctions::~CEvolutionLoopFunctions()
{
}

/****************************************/
/****************************************/

void CEvolutionLoopFunctions::Init(TConfigurationNode& t_node)
{
    /*
    * Create the random number generator
    */
    m_pcRNG = CRandom::CreateRNG("argos");

    /*
    * Process number of robots in swarm
    */
    try
    {
        GetNodeAttribute(t_node, "robots", m_unNumberRobots);
        m_pcvecEPuck.resize(m_unNumberRobots);
        m_pcvecController.resize(m_unNumberRobots);
        _vecctrlrob.resize(m_unNumberRobots);
    }
    catch(CARGoSException& ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing number of robots", ex);
    }

    for(size_t i = 0; i < m_unNumberRobots; ++i)
    {
        m_pcvecEPuck[i] = new CEPuckEntity(
                    std::string("ep") + std::to_string(i),    // entity id
                    "enn"    // controller id as set in the XML
                    );
        AddEntity(*m_pcvecEPuck[i]);
        m_pcvecController[i] = &dynamic_cast<CEPuckNNController&>(m_pcvecEPuck[i]->GetControllableEntity().GetController());
    }


//    /*
//    * Create the foot-bot and get a reference to its controller
//    */
//    m_pcEPuck = new CEPuckEntity(
//                "eb",    // entity id
//                "enn"    // controller id as set in the XML
//                );
//    AddEntity(*m_pcEPuck);
//    m_pcController = &dynamic_cast<CEPuckNNController&>(m_pcEPuck->GetControllableEntity().GetController());


    /*
    * Create the initial setup for each trial
    * The robot is placed 4.5 meters away from the light
    * (which is in the origin) at angles
    * { PI/12, 2*PI/12, 3*PI/12, 4*PI/12, 5*PI/12 }
    * wrt to the world reference.
    * Also, the rotation of the robot is chosen at random
    * from a uniform distribution.
    */
//    CRadians cOrient;
//    for(size_t i = 0; i < m_vecInitSetup.size(); ++i)
//    {
//        /* Set position */
//        m_vecInitSetup[i].Position.FromSphericalCoords(
//                    4.5f,                                          // distance from origin
//                    CRadians::PI_OVER_TWO,                         // angle with Z axis
//                    static_cast<Real>(i+1) * CRadians::PI / 12.0f // rotation around Z
//                    );
//        /* Set orientation */
//        cOrient = m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE);
//        m_vecInitSetup[i].Orientation.FromEulerAngles(
//                    cOrient,        // rotation around Z
//                    CRadians::ZERO, // rotation around Y
//                    CRadians::ZERO  // rotation around X
//                    );
//    }

    /*
    * Process trial information
    */
    try
    {
        GetNodeAttribute(t_node, "trials", m_unNumberTrials);
        m_vecInitSetup.resize(m_unNumberTrials);
    }
    catch(CARGoSException& ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing number of trials", ex);
    }


    Reset();
}

/****************************************/
/****************************************/

void CEvolutionLoopFunctions::Reset()
{
    for(size_t i = 0; i < m_unNumberRobots; ++i)
    {
        CVector3 Position = CVector3(m_pcRNG->Uniform(CRange<Real>(0.2, 4.8)), m_pcRNG->Uniform(CRange<Real>(0.2, 4.8)), 0.0f);
        CQuaternion Orientation;
        Orientation.FromEulerAngles(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                    CRadians::ZERO,
                                    CRadians::ZERO
                                    );

        while(!MoveEntity(m_pcvecEPuck[i]->GetEmbodiedEntity(),             // move the body of the robot
                   Position,    // to this position
                   Orientation, // with this orientation
                   false                                         // this is not a check, leave the robot there
                   ))
        {
            Position = CVector3(m_pcRNG->Uniform(CRange<Real>(0.2, 4.8)), m_pcRNG->Uniform(CRange<Real>(0.2, 4.8)), 0.0f);
            Orientation.FromEulerAngles(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                        CRadians::ZERO,
                                        CRadians::ZERO
                                        );

        }
    }
//    /*
//    * Move robot to the initial position corresponding to the current trial
//    */

//    /* Set position */
//    m_vecInitSetup[m_unCurrentTrial].Position = CVector3(m_pcRNG->Uniform(CRange<Real>(1.0, 4.0)), m_pcRNG->Uniform(CRange<Real>(1.0, 4.0)), 0.0f);
//    //std::cout << "Pos " << m_vecInitSetup[m_unCurrentTrial].Position << std::endl;


//    /* Set orientation */
//    m_vecInitSetup[m_unCurrentTrial].Orientation.FromEulerAngles(
//                m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),        // rotation around Z
//                CRadians::ZERO, // rotation around Y
//                CRadians::ZERO  // rotation around X
//                );

//    if(!MoveEntity(
//                m_pcEPuck->GetEmbodiedEntity(),             // move the body of the robot
//                m_vecInitSetup[m_unCurrentTrial].Position,    // to this position
//                m_vecInitSetup[m_unCurrentTrial].Orientation, // with this orientation
//                false                                         // this is not a check, leave the robot there
//                ))
//    {
//        LOGERR << "Can't move robot in <"
//               << m_vecInitSetup[m_unCurrentTrial].Position
//               << ">, <"
//               << m_vecInitSetup[m_unCurrentTrial].Orientation
//               << ">"
//               << std::endl;
//    }
}

/****************************************/
/****************************************/

void CEvolutionLoopFunctions::PreStep()
{
    CSpace::TMapPerType& m_cEPuck = GetSpace().GetEntitiesByType("e-puck");

    size_t robotindex = 0;
    for(CSpace::TMapPerType::iterator it = m_cEPuck.begin(); it != m_cEPuck.end(); ++it) //!TODO: Make sure the CSpace::TMapPerType does not change during a simulation (i.e it is not robot-position specific)
    {
        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);
        CEPuckNNController& cController = dynamic_cast<CEPuckNNController&>(cEPuck.GetControllableEntity().GetController());

        assert(cController.m_pcProximity->GetReadings().size() + 1 == Params::dnn::nb_inputs); //proximity sensors + bias  given as input to nn

        inputs.resize(Params::dnn::nb_inputs);

        Real MaxIRSensor = -1.0;
        for(size_t i = 0; i < cController.m_pcProximity->GetReadings().size(); ++i )
        {
            inputs[i] = cController.m_pcProximity->GetReadings()[i];
            MaxIRSensor = Max(MaxIRSensor, (Real) inputs[i]);
            num_senact[i] += (inputs[i] >= 0.1) ? 1.0 : 0.0;
        }
        inputs[Params::dnn::nb_inputs - 1] = +1.0; //Bias input

        _ctrlrob.step(inputs);
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
        for(size_t j = 0; j < _vecctrlrob[robotindex].get_outf().size(); j++)
            if(std::isnan(_vecctrlrob[robotindex].get_outf()[j]))
                outf[j] = 0.0;
            else
                outf[j]=10.0f*(2.0f*_vecctrlrob[robotindex].get_outf()[j]-1.0f); // to put nn values in the interval [-10;10] instead of [0;1]


        cController.m_fLeftSpeed  = outf[0];
        cController.m_fRightSpeed = outf[1];

        // compute linear speed for fitness function
        float s=(fabs(outf[0])+fabs(outf[1]))/20.0; // in [0,1]
        float ds=fabs(outf[0]-outf[1])/20.0; // in [0,1]
        speed+=s;
        lin_speed+=s*(1.0-sqrt(ds));
        num_ds += (ds >= 0.1) ? 1.0 : 0.0;


        // *** To save simulation time, we stop evaluation if the robot is stuck for more than 100 time steps ***
        /*curr_pos   = cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position;
        CRadians c_y, c_x;
        cEPuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(curr_theta, c_y, c_x);

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

        nb_coll += (1.0f - MaxIRSensor);

        if(stop_eval) // set stop_eval to true if you want to stop the evaluation (e.g., if robot is stuck)
        {
            argos::CSimulator::GetInstance().Terminate();
            //std::cout << "Terminate run " << std::endl;
        }
        ++robotindex;
    }
}

void CEvolutionLoopFunctions::PostStep()
{
}

/****************************************/
/****************************************/

//using TemplateCEvolutionLoopFunctions = CEvolutionLoopFunctions<class NN>;
REGISTER_LOOP_FUNCTIONS(CEvolutionLoopFunctions, "evolution_loop_functions")
