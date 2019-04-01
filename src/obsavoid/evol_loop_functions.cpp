/****************************************/
/****************************************/

#include <src/obsavoid/evol_loop_functions.h>

/****************************************/
/****************************************/

#define SENSOR_ACTIVATION_THRESHOLD 0.5

/****************************************/
/****************************************/


CObsAvoidEvolLoopFunctions::CObsAvoidEvolLoopFunctions() :
    m_unCurrentTrial(0),
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

void CObsAvoidEvolLoopFunctions::Init(TConfigurationNode& t_node)
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
        m_pcvecRobot.resize(m_unNumberRobots);
        m_pcvecController.resize(m_unNumberRobots);
        _vecctrlrob.resize(m_unNumberRobots);
    }
    catch(CARGoSException& ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing number of robots", ex);
    }

    for(size_t i = 0; i < m_unNumberRobots; ++i)
    {
        m_pcvecRobot[i] = new CThymioEntity(
                    std::string("th") + std::to_string(i),    // entity id
                    "tnn"    // controller id as set in the XML
                    );
        AddEntity(*m_pcvecRobot[i]);
        m_pcvecController[i] = &dynamic_cast<CThymioNNController&>(m_pcvecRobot[i]->GetControllableEntity().GetController());
    }

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

    m_vecInitSetup.clear();
    for(size_t m_unTrial = 0; m_unTrial < m_unNumberTrials; ++m_unTrial)
    {
        m_vecInitSetup.push_back(std::vector<SInitSetup>(m_unNumberRobots));
        for(size_t m_unRobot = 0; m_unRobot < m_unNumberRobots; ++m_unRobot)
        {
            // TODO: Set bounds for positions from configuration file
            CVector3 Position = CVector3(m_pcRNG->Uniform(CRange<Real>(0.2, 4.8)), m_pcRNG->Uniform(CRange<Real>(0.2, 4.8)), 0.0f);
            //std::cout << "Position1 " << Position << " trial " << m_unTrial << " time " << GetSpace().GetSimulationClock() << std::endl;
            CQuaternion Orientation;
            Orientation.FromEulerAngles(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                        CRadians::ZERO,
                                        CRadians::ZERO
                                        );

            while(!MoveEntity(m_pcvecRobot[m_unRobot]->GetEmbodiedEntity(),             // move the body of the robot
                              Position,    // to this position
                              Orientation, // with this orientation
                              false                                         // this is not a check, leave the robot there
                              ))
            {
                Position = CVector3(m_pcRNG->Uniform(CRange<Real>(0.2, 4.8)), m_pcRNG->Uniform(CRange<Real>(0.2, 4.8)), 0.0f);
                //std::cout << "Position2 " << Position << " trial " << m_unTrial << " time " << GetSpace().GetSimulationClock() << std::endl;
                Orientation.FromEulerAngles(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                            CRadians::ZERO,
                                            CRadians::ZERO
                                            );

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
    for(size_t m_unRobot = 0; m_unRobot < m_unNumberRobots; ++m_unRobot)
    {
        MoveEntity(m_pcvecRobot[m_unRobot]->GetEmbodiedEntity(),             // move the body of the robot
                   m_vecInitSetup[m_unCurrentTrial][m_unRobot].Position,    // to this position
                   m_vecInitSetup[m_unCurrentTrial][m_unRobot].Orientation, // with this orientation
                   false                                         // this is not a check, leave the robot there
                   );
    }
}

/****************************************/
/****************************************/

void CObsAvoidEvolLoopFunctions::PreStep()
{
    CSpace::TMapPerType& m_cThymio = GetSpace().GetEntitiesByType("Thymio");

    size_t robotindex = 0;
    for(CSpace::TMapPerType::iterator it = m_cThymio.begin(); it != m_cThymio.end(); ++it) //!TODO: Make sure the CSpace::TMapPerType does not change during a simulation (i.e it is not robot-position specific)
    {
        CThymioEntity& cThymio = *any_cast<CThymioEntity*>(it->second);
        CThymioNNController& cController = dynamic_cast<CThymioNNController&>(cThymio.GetControllableEntity().GetController());

        assert(cController.m_pcProximity->GetReadings().size() + 1 == Params::dnn::nb_inputs); //proximity sensors + bias  given as input to nn

        inputs.resize(ParamsDnn::dnn::nb_inputs);

        Real MaxIRSensor = -1.0;
        for(size_t i = 0; i < cController.m_pcProximity->GetReadings().size(); ++i )
        {
            inputs[i] = cController.m_pcProximity->GetReadings()[i].Value;
            MaxIRSensor = Max(MaxIRSensor, (Real) inputs[i]);
            num_senact[i] += (inputs[i] >= SENSOR_ACTIVATION_THRESHOLD) ? 1.0 : 0.0;
        }
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
        for(size_t j = 0; j < _vecctrlrob[robotindex].get_outf().size(); j++)
            if(std::isnan(_vecctrlrob[robotindex].get_outf()[j]))
                outf[j] = 0.0;
            else
                outf[j]=10.0f*_vecctrlrob[robotindex].get_outf()[j]; // to put nn values in the interval [-10;10] instead of [-1;1]
                //outf[j]=10.0f*(2.0f*_vecctrlrob[robotindex].get_outf()[j]-1.0f); // to put nn values in the interval [-10;10] instead of [0;1]


        cController.m_fLeftSpeed  = outf[0];
        cController.m_fRightSpeed = outf[1];

        // compute linear speed for fitness function
        float s=(fabs(outf[0])+fabs(outf[1]))/20.0; // in [0,1]
        float ds=fabs(outf[0]-outf[1])/20.0; // in [0,1]
        speed+=s;
        lin_speed+=s*(1.0-sqrt(ds));
        num_ds += (ds >= 0.1) ? 1.0 : 0.0;


        // *** To save simulation time, we stop evaluation if the robot is stuck for more than 100 time steps ***
        /*curr_pos   = cThymio.GetEmbodiedEntity().GetOriginAnchor().Position;
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

        nb_coll += (1.0f - MaxIRSensor);

        if(stop_eval) // set stop_eval to true if you want to stop the evaluation (e.g., if robot is stuck)
        {
            argos::CSimulator::GetInstance().Terminate();
            //std::cout << "Terminate run " << std::endl;
        }
        ++robotindex;
    }
}

void CObsAvoidEvolLoopFunctions::PostStep()
{
}

/****************************************/
/****************************************/

//using TemplateCObsAvoidEvolLoopFunctions = CObsAvoidEvolLoopFunctions<class NN>;
REGISTER_LOOP_FUNCTIONS(CObsAvoidEvolLoopFunctions, "obsavoid_evol_loopfunctions")
