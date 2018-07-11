/****************************************/
/****************************************/

#include "evolution_loop_functions.h"

/****************************************/
/****************************************/

CEvolutionLoopFunctions::CEvolutionLoopFunctions() :
    m_unCurrentTrial(0),
    m_vecInitSetup(5), //arg is number of trials
    m_pcEPuck(NULL),
    m_pcController(NULL),
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
    * Create the foot-bot and get a reference to its controller
    */
    m_pcEPuck = new CEPuckEntity(
                "eb",    // entity id
                "enn"    // controller id as set in the XML
                );
    AddEntity(*m_pcEPuck);
    m_pcController = &dynamic_cast<CEPuckNNController&>(m_pcEPuck->GetControllableEntity().GetController());

    /*
    * Create the initial setup for each trial
    * The robot is placed 4.5 meters away from the light
    * (which is in the origin) at angles
    * { PI/12, 2*PI/12, 3*PI/12, 4*PI/12, 5*PI/12 }
    * wrt to the world reference.
    * Also, the rotation of the robot is chosen at random
    * from a uniform distribution.
    */
    CRadians cOrient;
    for(size_t i = 0; i < m_vecInitSetup.size(); ++i)
    {
        /* Set position */
        m_vecInitSetup[i].Position.FromSphericalCoords(
                    4.5f,                                          // distance from origin
                    CRadians::PI_OVER_TWO,                         // angle with Z axis
                    static_cast<Real>(i+1) * CRadians::PI / 12.0f // rotation around Z
                    );
        /* Set orientation */
        cOrient = m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE);
        m_vecInitSetup[i].Orientation.FromEulerAngles(
                    cOrient,        // rotation around Z
                    CRadians::ZERO, // rotation around Y
                    CRadians::ZERO  // rotation around X
                    );
    }

    /*
    * Process trial information, if any
    */
    try
    {
        GetNodeAttribute(t_node, "trial", m_unCurrentTrial);
        Reset();
    }
    catch(CARGoSException& ex) {}
}

/****************************************/
/****************************************/

void CEvolutionLoopFunctions::Reset()
{
    /*
    * Move robot to the initial position corresponding to the current trial
    */

    /* Set position */
    m_vecInitSetup[m_unCurrentTrial].Position = CVector3(m_pcRNG->Uniform(CRange<Real>(1.0, 4.0)), m_pcRNG->Uniform(CRange<Real>(1.0, 4.0)), 0.0f);
    //std::cout << "Pos " << m_vecInitSetup[m_unCurrentTrial].Position << std::endl;


    /* Set orientation */
    m_vecInitSetup[m_unCurrentTrial].Orientation.FromEulerAngles(
                m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),        // rotation around Z
                CRadians::ZERO, // rotation around Y
                CRadians::ZERO  // rotation around X
                );

    if(!MoveEntity(
                m_pcEPuck->GetEmbodiedEntity(),             // move the body of the robot
                m_vecInitSetup[m_unCurrentTrial].Position,    // to this position
                m_vecInitSetup[m_unCurrentTrial].Orientation, // with this orientation
                false                                         // this is not a check, leave the robot there
                ))
    {
        LOGERR << "Can't move robot in <"
               << m_vecInitSetup[m_unCurrentTrial].Position
               << ">, <"
               << m_vecInitSetup[m_unCurrentTrial].Orientation
               << ">"
               << std::endl;
    }
}

/****************************************/
/****************************************/

void CEvolutionLoopFunctions::PreStep()
{
    CSpace::TMapPerType& m_cEPuck = GetSpace().GetEntitiesByType("e-puck");

    for(CSpace::TMapPerType::iterator it = m_cEPuck.begin(); it != m_cEPuck.end(); ++it)
    {
        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);
        CEPuckNNController& cController = dynamic_cast<CEPuckNNController&>(cEPuck.GetControllableEntity().GetController());

        assert(cController.m_pcProximity->GetReadings().size() == Params::dnn::nb_inputs);

        inputs.resize(Params::dnn::nb_inputs);
//        inputs[0] = cController.m_pcProximity->GetReadings()[0].Value;;
//        inputs[1] = cController.m_pcProximity->GetReadings()[1].Value;
//        inputs[2] = cController.m_pcProximity->GetReadings()[22].Value;
//        inputs[3] = cController.m_pcProximity->GetReadings()[23].Value;


        Real MaxIRSensor = -1.0;
        for(size_t i = 0; i < Params::dnn::nb_inputs; ++i )
        {
            inputs[i] = cController.m_pcProximity->GetReadings()[i];
            MaxIRSensor = Max(MaxIRSensor, (Real) inputs[i]);
            num_senact[i] += (inputs[i] >= 0.1) ? 1.0 : 0.0;
        }

        _ctrlrob.step(inputs);

        outf.resize(_ctrlrob.get_outf().size());
        assert(_ctrlrob.get_outf().size() == 2);

        for(size_t j = 0; j < _ctrlrob.get_outf().size(); j++)
            if(std::isnan(_ctrlrob.get_outf()[j]))
                outf[j] = 0.0;
            else
                outf[j]=10.0f*(2.0f*_ctrlrob.get_outf()[j]-1.0f); // to put nn values in the interval [-10;10] instead of [0;1]
                //outf[j]=4*(2*_ctrlrob.get_outf()[j]-1); // to put nn values in the interval [-4;4] instead of [0;1]

        //std::cout << "Speed " << outf << std::endl;
        cController.m_fLeftSpeed  = outf[0];
        cController.m_fRightSpeed = outf[1];

        //std::cout << "Left Speed " << cController.m_fLeftSpeed << " Right Speed " << cController.m_fRightSpeed  << std::endl;


        // compute linear speed for fitness function
        //float s=(outf[0]+outf[1])/20.0; // in [-1;1]
        //float ds=fabs(outf[0]-outf[1])/20.0; // in [0;1]
        //speed+=s;
        //lin_speed+=s*(1.0-ds);

        float s=(fabs(outf[0])+fabs(outf[1]))/20.0; // in [0,1]
        float ds=fabs(outf[0]-outf[1])/20.0; // in [0;1]
        speed+=s;
        lin_speed+=s*(1.0-sqrt(ds));
        num_ds += (ds >= 0.1) ? 1.0 : 0.0;


        // *** To save simulation time, we stop evaluation if the robot is stuck for more than 100 time steps ***
        curr_pos   = cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position;
        CRadians c_y, c_x;
        cEPuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(curr_theta, c_y, c_x);


        /*if (((old_pos-curr_pos).Length() <0.005))// &&
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



        //std::cout << cFootBot.GetEmbodiedEntity().IsCollidingWithSomething() << std::endl;

        //std::cout << cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position << std::endl;


        if(stop_eval)
        {
            argos::CSimulator::GetInstance().Terminate();
            //std::cout << "Terminate run " << std::endl;
        }

        //std::cout << argos::CSimulator::GetInstance().GetSpace().GetSimulationClock() << std::endl;
    }
}

void CEvolutionLoopFunctions::PostStep()
{
}

/****************************************/
/****************************************/

//using TemplateCEvolutionLoopFunctions = CEvolutionLoopFunctions<class NN>;
REGISTER_LOOP_FUNCTIONS(CEvolutionLoopFunctions, "evolution_loop_functions")
