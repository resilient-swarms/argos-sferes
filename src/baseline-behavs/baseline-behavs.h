#ifndef BASELINE_BEHAVS_H
#define BASELINE_BEHAVS_H

/*
 * Include some necessary headers.
 */
#include <argos3/core/utility/math/angles.h>
//#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/logging/argos_log.h>



/****************************************/
/****************************************/
/* Definitions for behaviors used to control robot */

#include "behavior.h"
#include "aggregatebehavior.h"
#include "dispersebehavior.h"
#include "coveragebehavior.h"
#include "bordercoveragebehavior.h"
#include "randomwalkbehavior.h"
#include "homingtofoodbeaconbehavior.h"
#include "circlebehavior.h"
#include "flockingbehavior.h"
#include "src_chain_behavior.h"
#include "link_chain_behavior.h"
#include "dest_chain_behavior.h"


/****************************************/
/****************************************/

/* now include core utilities */
#include <src/core/base_controller.h>

#include <src/core/sensingandcommunication.h>
/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;




/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CBaselineBehavs : public BaseController
{

public:
    using BaseController::SWheelTurningParams;
    using BaseController::FaultBehavior;
    struct ExperimentToRun
    {
        /* The type of experiment to run */
        enum SwarmBehavior
        {
            SWARM_AGGREGATION = 0,
            SWARM_DISPERSION,
            SWARM_HOMING,
            SWARM_COVERAGE,
            SWARM_BORDERCOVERAGE,
            SWARM_FLOCKING,
            SWARM_CHAINING,
            SWARM_STOP,
            SWARM_NONE
        };
        enum SwarmBehavior SBehavior;



        //std::string id_FaultyRobotInSwarm;
        std::string swarmbehav;

        ExperimentToRun();

        void Init(TConfigurationNode& t_node);
    };





    struct RobotDetails
    {
        Real iterations_per_second; /* controlcycles run per second*/
        Real seconds_per_iterations;
        Real HALF_INTERWHEEL_DISTANCE;  // in m
        Real INTERWHEEL_DISTANCE;  // in m
        Real WHEEL_RADIUS;  // in m

        CRadians m_cNoTurnOnAngleThreshold; CRadians m_cSoftTurnOnAngleThreshold;

        Real MaxLinearSpeed; //cm/ (controlcycle)
        Real MaxLinearAcceleration; //cm/controlcycle/controlcycle

        Real MaxAngularSpeed; //rad/controlcycle
        Real MaxAngularAcceleration; //rad/controlcycle/controlcycle;

        RobotDetails()
        {
            iterations_per_second  = 10.0f; /*10 ticks per second so dt=0.01s. i.e., the controlcycle is run 10 times per second*/
            seconds_per_iterations = 1.0f / iterations_per_second;
            HALF_INTERWHEEL_DISTANCE = 0.045f;  // m
            INTERWHEEL_DISTANCE  = 0.09f;  // m
            WHEEL_RADIUS = 0.022f;  // m

            m_cNoTurnOnAngleThreshold   = ToRadians(CDegrees(10.0f)); //10.0 - straight to food spot; 35.0 spiral to food spot
            m_cSoftTurnOnAngleThreshold = ToRadians(CDegrees(70.0f));
        }

        void SetKinematicDetails(Real f_MaxLeftWheelSpeed, Real f_MaxRightWheelSpeed) // arguments are speeds in cm/s
        {
            //std::cout << " f_MaxLeftWheelSpeed " << f_MaxLeftWheelSpeed << std::endl;

            // the max linear speed is 1 cm/controlcycle.
            // so, MaxLinearSpeed = 1
            MaxLinearSpeed        = ((f_MaxLeftWheelSpeed + f_MaxRightWheelSpeed) / 2.0f) * seconds_per_iterations;  //in cm/ (controlcycle)

            //std::cout << " MaxLinearSpeed " << MaxLinearSpeed << std::endl;

            // as the max speed is 1 cm/controlcycle (resultant speed is always positive as the robot does not traverse backwards), the max acceleration is +/-1 cm/controlcycle/controlcycle
            // so, MaxLinearAcceleration = |+/-1 cm/controlcycle/controlcycle| = 1cm/controlcycle/controlcycle
            MaxLinearAcceleration = MaxLinearSpeed;

            // the max angular speed is +/-21.6 degrees/controlcycle,
            // so MaxAngularSpeed = |+/-21.621 degrees/controlcycle| = |21.621 degrees/controlcycle|
            MaxAngularSpeed       = ((f_MaxLeftWheelSpeed + f_MaxRightWheelSpeed) /
                                     (INTERWHEEL_DISTANCE * 100.0f)) * seconds_per_iterations; //rad/controlcycle

            // as the max angular speed is +/-21.6 degrees/controlcycle, the max acceleration is +/-43.2421 radians/controlcycle/controlcycle
            // so MaxAngularAcceleration = |+/-43.2421 degrees/controlcycle/controlcycle| = 43.2421 degrees/controlcycle/controlcycle
            MaxAngularAcceleration   = 2.0f * MaxAngularSpeed; //rad/controlcycle/controlcycle;
        }
    };

    RobotDetails m_sRobotDetails;

    /* Class constructor. */
    CBaselineBehavs();

    /* Class destructor. */
    virtual ~CBaselineBehavs();

    /*
     *
     *
    */
    virtual void CopyRobotDetails(RobotDetails& sRobotDetails);

    /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_diffusion_controller> section.
    */
    virtual void Init(TConfigurationNode& t_node);

    /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
    virtual void ControlStep();

    /*
         * Execute general faults
        */
    virtual void RunGeneralFaults();

    /*
         * Run one of the swarm experiments (Aggregation, Dispersion, Homing)
        */
    virtual void RunHomogeneousSwarmExperiment();


    /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
    virtual void Reset() {m_pcRABA->ClearData();}

    /*
         *  This function returns the interger cast of the string robot id
        */
    virtual unsigned RobotIdStrToInt();

    /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
    virtual void Destroy(){}




    /*
        * Returns the experiment type
        */
    inline ExperimentToRun& GetExperimentType()
    {
        return m_sExpRun;
    }


    Real m_fInternalRobotTimer;
    std::vector <int> beaconrobots_ids;
    unsigned m_uRABDataIndex;

    bool src_robot, dest_robot;

private:


    TBehaviorVector m_vecBehaviors;

    CFlockingBehavior* m_pFlockingBehavior;



    /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><footbot_diffusion_controller> section.
    */

    /* Maximum tolerance for the angle between
    * the robot heading direction and
    * the closest obstacle detected. */
    CDegrees m_cAlpha;
    /* Maximum tolerance for the proximity reading between
    * the robot and the closest obstacle.
    * The proximity reading is 0 when nothing is detected
    * and grows exponentially to 1 when the obstacle is
    * touching the robot.
    */
    Real m_fDelta;
    /* Wheel speed. */
    Real m_fWheelVelocity;
    /* Angle tolerance range to go straight.
    * It is set to [-alpha,alpha]. */
    CRange<CRadians> m_cGoStraightAngleRange;

    /*Information on the experiment to run - the swarm behavior and the faulty behavior*/
    ExperimentToRun m_sExpRun;



    unsigned m_uRobotId;

    UInt64 m_unBorderCoverageStartTime;

};

#endif
