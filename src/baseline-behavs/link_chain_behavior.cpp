#include "link_chain_behavior.h"
#include <argos3/core/utility/datatypes/color.h>

/******************************************************************************/
/******************************************************************************/


CLinkChainBehavior::CLinkChainBehavior(unsigned *ptr_rabdataindex, CCI_RangeAndBearingActuator *RABA, CCI_ThymioLedsActuator *Leds, unsigned RobotId,
                                       bool *BeaconSignalOn,
                                       int *ParentRobotId,
                                       int *ChildRobotId,
                                       int *TimeLastRequest) :
    m_ptrRabDataIndex(ptr_rabdataindex),
    m_pcRABA(RABA),
    m_pcLeds(Leds),
    m_unRobotId(RobotId),
    m_bBeaconSignalOn(BeaconSignalOn),
    m_iParentRobotId(ParentRobotId),
    m_iChildRobotId(ChildRobotId),
    m_iTimeLastRequest(TimeLastRequest)

{
}

/******************************************************************************/
/******************************************************************************/

bool CLinkChainBehavior::TakeControl()
{
    std::cout << "For robot " << m_unRobotId << " in CLinkChainBehavior::TakeControl of robot -- " <<  (*m_iParentRobotId) << " " << (*m_bBeaconSignalOn) << " " << (*m_iTimeLastRequest) << std::endl;

//    if((*m_iTimeLastRequest) != -1)
//        if((m_sSensoryData.m_rTime - (*m_iTimeLastRequest)) < 100)
//            return true;
//        else
//            (*m_iTimeLastRequest) = -1;

    // relinquish control if robot was a link in the chain and now have lost contact with its parent node in the chain
    if((*m_iParentRobotId) != -1)
    {
        for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
            for(size_t j = 0; j <  m_sSensoryData.m_RABSensorData[i].Data.Size(); ++j)
                if(m_sSensoryData.m_RABSensorData[i].Data[j]   == CBehavior::m_sRobotData.SELF_INFO_PACKET_MARKER &&
                        m_sSensoryData.m_RABSensorData[i].Data[j+1] == (*m_iParentRobotId))
                    return true;

        (*m_iParentRobotId) = -1;
        (*m_iChildRobotId)  = -1;
        (*m_bBeaconSignalOn) = false;

        std::cout << "For robot " << m_unRobotId << " in CLinkChainBehavior::TakeControl of robot -- retutning false -1 " <<  (*m_iParentRobotId) << " " << (*m_bBeaconSignalOn) << std::endl;

        m_pcLeds->SetColor(CColor::BLACK);
        return false;
    }

    // retain control if beacon and is still connected to parent node. Action send beacon signal
    if((*m_bBeaconSignalOn))
        return true;

    // get control if not part of chain, but has sensed a beacon signal
    if(!(*m_bBeaconSignalOn) && (*m_iParentRobotId) == -1)
        for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
            if(m_sSensoryData.m_RABSensorData[i].Data[0] == CBehavior::m_sRobotData.BEACON_SIGNAL_MARKER)
                return true;

    std::cout << "For robot " << m_unRobotId << "in CLinkChainBehavior::TakeControl of robot -- retutning control - 2" <<  (*m_iParentRobotId) << " " << (*m_bBeaconSignalOn) << std::endl;

    m_pcLeds->SetColor(CColor::BLACK);
    return false;

}

/******************************************************************************/
/******************************************************************************/

void CLinkChainBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
    // Todo: Set wheel speeds to match your child node's movement with some springness, while avoiding snaping away from your parent
    fLeftWheelSpeed  = 0.0;
    fRightWheelSpeed = 0.0;

    std::cout << "For robot " << m_unRobotId << " CLinkChainBehavior::Action of robot " <<  std::endl;
    std::cout << "*m_iChildRobotId = " << *m_iChildRobotId << std::endl;

    if((*m_bBeaconSignalOn))
    {
        m_pcLeds->SetColor(CColor::RED);

        // Send out the beacon signal
        m_pcRABA->SetData(*m_ptrRabDataIndex, CBehavior::m_sRobotData.BEACON_SIGNAL_MARKER);
        if(*m_ptrRabDataIndex != 0)
            exit(-1);
        std::cout << "For robot " << m_unRobotId << " CLinkChainBehavior emitting beacon signal " << std::endl;
        (*m_ptrRabDataIndex)++;

        // decide if the beacon can be passed on to another robot to form the chain
        Real min_dist_to_beacon = 10000.0;
        int new_beacon_id = -1;
        for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
        {
            // Extract robot id of RAB emitter
            int robot_id = -1;
            for(size_t j = 0; j <  m_sSensoryData.m_RABSensorData[i].Data.Size(); ++j)
                if(m_sSensoryData.m_RABSensorData[i].Data[j] == CBehavior::m_sRobotData.SELF_INFO_PACKET_MARKER)
                    robot_id = m_sSensoryData.m_RABSensorData[i].Data[j+1];

            if(robot_id == -1)
            {
                /*printf("\n Robot id has not been communicated. Exiting ...  ");
                exit(-1);*/
                continue;
            }


            // Check if RAB emitter is making a connection request. If so
            for(size_t j = 0; j <  m_sSensoryData.m_RABSensorData[i].Data.Size(); ++j)
            {
                if(m_sSensoryData.m_RABSensorData[i].Data[j] == CBehavior::m_sRobotData.CHAIN_CONNECTOR_REQUEST_MARKER &&
                        m_sSensoryData.m_RABSensorData[i].Range < min_dist_to_beacon)
                {
                    min_dist_to_beacon = m_sSensoryData.m_RABSensorData[i].Range;
                    new_beacon_id = robot_id;
                    break;
                }
            }
        }

        if(new_beacon_id != -1)
        {
            (*m_bBeaconSignalOn) = false;
            m_pcRABA->SetData((*m_ptrRabDataIndex)++, CBehavior::m_sRobotData.CHAIN_CONNECTOR_PACKET_MARKER);
            m_pcRABA->SetData((*m_ptrRabDataIndex)++, CBehavior::m_sRobotData.CHAIN_CONNECTOR_REQUESTACCEPTED_MARKER);
            m_pcRABA->SetData((*m_ptrRabDataIndex)++, new_beacon_id);
            m_pcRABA->SetData((*m_ptrRabDataIndex)++, CBehavior::m_sRobotData.CHAIN_CONNECTOR_PACKET_FOOTER_MARKER);
            (*m_iChildRobotId) = new_beacon_id;

            std::cout << "For robot " << m_unRobotId << " in CLinkChainBehavior passing beacon signal to robot " << new_beacon_id << std::endl;
        }
    }
    else if(*m_iParentRobotId == -1) // robot is not yet part of chain
    {
        m_pcLeds->SetColor(CColor::BLUE);

        // search for robot emitting beacon signal
        for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
        {
            if(m_sSensoryData.m_RABSensorData[i].Data[0] == CBehavior::m_sRobotData.BEACON_SIGNAL_MARKER)
            {
                std::cout << "For robot " << m_unRobotId << " CLinkChainBehavior::Action of robot -- extracted beacon signal robot" <<  std::endl;

                // first extract robot id of beacon robot
                int robot_id = -1;
                for(size_t j = 1; j <  m_sSensoryData.m_RABSensorData[i].Data.Size(); ++j)
                    if(m_sSensoryData.m_RABSensorData[i].Data[j] == CBehavior::m_sRobotData.SELF_INFO_PACKET_MARKER)
                        robot_id = m_sSensoryData.m_RABSensorData[i].Data[j+1];

                if(robot_id == -1)
                {
                    /*printf("\n Robot id has not been communicated in link-chain behaviour. Exiting ...  ");
                    exit(-1);*/
                    continue;
                }

                // Check for response else make request
                // First, check if beacon robot has accepted a connection request.
                for(size_t j = 0; j <  m_sSensoryData.m_RABSensorData[i].Data.Size(); ++j)
                {
                    if(m_sSensoryData.m_RABSensorData[i].Data[j]   == CBehavior::m_sRobotData.CHAIN_CONNECTOR_REQUESTACCEPTED_MARKER &&
                       m_sSensoryData.m_RABSensorData[i].Data[j+1] == m_unRobotId)
                    {

                        std::cout << "For robot " << m_unRobotId << " CLinkChainBehavior::Action of robot -- beacon signal passed" <<  std::endl;

                        (*m_bBeaconSignalOn) = true;
                        (*m_iParentRobotId) = robot_id;
                        return;
                    }
                }

                // make a request
                std::cout << "For robot " << m_unRobotId << " CLinkChainBehavior::Action of robot -- requesting beacon signal" <<  std::endl;
                m_pcRABA->SetData((*m_ptrRabDataIndex)++, CBehavior::m_sRobotData.CHAIN_CONNECTOR_PACKET_MARKER);
                m_pcRABA->SetData((*m_ptrRabDataIndex)++, CBehavior::m_sRobotData.CHAIN_CONNECTOR_REQUEST_MARKER);
                m_pcRABA->SetData((*m_ptrRabDataIndex)++, CBehavior::m_sRobotData.CHAIN_CONNECTOR_PACKET_FOOTER_MARKER);
                (*m_iTimeLastRequest) = m_sSensoryData.m_rTime;
                std::cout << "For robot " << m_unRobotId << " in CLinkChainBehavior requesting beacon signal " << std::endl;
                return;
            }
        }
    }
    else // robot is part of chain -- but not the last node
        m_pcLeds->SetColor(CColor::GREEN);
}

/******************************************************************************/
/******************************************************************************/

void CLinkChainBehavior::PrintBehaviorIdentity()
{
    std::cout << "Link-chain behaviour taking over";
}

/******************************************************************************/
/******************************************************************************/
