#include "src_chain_behavior.h"
#include <argos3/core/utility/datatypes/color.h>


/******************************************************************************/
/******************************************************************************/

bool CSrcChainBehavior::m_bSrcSignalOn = true;
int  CSrcChainBehavior::m_iChildRobotId = -1;

CSrcChainBehavior::CSrcChainBehavior(unsigned *ptr_rabdataindex, CCI_RangeAndBearingActuator *RABA, CCI_ThymioLedsActuator *Leds, unsigned robot_id) :
    m_ptrRabDataIndex(ptr_rabdataindex),
    m_pcRABA(RABA),
    m_pcLeds(Leds),
    m_unRobotId(robot_id)

{
}

/******************************************************************************/
/******************************************************************************/
    
bool CSrcChainBehavior::TakeControl()
{
    return true;
}

/******************************************************************************/
/******************************************************************************/

void CSrcChainBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
    /*
     * The source is always stationary
     */
    fLeftWheelSpeed  = 0.0;
    fRightWheelSpeed = 0.0;

    if(m_bSrcSignalOn)
    {
        m_pcLeds->SetColor(CColor::RED);

        // Send out the beacon signal
        m_pcRABA->SetData(*m_ptrRabDataIndex, CBehavior::m_sRobotData.BEACON_SIGNAL_MARKER);
        (*m_ptrRabDataIndex)++;
        std::cout << "CSrcChainBehavior emitting beacon signal " << std::endl;

        // decide if the beacon can be passed on to another robot to form the chain
        Real min_dist_to_beacon = 10000.0;
        int new_beacon_id = -1;
        for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
        {
            // Extract robot id of requester
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
            m_bSrcSignalOn = false;
            m_pcRABA->SetData((*m_ptrRabDataIndex)++, CBehavior::m_sRobotData.CHAIN_CONNECTOR_PACKET_MARKER);
            m_pcRABA->SetData((*m_ptrRabDataIndex)++, CBehavior::m_sRobotData.CHAIN_CONNECTOR_REQUESTACCEPTED_MARKER);
            m_pcRABA->SetData((*m_ptrRabDataIndex)++, new_beacon_id);
            m_pcRABA->SetData((*m_ptrRabDataIndex)++, CBehavior::m_sRobotData.CHAIN_CONNECTOR_PACKET_FOOTER_MARKER);
            m_iChildRobotId = new_beacon_id;
            std::cout << "CSrcChainBehavior passing beacon to robot " << new_beacon_id << std::endl;

        }
    }
    else
    {
        m_pcLeds->SetColor(CColor::BLUE);
        std::cout << "CSrcChainBehavior not emitting beacon signal " << std::endl;


        // Check if you are still connected to the child robot
        for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
            for(size_t j = 0; j <  m_sSensoryData.m_RABSensorData[i].Data.Size(); ++j)
                if(m_sSensoryData.m_RABSensorData[i].Data[j]   == CBehavior::m_sRobotData.SELF_INFO_PACKET_MARKER &&
                   m_sSensoryData.m_RABSensorData[i].Data[j+1] == m_iChildRobotId)
                    return;

        std::cout << "CSrcChainBehavior resuming emitting beacon signal " << std::endl;

        m_iChildRobotId = -1;
        m_bSrcSignalOn = true;
    }
}

/******************************************************************************/
/******************************************************************************/

void CSrcChainBehavior::PrintBehaviorIdentity()
{
    std::cout << "Src-chain behaviour taking over";
}

/******************************************************************************/
/******************************************************************************/
