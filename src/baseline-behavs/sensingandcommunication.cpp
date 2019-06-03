
#include "sensingandcommunication.h"

/****************************************/
/****************************************/

//unsigned m_uRABDataIndex;

/****************************************/
/****************************************/

void SenseCommunicate(unsigned RobotId, CCI_RangeAndBearingActuator *m_pcRABA, unsigned& DataIndex)
{
    unsigned m_uRobotId;

    //m_uRABDataIndex = DataIndex;

    m_uRobotId = RobotId;

    SendIdToNeighbours(m_pcRABA, RobotId, DataIndex);
}

/****************************************/
/****************************************/

void SendIdToNeighbours(CCI_RangeAndBearingActuator *m_pcRABA, unsigned RobotId, unsigned &m_uRABDataIndex)
{
    /*Communicate your id to neighbours, so they know who they are observing*/
    /*Also communicate the bearing at which you observed the neighbours */

    WriteToCommunicationChannel(m_pcRABA, RobotId, m_uRABDataIndex);
}

/****************************************/
/****************************************/

void WriteToCommunicationChannel(CCI_RangeAndBearingActuator *m_pcRABA, unsigned SelfId, unsigned &databyte_index)
{
    m_pcRABA->SetData(databyte_index++, SELF_INFO_PACKET);
    m_pcRABA->SetData(databyte_index++, SelfId);


    if(databyte_index >= m_pcRABA->GetSize()-1)
    {
        std::cerr << " buffer_full in WriteToCommunicationChannel";
        exit(-1);
    }

     m_pcRABA->SetData(databyte_index++, SELF_INFO_PACKET_FOOTER);
}


/****************************************/
/****************************************/

bool func_SortPacketsOnRange (const CCI_RangeAndBearingSensor::SPacket i, const CCI_RangeAndBearingSensor::SPacket j) { return (i.Range < j.Range); }

/****************************************/
/****************************************/
