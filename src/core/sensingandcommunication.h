#ifndef SENSINGANDCOMMUNICATION_H_
#define SENSINGANDCOMMUNICATION_H_

#include <iostream>
#include <vector>
#include <algorithm>    // std::sort

/****************************************/
/****************************************/
/* ARGoS headers */

/* Definition of the CCI_Controller class. */

/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>

/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>

/* Definition of the differential steering wheel encoder */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_sensor.h>

/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>

/****************************************/
/****************************************/


#define DATA_BYTE_BOUND 240.0f
#define BEACON_SIGNAL 241
#define NEST_BEACON_SIGNAL 242

#define SELF_INFO_PACKET 243 /* used to encompass info of self -- in this case, robot id.*/
#define SELF_INFO_PACKET_FOOTER 244

#define CHAIN_CONNECTOR_PACKET 245
#define CHAIN_CONNECTOR_REQUEST 246
#define CHAIN_CONNECTOR_REQUESTACCEPTED 247
#define CHAIN_CONNECTOR_PACKET_FOOTER 248

#define RELAY_PACKET 249
#define RELAY_PACKET_FOOTER 250

#define VOTER_PACKET 251
#define VOTER_PACKET_FOOTER 252


/****************************************/
/****************************************/
/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;


/* Index to last data entry in RAB actuator data.
*/
extern unsigned m_uRABDataIndex;

/*
 *
 */
void SenseCommunicate(unsigned RobotId, CCI_RangeAndBearingActuator* m_pcRABA, unsigned& DataIndex);

/*
 *
 */
void SendIdToNeighbours(CCI_RangeAndBearingActuator* m_pcRABA, unsigned RobotId, unsigned& DataIndex);

/*
 *
 */
void WriteToCommunicationChannel(CCI_RangeAndBearingActuator* m_pcRABA, unsigned SelfId, unsigned& DataIndex);

#endif
