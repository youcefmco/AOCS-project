/******************************************************************************
 * Copyright (c) 2013 Surrey Satellite Technology, Ltd.
 * All rights reserved.
 *
 ******************************************************************************
 * Documentation :
 *
 ******************************************************************************
 * SSTL DISCLAIMER   :
 *
 * This C code is furnished under a license and may be used and
 * copied only in accordance with the terms of such license and
 * with the inclusion of the above copyright notice.  This
 * C or any other copies thereof may not be provided or
 * otherwise made available to any other person. No title to or
 * ownership of the software is hereby transferred.
 *
 ******************************************************************************
 * CVS VERSION CONTROL (do not edit)
 *
 * Last Update : $Date: 2013/09/03 13:35:21 $
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/include/AOCS_AocsShell.h,v $
 * Revision    : $Revision: 1.6 $
 *
 * History:
 *
 * $Log: AOCS_AocsShell.h,v $
 * Revision 1.6  2013/09/03 13:35:21  ytrichakis
 * Removed calls to logging library as now all logs are done in the internel ADYYMMDD file
 *
 * Revision 1.5  2013/07/18 15:56:52  ytrichakis
 * fixed wheel bug(DR#14548) and applied the same solution in case of failed AIM
 *
 * Revision 1.4  2013/06/21 13:54:38  ytrichakis
 * Send command 5 only to non-failed AIMs. Also set ewod closing tie to 1 day
 *
 * Revision 1.3  2013/05/24 13:03:17  ytrichakis
 * Fixed AIM0 problem and added TC return handling from SKED RTP
 *
 * Revision 1.2  2013/04/23 15:49:21  ytrichakis
 * Added block wait till SKED msgQ is ready to open
 *
 * Revision 1.1  2013/04/11 13:42:58  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/

#ifndef __AOCS_H_
#define __AOCS_H_

/*---------------------------------------------------------------------------
 * Includes
 */
#include <mqueue.h>
#include "GDEF_GlobDefs.h"
#include "Adcs_IntDefs.h"

/*---------------------------------------------------------------------------
 * Defines and Macros (shared within this module)
 */

#define AOCS_PROCESS_DISPATCHER_THREAD_NAME        "AOCS-DIS"                 /*!< Memory Dump Dispatcher Thread Name                        */
#define AOCS_PROCESS_CAN_THREAD_NAME               "AOCS-CAN"                 /*!< Memory Dump CAN Thread Name                               */

#define PROCESS_NAME                               "AOCS"         		      /*!< Process Name                                              */
#define LOGNAME                                    "AD000000"               	/*!< AOCS Log Filename Prefix for AOCS logging                 */
#define LOG                                        "AS000000"               	/*!< AOCS Log Filename Prefix for general logging              */
#define MESSAGE_QUEUE_PREFIX                       "/AOCS"                    /*!< CAN Message Queue Prefix  */
#define DISPATCHER_MESSAGE_QUEUE                   "_DispatcherMessageQueue"  /* !!!!! May have to put this somewhere more Global !!!!       */
#define DISPATCHER_QUEUE_NAME_SIZE                 (0x32)                     /*!< Dispatcher Queue Name Size                                */

#define AOCS_MESSAGE_QUEUE_SIZE                    (0x32)                     /*!< AOCS Message Queue Size                                   */

#define DEFAULT_TELEMETRY_REQUEST_TIMEOUT          (5000)                	   /*!< Default Telemetry Request Timeout                         */
#define DEFAULT_TELECOMMAND_REQUEST_TIMEOUT        (1000)                	   /*!< Default Telecommand Timeout                               */
#define DEFAULT_AOCS_TC_RETRY_COUNT                (3)                   	   /*!< Initial cmd + this def retries for TC's sent on CAN BUS   */

#define NSEC_TO_SEC                                (1000000000)
#define FIRE_CMDS_DELAY                            (200000000)

#define TEST_ON_OBC
#define TEST_ON_DEBUG1

/*---------------------------------------------------------------------------
 * Typedefs (shared within this module)
 */
typedef enum
{
   AOCS_PROCESS_MINIMUM_CAN_QUEUE_IDENTIFIER        = -1,                     /*!< AOCS Process CAN Minimum Queue Identifier                 */
   AOCS_PROCESS_CAN_RECEIVE_QUEUE_IDENTIFIER        =  0,                     /*!< AOCS Process CAN Receive Queue Identifier                 */
   AOCS_PROCESS_CAN_QUEUE_IDENTIFIER                =  1,                     /*!< AOCS Process Telecommand Queue Identifier                 */
   AOCS_PROCESS_SKED_QUEUE_IDENTIFIER               =  2,                     /*!< AOCS Process SKED Queue Identifier                        */
   AOCS_PROCESS_MAXIMUM_CAN_QUEUE_IDENTIFIER        =  3                      /*!< AOCS Process CAN Maximum Queue Identifier                 */
}te_aocs_can_registration_queue_identifier;

typedef enum
{
   MINIMUM_LOG_FILE_IDENTIFIER = -1,     /*!< Maximum Log File Identifier              */
   AOCS_PROCESS_LOG_FILE       =  0,     /*!< AOCS Process Log File Identifier         */
   MAXIMUM_LOG_FILE_IDENTIFIER =  1      /*!< Maximum Log File Identifier              */
}te_log_file_identifier;

typedef struct
{
   tGDEF_UINT16 chan;
   tGDEF_UINT16 len;
} tsADLK_CHAN_LOOKUP;

/* Not needed TODO */
typedef struct
{
   tGDEF_UINT32 SensorFrames;
   tGDEF_UINT32 ThrusterCmd;
   tGDEF_UINT32 StatusChannel;
   tGDEF_UINT32 StatusChannel2;
   tGDEF_UINT32 LostTmCounter;
} tsAOCS_SHELL_STATUS;

typedef struct
{
   tGDEF_UINT16 chan;
   tGDEF_UINT8  TF;
   tGDEF_UINT8  TS;
   tGDEF_UCHAR  *pData;
   tGDEF_UINT8  len;
} tsCAN_TLM_DATA;

/*---------------------------------------------------------------------------
 * Data - (shared within this module)
 */
extern mqd_t sked_communications_channel;

/*---------------------------------------------------------------------------
 * Module Function Prototypes
 */
#if 0
/*!
 *  \brief Routine to the CAN API Registration Structure
 *  \param  void           - None
 *  \return tsCANS_RegRtn* - Pointer to the CAN API Registration Structure
 */
extern tsCANS_RegRtn *getCANReg(void);

/*!
 *  \brief Routine to return the Log File Configuration Structure for the given Log File Identifier
 *  \param  te_log_file_identifier     - Log File Identifier
 *  \return ts_log_file_configuration* - Pointer to the Log File Configuration Structure
 */
extern ts_log_file_configuration* get_log_file_configuration_structure(te_log_file_identifier log_file_identifier);
#endif
void InitAocs(void);

#endif
