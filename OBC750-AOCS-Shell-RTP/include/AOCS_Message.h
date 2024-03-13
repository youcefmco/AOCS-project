/****************************************************************************** 
 * Copyright (c) 2014 Surrey Satellite Technology, Ltd.
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
 * Last Update : $Date: 2015/11/17 14:22:06 $
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/include/AOCS_Message.h,v $
 * Revision    : $Revision: 1.1 $
 *
 * History:
 *
 * $Log: AOCS_Message.h,v $
 * Revision 1.1  2015/11/17 14:22:06  ytrichakis
 * Updated for new messgae handling + sked
 *
 * Revision 1.1  2015/09/18 09:28:11  ytrichakis
 * Removed ewod handling.
 * Fixes for R_20
 *
 *
 ******************************************************************************/

#ifndef __AOCS_MESSAGE_H
#define __AOCS_MESSAGE_H

#ifdef __cplusplus
/* *INDENT-OFF* */
extern "C" {
/* *INDENT-ON* */
#endif

/*---------------------------------------------------------------------------
 * Includes
 */

/*---------------------------------------------------------------------------
 * Defines and Macros (shared within this module)
 */

#define AOCS_SERVICE_DISPATCHER_MESSAGE_QUEUE_NAME      "/AOCS_DispatcherMessageQueue"   /*!< AOCS Dispatcher Message Queue Name         */
#define AOCS_INTERFACE_MAXIMIM_MESSAGE_QUEUE_NAME_SIZE  (0x0040)                         /*!< AOCS Maximum Message Queue Name Size       */

/*---------------------------------------------------------------------------
 * Typedefs (shared within this module)
 */

typedef enum
{
   eAOCSTimerMessage         = 0x00,
   ePPSTimerMessage          = 0x01,
   eActuationTimerMessage    = 0x02,
   eAimCheckTimerMessage     = 0x03,
   eAOCSLocalCanCmd          = 0x04,
   eAOCSRegistrationMessage  = 0x05,
   eAOCSProcessTerminating   = 0x06
}te_aocs_message_type;

typedef struct
{
   void                          *message_pointer;                                                                     /*!< Message Index                                        */
   tsTcTlmNodeValue              TcTlmNodeValue;                                                                       /*!< Internal CAN Message                                 */
   tGDEF_INT8                    aocs_interface_message_queue_name[AOCS_INTERFACE_MAXIMIM_MESSAGE_QUEUE_NAME_SIZE];    /*!< AOCS Interface Registation Message Queue Name        */
}ts_aocs_message_data;

typedef struct
{
   te_aocs_message_type         aocs_message_type;                                  /*!< AOCS Message Type                            */
   ts_aocs_message_data         aocs_message_data;                                  /*!< AOCS Message Data                            */
   te_obc750_process_identifier obc750_process_identifier;                          /*!< Process Identifier                           */
} ts_aocs_message;

/*---------------------------------------------------------------------------
 * Data - (shared within this module)
 */
 
/*---------------------------------------------------------------------------
 * Module Function Prototypes
 */

#ifdef __cplusplus
/* *INDENT-OFF* */
}
/* *INDENT-ON* */
#endif

#endif /* __AOCS_MESSAGE_H */

/*!--------------------------------------------------------------------------
 * End of file
 */
