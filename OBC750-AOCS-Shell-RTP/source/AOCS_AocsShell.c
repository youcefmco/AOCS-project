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
 * Last Update : $Date: 2016/03/17 15:57:36 $
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/source/AOCS_AocsShell.c,v $
 * Revision    : $Revision: 1.20 $
 *
 * History:
 *
 * $Log: AOCS_AocsShell.c,v $
 * Revision 1.20  2016/03/17 15:57:36  ytrichakis
 * Create FSM thread before CAN and check for EEROR in dispatcher queue
 *
 * Revision 1.14  2014/07/15 16:13:46  ytrichakis
 * Changed ts_ChanValue to tsTcChanValue
 *
 * Revision 1.13  2013/09/10 13:12:51  ytrichakis
 * Fixed downlink IP telemetry
 *
 * Revision 1.12  2013/09/06 15:47:57  ytrichakis
 * removed printf
 *
 * Revision 1.11  2013/09/06 13:40:22  ytrichakis
 * removed unguarded printfs
 *
 * Revision 1.10  2013/09/03 13:35:21  ytrichakis
 * Removed calls to logging library as now all logs are done in the internel ADYYMMDD file
 *
 * Revision 1.9  2013/09/02 09:20:34  ytrichakis
 * AOCS Version before completion review 4/9/13
 *
 * Revision 1.8  2013/07/18 15:56:53  ytrichakis
 * fixed wheel bug(DR#14548) and applied the same solution in case of failed AIM
 *
 * Revision 1.7  2013/06/24 13:13:25  ytrichakis
 * Removed TODO's as agreed between YT and AJ (07/06/13)
 *
 * Revision 1.6  2013/06/21 13:54:39  ytrichakis
 * Send command 5 only to non-failed AIMs. Also set ewod closing tie to 1 day
 *
 * Revision 1.5  2013/06/04 10:09:03  ytrichakis
 * AOCS Delivery 4th June for flight rehearsal
 *
 * Revision 1.4  2013/05/24 13:03:27  ytrichakis
 * Fixed AIM0 problem and added TC return handling from SKED RTP
 *
 * Revision 1.3  2013/04/23 15:49:21  ytrichakis
 * Added block wait till SKED msgQ is ready to open
 *
 * Revision 1.2  2013/04/17 14:17:40  ytrichakis
 * Check in with fixing the identation only
 *
 * Revision 1.1  2013/04/11 13:42:21  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/
#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <timers.h>
#include <limits.h>
#include <math.h>
#include <mqueue.h>
#include <signal.h>
#include <pthread.h>
#include <sched.h>
#include <sys/stat.h>

#include "GDEF_GlobDefs.h"
#include "Adcs_mission.h"
#include "mission.h"
#include "endian_transposition.h"

#include "downlink_telemetry.h"

#include "AOCS_AocsShell.h"
#include "CANS_Interface.h"
#include "CANS_API.h"
#include "spacecraft_log.h"
#include "CANA_ServerApiLib.h"
#include "SXAP_SkedExecAPI.h"
#include "ACON_AocsConfig.h"
#include "EWOD_EwodHandler.h"
#include "AROH_AroHandler.h"
#include "AINT_AdcsInterface.h"
#include "ATTC_DnlkDefs.h"
#include "ATTC_AocsTTC.h"
#include "AMOH_AocsModeHandler.h"
#include "HINT_MWheel.h"
#ifdef _INC_STAR_TRACKER_
#include "HINT_Dpu.h"
#endif
#include "HINT_AIM.h"
#include "HMGR_HardwareManager.h"
#include "HMGR_Globals.h"
#include "AFIH_FileHandler.h"
#include "AOCS_AocsFSM.h"
#include "AOCS_Message.h"
#include "AOCS_AocsFSM.h"

/*-----------------------------------------------------------------------
 * Defines and Macros
 */
#define AOCS_NODE_COUNT        (1)
#define AOCS_RX_QUEUES         (3)
#define NO_OF_AIMS             (2)
#define FOUR_BYTES_LENGTH      (4)
#define REGISTERED_SKEDS       (4)
#define MSG_QUEUE_OPEN_TIMEOUT (30)

/*---------------------------------------------------------------------------
 * Private Function Prototypes (declared as static; used only within this module)
 */
static void TaskInit(void);
#ifdef USE_LOGGING_LIBRARY
static void aocs_process_log_file_revision_information(void);
#endif
static void aocs_can_thread(void);
static void TaskExit(void);

/*---------------------------------------------------------------------------
 * Local Data (declared as static; used only within this module)
 */
static const tsADLK_CHAN_LOOKUP ADLK_tlmTable[] = ADLK_TLM_TABLE;       /* table to map telemtry points to dnlk type */
static const tGDEF_UINT16       ADLK_numChans   = sizeof(ADLK_tlmTable) / sizeof(tsADLK_CHAN_LOOKUP);
static teGDEF_BOOLEAN           setTimerFlag    = GDEF_FALSE;

static ts_aocs_shell_configuration aocs_shell_configuration_table = 
{   
   .firewall_head                 = FIREWALL_PATTERN,                      /*!< Firewall Pattern to help detect Memory Corruption   */
   .aocs_shell_protection_mutex   = PTHREAD_MUTEX_INITIALIZER,             /*!< AOCS Shell Protection Mutex                         */
   .aocs_shell_fsm_state          = 0,                                     /*!< AOCS Shell FSM State                                */
   .aocs_shell_fsm_event          = 0,                                     /*!< AOCS Shell FSM Event                                */
   .aocs_shell_event_action_time  = 0,                                     /*!< AOCS Shell FSM Event Action Time                    */
#ifdef FSM_TIMER   
   .aocs_shell_posix_timer_id     = 0,                                     /*!< POSIX Timer ID                                      */
   .aocs_shell_posix_timer        = {{0}},                                 /*!< POSIX Timer                                         */
#endif /* FSM_TIMER */
   .aocs_shell_pps_timer_id       = 0,                                     /*!< AOCS Shell PPS Timer ID                             */
   .aocs_shell_pps_timer          =                                        /*!< AOCS Shell PPS Timer                                */
   {   
      .it_interval =
      {
         .tv_sec  = 1,
         .tv_nsec = 0,
      },
      .it_value =
      {
         .tv_sec  = 0,
         .tv_nsec = 0,
      },
   },                                 
   .aocs_shell_actuation_timer_id = 0,                                     /*!< AOCS Shell Actuation Timer ID                       */
   .aocs_shell_actuation_timer    =                                        /*!< AOCS Shell Actuation Timer                          */
   {
      .it_interval =
      {
         .tv_sec  = 0,
         .tv_nsec = 0,
      },
      .it_value =
      {
         .tv_sec  = 0,
         .tv_nsec = FIRE_CMDS_DELAY,
      },
   },
#ifdef PERFORM_CAN_TIME_SYNCHRONISATION_WITHIN_AOCS_SHELL   
   .aocs_shell_aim_check_timer_id = 0,                                     /*!< AOCS Shell AIM Check Timer ID                       */
   .aocs_shell_aim_check_timer    =                                        /*!< AOCS Shell AIM Check Timer                          */
   {
      .it_interval =
      {
         .tv_sec  = 4,
         .tv_nsec = 0,
      },
      .it_value =
      {
         .tv_sec  = 4,
         .tv_nsec = 0,
      },
   },
#endif /* PERFORM_CAN_TIME_SYNCHRONISATION_WITHIN_AOCS_SHELL */
   .firewall_tail                 = FIREWALL_PATTERN                       /*!< Firewall Pattern to help detect Memory Corruption   */
};

static ts_log_file_configuration log_file_configuration_table[MAXIMUM_LOG_FILE_IDENTIFIER] = 
{
   {
      .firewall_head             = FIREWALL_PATTERN,                       /*!< Firewall Pattern to help detect Memory Corruption   */ 
      .file_protection_mutex     = PTHREAD_MUTEX_INITIALIZER,              /*!< File Access Protection Mutex                        */
      .file_name_format          = {{{0}}},                                /*!< File Name                                           */
      .file_close_time           = 0,                                      /*!< File Close Time                                     */
      .file_pointer              = NULL,                                   /*!< File Pointer                                        */
      .file_header_checksum      = 0,                                      /*!< File Header Checksum                                */
      .sequence_number           = 0,                                      /*!< File Name Sequence Number                           */
      .tm_mday                   = 0,                                      /*!< File Open Calendar Month                            */
      .file_close_timer_id       = 0,                                      /*!< File Close Timer Identifier                         */
      .file_type_identifier      = 0,                                      /*!< File Type Identifier                                */
      .file_name_sequence_number = GDEF_FALSE,                             /*!< File Name Contains Sequence Number                  */
      .file_name_prefix          = "GL",                                   /*!< File Name Prefix                                    */
      .firewall_tail             = FIREWALL_PATTERN                        /*!< Firewall Pattern to help detect Memory Corruption   */
   }
};

static tsCANS_RegRtn             stCANReg      = {0};         /* CAN API Registration return structure - passed to all subsequent CAN calls */
static tsCANA_TcTlmRequestParams stTcReqParams = {0};         /* CAN API Default TC request parameters */

/*---------------------------------------------------------------------------
 * Public Functions
 */

/*---------------------------------------------------------------------------
 * Global Data
 */
mqd_t dispatcher_thread_communications_channel = (mqd_t)EERROR;
extern tsAOCS_TTC    ttcState;


/*!
 *  \brief Routine to the CAN API Registration Structure
 *  \param  void           - None
 *  \return tsCANS_RegRtn* - Pointer to the CAN API Registration Structure
 */
tsCANS_RegRtn *getCANReg(void)
{
   return (&stCANReg);
}

/*!
 *  \brief Routine to return the Log File Configuration Structure for the given Log File Identifier
 *  \param  te_log_file_identifier     - Log File Identifier
 *  \return ts_log_file_configuration* - Pointer to the Log File Configuration Structure
 */
ts_log_file_configuration* get_log_file_configuration_structure(te_log_file_identifier log_file_identifier)
{
   ts_log_file_configuration *log_file_configuration = NULL;
   
   /*
    * We will first perform a range check on the given Log File Identifier.
    */
   LOG_FILE_IDENTIFIER_RANGE_CHECK(log_file_identifier, MINIMUM_LOG_FILE_IDENTIFIER, MAXIMUM_LOG_FILE_IDENTIFIER);
   
   /*
    * Obtain a pointer to the Log File Configuration Structure for the given Log File Identifier.
    */
   log_file_configuration = &log_file_configuration_table[log_file_identifier];
   
   /*
    * RETURN
    */
   return log_file_configuration;
}

/*!
 *  \brief Routine to generate the telemetry information that will be transmitted on the downlink.
 *  \param  ts_downlink_telemetry_frame_write_configuration *const - Pointer to the Downlink Link Telemetry Configuration Structure
 *  \return void                                                   - None
 */
void AOCS_downlink_telemetry_handler(ts_downlink_telemetry_frame_write_configuration *downlink_telemetry_frame_write_configuration)
{
#ifdef REALTIME_DEBUG
   (void)printf("%s: Entered Function [aocs_downlink_telemetry_handler]\r\n",PROCESS_NAME);
#endif /* REALTIME_DEBUG */

   /*lint --e(506) Constant value Boolean */
   assert(downlink_telemetry_frame_write_configuration != NULL);

   /*
    * Initialise the Downlink Telemetry Frame that we will transmit to the Ground Station.
    */
   downlink_telemetry_frame_initialise(downlink_telemetry_frame_write_configuration);

   /*
    * Build the AOCS Process Telemetry Downlink Frame that we will transmit to the Ground Station.
    */
   {

      ts_downlink_telemetry_data downlink_telemetry_data[ADLK_MAX_NUM_CHANS] = {{0}};
      tGDEF_UINT8                index                                       = 0;
      tGDEF_UINT32*              ADLK_pTlmData                               = NULL;

      for(index = 0; index < ADLK_numChans; index++)
      {
         ATTC_TlmHandler(ADLK_tlmTable[index].chan, &ADLK_pTlmData);
         /*
          * Update the telemetry data
          */
         downlink_telemetry_data[index].node    = CANADDR_ADCS_PROCESS;
         downlink_telemetry_data[index].channel = ADLK_tlmTable[index].chan;
         downlink_telemetry_data[index].value   = *ADLK_pTlmData;

         if (ADLK_tlmTable[index].len == FOUR_BYTES_LENGTH)
         {
            downlink_telemetry_data[index].state = eCANS_TRAN_TLM_32BIT;
         }
         else
         {
            /* dnlk will assume 2 bytes */

            /* if invalid data only use top 2 bytes */
            if (downlink_telemetry_data[index].value == AINT_INVALID_32BIT_TLM)
            {
               downlink_telemetry_data[index].value >>= 16;
            }
            
            /* mask off top two bytes */
            downlink_telemetry_data[index].value = (downlink_telemetry_data[index].value) & 0x0000FFFF;
            downlink_telemetry_data[index].state = eCANS_TRAN_TLM_16BIT;
         }
      
         /*
          * Send downlink telemetry
          */
         downlink_telemetry_write_frame_data(downlink_telemetry_frame_write_configuration, downlink_telemetry_data[index]);

      }
   }

   /*
    * RETURN
    */
   return;
}

/*!
 *  \brief Routine to handle the PPS Timer Signal
 *  \param  timer_t              timerid                   - POSIX Timer ID
 *  \param  ts_gps_configuration *aocs_shell_configuration - Pointer to the AOCS Configuration Structure
 *  \return void                                           - None
 */
void aocs_pps_timer_signal_handler(timer_t timerid, ts_aocs_shell_configuration *const aocs_shell_configuration)
{
   ts_aocs_message aocs_message = {0};
   tGDEF_INT32     status       = EERROR;
      
   /*lint --e(506) Constant value Boolean */  
   assert(aocs_shell_configuration != NULL);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_head == FIREWALL_PATTERN);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_tail == FIREWALL_PATTERN);
 
   /*
    * Build a message that we will send to the AOCS Process Dispatcher.
    */
   aocs_message.aocs_message_type                 = ePPSTimerMessage;  
   aocs_message.aocs_message_data.message_pointer = ((void *)aocs_shell_configuration);
   
   /*lint --e(506) Constant value Boolean */  
   assert(dispatcher_thread_communications_channel != NULL);
   
   /*
    * Send the message to the AOCS Process Dispatcher.
    */
   status = mq_send(dispatcher_thread_communications_channel, ((const char *)(&aocs_message)), sizeof(ts_aocs_message), 0);
   /*lint --e(792) void cast of void expression */
   assert(status == EOK);
   
   /*
    * RETURN
    */
   return;
}

/*!
 *  \brief Routine to handle the Actuation Timer Signal
 *  \param  timer_t              timerid                   - POSIX Timer ID
 *  \param  ts_gps_configuration *aocs_shell_configuration - Pointer to the AOCS Configuration Structure
 *  \return void                                           - None
 */
void aocs_actuation_timer_signal_handler(timer_t timerid, ts_aocs_shell_configuration *const aocs_shell_configuration)
{
   ts_aocs_message aocs_message = {0};
   tGDEF_INT32     status       = EERROR;
      
   /*lint --e(506) Constant value Boolean */  
   assert(aocs_shell_configuration != NULL);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_head == FIREWALL_PATTERN);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_tail == FIREWALL_PATTERN);
 
   /*
    * Build a message that we will send to the AOCS Process Dispatcher.
    */
   aocs_message.aocs_message_type                 = eActuationTimerMessage;  
   aocs_message.aocs_message_data.message_pointer = ((void *)aocs_shell_configuration);
   
   /*
    * Send the message to the AOCS Process Dispatcher.
    */
   status = mq_send(dispatcher_thread_communications_channel, ((const char *)(&aocs_message)), sizeof(ts_aocs_message), 0);
   /*lint --e(792) void cast of void expression */
   assert(status == EOK);
    
   /*
    * RETURN
    */
   return;
}

/*!
 *  \brief Routine to handle the AIM Check Timer Signal
 *  \param  timer_t              timerid                   - POSIX Timer ID
 *  \param  ts_gps_configuration *aocs_shell_configuration - Pointer to the AOCS Configuration Structure
 *  \return void                                           - None
 */
void aocs_aim_check_timer_signal_handler(timer_t timerid, ts_aocs_shell_configuration *const aocs_shell_configuration)
{
   ts_aocs_message aocs_message = {0};
   tGDEF_INT32     status       = EERROR;
      
   /*lint --e(506) Constant value Boolean */  
   assert(aocs_shell_configuration != NULL);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_head == FIREWALL_PATTERN);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_tail == FIREWALL_PATTERN);
 
   /*
    * Build a message that we will send to the AOCS Process Dispatcher.
    */
   aocs_message.aocs_message_type                 = eAimCheckTimerMessage;  
   aocs_message.aocs_message_data.message_pointer = ((void *)aocs_shell_configuration);
   
   /*
    * Send the message to the AOCS Process Dispatcher.
    */
   status = mq_send(dispatcher_thread_communications_channel, ((const char *)(&aocs_message)), sizeof(ts_aocs_message), 0);
   /*lint --e(792) void cast of void expression */
   assert(status == EOK);
    
   /*
    * RETURN
    */
   return;
}

/*!
 *  \brief Routine to handle the Message Interface implemented as a POSIX Thread.
 *  \param  void - None
 *  \return void - None
 */
void aocs_message_dispatcher_thread(ts_aocs_shell_configuration *const aocs_shell_configuration)
{
   tGDEF_INT32    status                   = EERROR;
   struct mq_attr message_queue_attributes = {0};
   
   /*lint --e(506) Constant value Boolean */
   assert(aocs_shell_configuration != NULL);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_head == FIREWALL_PATTERN);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_tail == FIREWALL_PATTERN);

#ifdef REALTIME_DEBUG
   (void)printf("%s Process Message Dispatcher Thread Started\r\n", PROCESS_NAME);
#endif /* REALTIME_DEBUG */
   {
      tGDEF_INT8  dispatcher_message_queue_name[50] = {0};

      strncpy(dispatcher_message_queue_name, MESSAGE_QUEUE_PREFIX, sizeof(dispatcher_message_queue_name));
      strcat(dispatcher_message_queue_name, DISPATCHER_MESSAGE_QUEUE);

      /*
       * Unlink and Remove POSIX Message Queue from the Name Space.
       */
      status = mq_unlink(dispatcher_message_queue_name);
      if(status == EERROR)
      {
#ifdef REALTIME_DEBUG
         (void)printf("Unable to unlink and remove message queue %s from name space\r\n", dispatcher_message_queue_name);
#endif /* REALTIME_DEBUG */
      }
      else
      {
#ifdef REALTIME_DEBUG
         (void)printf("Unlinked and removed message queue %s from name space\r\n", dispatcher_message_queue_name);
#endif /* REALTIME_DEBUG */
      }
      
      /*
       * Create the AOCS Shell Proctection Mutex and the AOCS Shell PSS and Actuation Timers.
       */
      {
         pthread_mutexattr_t aocs_shell_mutex_attributes = {0};
         
         /*
          * Initialise the AOCS Shell Protection Mutex attribute object.
          */
         status = pthread_mutexattr_init(&aocs_shell_mutex_attributes);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK);

#ifdef SET_PTHREAD_MUTEX_RECURSIVE
         /*
          * Set the RECURSIVE attribute allowing the Mutex to be relocked by a Thread.
          */
         pthread_mutexattr_settype(&aocs_shell_mutex_attributes, PTHREAD_MUTEX_RECURSIVE);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK);
#endif /* SET_PTHREAD_MUTEX_RECURSIVE */
          
         /*
          * Initialise the AOCS Shell Protection Mutex.
          */
         status = pthread_mutex_init(&aocs_shell_configuration->aocs_shell_protection_mutex, &aocs_shell_mutex_attributes);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK);  
         
         /*
          * Create the AOCS PPS Timer.
          */
         status = timer_create(CLOCK_REALTIME, NULL, &aocs_shell_configuration->aocs_shell_pps_timer_id);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK);
         
         /*
          * Connect the AOCS PPS Timer.
          */
         status = timer_connect(aocs_shell_configuration->aocs_shell_pps_timer_id, ((VOIDFUNCPTR)(aocs_pps_timer_signal_handler)), (tGDEF_INT32)aocs_shell_configuration);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK); 
         
         /*
          * Create the AOCS Actuation Timer.
          */
         status = timer_create(CLOCK_REALTIME, NULL, &aocs_shell_configuration->aocs_shell_actuation_timer_id);         
         /*lint --e(792) void cast of void expression */
         assert(status == EOK);
         
         /*
          * Connect the AOCS Actuation Timer.
          */
         status = timer_connect(aocs_shell_configuration->aocs_shell_actuation_timer_id, ((VOIDFUNCPTR)(aocs_actuation_timer_signal_handler)), (tGDEF_INT32)aocs_shell_configuration);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK);

#ifdef PERFORM_CAN_TIME_SYNCHRONISATION_WITHIN_AOCS_SHELL         
         /*
          * Create the AOCS PPS Timer.
          */
         status = timer_create(CLOCK_REALTIME, NULL, &aocs_shell_configuration->aocs_shell_aim_check_timer_id);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK);
         
         /*
          * Connect the AOCS PPS Timer.
          */
         status = timer_connect(aocs_shell_configuration->aocs_shell_aim_check_timer_id, ((VOIDFUNCPTR)(aocs_aim_check_timer_signal_handler)), (tGDEF_INT32)aocs_shell_configuration);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK); 
#endif /* PERFORM_CAN_TIME_SYNCHRONISATION_WITHIN_AOCS_SHELL */
      }

      /*
       * Configure the POSIX Message Queue Attributes
       */
      message_queue_attributes.mq_maxmsg  = AOCS_MESSAGE_QUEUE_SIZE;
      message_queue_attributes.mq_msgsize = sizeof(ts_aocs_message);

      /*
       * Create the AOCS Process Dispatcher Thread Message Queue.
       */
      dispatcher_thread_communications_channel = mq_open(dispatcher_message_queue_name, (O_RDWR | O_CREAT), (S_IRUSR | S_IWUSR), &message_queue_attributes);
      /*lint --e(792) void cast of void expression */
      assert(dispatcher_thread_communications_channel != ((mqd_t)EERROR));
   }
   
   /*
    * Start the AOCS Shell PPS Timer.
    */
   {
      struct timespec acos_pps_timer = {0};
      
      status = clock_gettime(CLOCK_REALTIME, &acos_pps_timer);
      /*lint --e(792) void cast of void expression */
      assert(status == EOK); 
      
      aocs_shell_configuration->aocs_shell_pps_timer.it_value.tv_sec  = 0;
      aocs_shell_configuration->aocs_shell_pps_timer.it_value.tv_nsec = NSEC_TO_SEC - acos_pps_timer.tv_nsec;
      
      status = timer_settime(aocs_shell_configuration->aocs_shell_pps_timer_id, CLOCK_REALTIME, &aocs_shell_configuration->aocs_shell_pps_timer, NULL);
      /*lint --e(792) void cast of void expression */
      assert(status == EOK);     
   }
   
#ifdef PERFORM_CAN_TIME_SYNCHRONISATION_WITHIN_AOCS_SHELL
   /*
    * Start the AOCS Shell AIM Check Timer.
    */
   {
      status = timer_settime(aocs_shell_configuration->aocs_shell_aim_check_timer_id, CLOCK_REALTIME, &aocs_shell_configuration->aocs_shell_aim_check_timer, NULL);
      /*lint --e(792) void cast of void expression */
      assert(status == EOK); 
   }
#endif /* PERFORM_CAN_TIME_SYNCHRONISATION_WITHIN_AOCS_SHELL */

   /*
    * Repeatedly read messages received on the AOCS Dispatcher Message Queue
    */
   for(;;)
   {
      ts_aocs_message aocs_message     = {0};
      tGDEF_INT32     status           = 0;
      tGDEF_INT32     message_priority = 0;

      /*
       * Block on the AOCS Shell Dispatcher Message Queue
       */
      status = mq_receive(dispatcher_thread_communications_channel, ((char*)(&aocs_message)), sizeof(ts_aocs_message), ((unsigned int*)(&message_priority)));
      if(status != EERROR)
      {
         /*
          * Process the received message
          */
         switch(aocs_message.aocs_message_type)
         {
            case eAOCSTimerMessage:
               {
                  ts_aocs_shell_configuration *aocs_shell_configuration = ((ts_aocs_shell_configuration*)(aocs_message.aocs_message_data.message_pointer));
                  
                  /*lint --e(506) Constant value Boolean */
                  assert(aocs_shell_configuration != NULL);
                  /*lint --e(792) void cast of void expression */
                  assert(aocs_shell_configuration->firewall_head == FIREWALL_PATTERN);
                  /*lint --e(792) void cast of void expression */
                  assert(aocs_shell_configuration->firewall_tail == FIREWALL_PATTERN);
                  
                  aocs_shell_process_event(aocs_shell_configuration);
               }
               break;

            case ePPSTimerMessage: 
               {
                  ts_aocs_shell_configuration *aocs_shell_configuration = ((ts_aocs_shell_configuration*)(aocs_message.aocs_message_data.message_pointer));
                  
                  /*lint --e(506) Constant value Boolean */
                  assert(aocs_shell_configuration != NULL);
                  /*lint --e(792) void cast of void expression */
                  assert(aocs_shell_configuration->firewall_head == FIREWALL_PATTERN);
                  /*lint --e(792) void cast of void expression */
                  assert(aocs_shell_configuration->firewall_tail == FIREWALL_PATTERN);
                  
                  /*
                   * Obtain the AOCS Shell Protection Mutex.
                   */
                  status = pthread_mutex_lock(&aocs_shell_configuration->aocs_shell_protection_mutex);
                  /*lint --e(792) void cast of void expression */
                  assert(status == EOK);
                  
                  {
                     aocs_shell_configuration->aocs_shell_fsm_event         = PPS_RECEIVED;
                     aocs_shell_configuration->aocs_shell_event_action_time = time(NULL);
                  }
                  
                  /*
                   * Release the AOCS Shell Protection Mutex.
                   */
                  status = pthread_mutex_unlock(&aocs_shell_configuration->aocs_shell_protection_mutex);
                  /*lint --e(792) void cast of void expression */
                  assert(status == EOK);
                  
                  aocs_shell_deliver_event(aocs_shell_configuration);
               }
               break;
               
            case eActuationTimerMessage:
               {
                  ts_aocs_shell_configuration *aocs_shell_configuration = ((ts_aocs_shell_configuration*)(aocs_message.aocs_message_data.message_pointer));
                  
                  /*lint --e(506) Constant value Boolean */
                  assert(aocs_shell_configuration != NULL);
                  /*lint --e(792) void cast of void expression */
                  assert(aocs_shell_configuration->firewall_head == FIREWALL_PATTERN);
                  /*lint --e(792) void cast of void expression */
                  assert(aocs_shell_configuration->firewall_tail == FIREWALL_PATTERN);
                  
                  /* 
                   * Obtain the AOCS Shell Protection Mutex.
                   */
                  status = pthread_mutex_lock(&aocs_shell_configuration->aocs_shell_protection_mutex);
                  /*lint --e(792) void cast of void expression */
                  assert(status == EOK);
                  
                  {
                     aocs_shell_configuration->aocs_shell_fsm_event         = ACTUATE;
                     aocs_shell_configuration->aocs_shell_event_action_time = time(NULL);
                  }
                  
                  /*
                   * Release the AOCS Shell Protection Mutex.
                   */
                  status = pthread_mutex_unlock(&aocs_shell_configuration->aocs_shell_protection_mutex);
                  /*lint --e(792) void cast of void expression */
                  assert(status == EOK);
                  
                  aocs_shell_deliver_event(aocs_shell_configuration);
               }
               break;
               
            case eAimCheckTimerMessage:
               {
                  /*
                   * Send sync cmds only if we are not SBM of SAFE
                   */
                  if((ttcState.mode.current != MODE_SBM) && (ttcState.mode.current != MODE_SAFE))
                  {
                     tsTcTlmNodeValue          stTc           = {0};
                     tsCANA_TcTlmRequestParams stTcReqParams  = {0};
                     teCANA_APIRes             CANA_APIRes    = 0;
                     
                     stTcReqParams.psRegInfo      = (tsCANS_RegRtn*)getCANReg();
                     stTcReqParams.eSendQPriority = eCANS_LO_PRI_Q;
                     stTcReqParams.eWait          = eCANS_NO_WAIT;
                     stTcReqParams.u1Src          = CANADDR_ADCS_PROCESS;
                     stTcReqParams.u4RetryCount   = DEFAULT_AOCS_TC_RETRY_COUNT;
                     stTcReqParams.u4RtnQIdx      = DEFAULT_DISPATCHER_Q;
                     stTcReqParams.u4Timeout      = DEFAULT_TELECOMMAND_REQUEST_TIMEOUT;
                     
                     /*
                      * Send TC 5 only to working AIMs.
                      */
                     if(ttcState.algConfig.mag.failedAIM != 1)
                     {
                        /*
                         * Build and send the sync Telecommand(5,1) to AIM0
                         */
                        stTc.u1Dest  = CANADDR_AIM0;
                        stTc.u2Chan  = 5;
                        stTc.u4Value = 1;

                        CANA_APIRes = CANA_send_tc(&stTcReqParams, &stTc);
                        HMGR_numAcksPending++;
                        assert(CANA_APIRes == eCANA_OK);
                     }

                     if(ttcState.algConfig.mag.failedAIM != 2)
                     {
                        /*
                         * Build and send the sync Telecommand(5,1) to AIM1
                         */
                        stTc.u1Dest  = CANADDR_AIM1;
                        stTc.u2Chan  = 5;
                        stTc.u4Value = 1;

                        CANA_APIRes = CANA_send_tc(&stTcReqParams, &stTc);
                        HMGR_numAcksPending++;
                        assert(CANA_APIRes == eCANA_OK);
                     }
                  }                  
               }
               break;
               
            default:
               EWOD_WriteMessage("Unknown Message received\n");
               break;
         } /* switch(aocs_message.eMsgType) */
      }
   }

   /*
    * NOT REACHED
    */
   /*lint --e(792) void cast of void expression */
   /*lint --e(527) unreachable */
   assert(GDEF_FALSE);
}

/*!
 *  \brief Entry point for the OBC750 AOCS Process
 *  \param  int    - Number of Command Line Arguments
 *  \param  char** - Command Line Argument List
 *  \return int    - OK
 */
int main(int argc, char** argv)
{
   ts_aocs_shell_configuration *aocs_shell_configuration = &aocs_shell_configuration_table;
   tsCANS_Registration         CANS_Registration         = {{0}};

#ifdef REALTIME_DEBUG
   (void)printf("%s Process Started\r\n",PROCESS_NAME);
#endif /* REALTIME_DEBUG */
  
   /*lint --e(506) Constant value Boolean */
   assert(aocs_shell_configuration != NULL);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_head == FIREWALL_PATTERN);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_tail == FIREWALL_PATTERN);
   
   /*
    * Initalise the AOCS Process Log.
    */
   log_file_initialise(get_log_file_configuration_structure(AOCS_PROCESS_LOG_FILE));
  
   /*
    * Populate the Client RTP Name Element of the CAN Server Registration Structure.
    */
   snprintf(CANS_Registration.sClientRTPName, sizeof(CANS_Registration.sClientRTPName),"%s@%d", PROCESS_NAME, CANADDR_ADCS_PROCESS);

   CANS_Registration.u1CanNodeFirst           = CANADDR_ADCS_PROCESS;
   CANS_Registration.u1CanNodesCount          = AOCS_NODE_COUNT;
   CANS_Registration.u1RxQueuesRqd            = AOCS_PROCESS_MAXIMUM_CAN_QUEUE_IDENTIFIER;;

   CANS_Registration.u4ServiceFlags[AOCS_PROCESS_CAN_RECEIVE_QUEUE_IDENTIFIER]   = eCANS_SRVC_SUPPORTS_TLM_REQUESTS | eCANS_SRVC_SUPPORTS_TC_REQUESTS | eCANS_SRVC_SUPPORTS_RAW_DATAGRAMS | eCANS_SRVC_SUPPORTS_STRM_PUT | eCANS_SRVC_SUPPORTS_TLM_PUTS;
   CANS_Registration.u4ServiceFlags[AOCS_PROCESS_CAN_QUEUE_IDENTIFIER]           = eCANS_SRVC_REQUESTS_TELEMETRIES  | eCANS_SRVC_REQUESTS_TELECOMMANDS;
   CANS_Registration.u4ServiceFlags[AOCS_PROCESS_SKED_QUEUE_IDENTIFIER]          = eCANS_SRVC_SUPPORTS_TC_REQUESTS  | eCANS_SRVC_REQUESTS_TELECOMMANDS| eCANS_SRVC_SUPPORTS_TC_BATCH;

   CANS_Registration.bOpenTxHiQ               = GDEF_FALSE;
   CANS_Registration.bOpenTxLoQ               = GDEF_TRUE;
   CANS_Registration.bOpenDiagnosticQ         = GDEF_FALSE;

   if(CANA_Register_RTP(&CANS_Registration, &stCANReg) == GDEF_FALSE)
   {
      (void)EWOD_WriteMessage("CAN Registration failed\n");
#ifdef REALTIME_DEBUG
      (void)printf("%s - Registration failed!!! \n",PROCESS_NAME);
      (void)printf("Name %s, Rtn Code %d, QErrs %d, RxQId0 %d, RxQId1 %d, RxQId2 %d, TxQIdLo %d, TxQIdHi %d\n", stClientReg.sClientRTPName, stCANReg.eRegStatus, stCANReg.u4MsgQueueErrors, (int)stCANReg.mqRxMsgQId[0], (int)stCANReg.mqRxMsgQId[1], (int)stCANReg.mqRxMsgQId[2], (int)stCANReg.mqTxLoQId, (int)stCANReg.mqTxHiQId);
#endif
      /*lint --e(506) Constant value Boolean */
      assert(GDEF_FALSE);
   }
   else
   {
      /*
       * Build the default TC to CAN bus request parameters#
       */
      stTcReqParams.psRegInfo      = (tsCANS_RegRtn*)getCANReg();
      stTcReqParams.eSendQPriority = eCANS_LO_PRI_Q;
      stTcReqParams.eWait          = eCANS_WAIT;
      stTcReqParams.u1Src          = CANADDR_ADCS_PROCESS;
      stTcReqParams.u4RetryCount   = DEFAULT_AOCS_TC_RETRY_COUNT;
      stTcReqParams.u4RtnQIdx      = AOCS_PROCESS_CAN_QUEUE_IDENTIFIER;
      stTcReqParams.u4Timeout      = DEFAULT_TELECOMMAND_REQUEST_TIMEOUT;

      /*
       *  Create the AOCS CAN Thread.
       */
      {
         tGDEF_INT32        status                = 0;
         pthread_t          can_thread_id         = 0;
         pthread_attr_t     can_thread            = {0};      
         struct sched_param can_thread_parameters = {0};
        
         /*
          * Configure The Parameters Of the AOCS CAN Thread.
          */
         status = pthread_attr_init(&can_thread);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK);
        
         /*
          * Set the CAN Thread Name Attribute.
          */
         status = pthread_attr_setname(&can_thread, AOCS_PROCESS_CAN_THREAD_NAME);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK);
        
         /*
          * Set the CAN Thread Detach State Attribute. 
          */
         status = pthread_attr_setdetachstate(&can_thread, PTHREAD_CREATE_DETACHED);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK);
        
         /*
          * Set the CAN Thread Scheduling Policy.
          * William Hoey Note:
          * Set Round Robin Scheduling to ensure that equal Priority Threads get access to the CPU in the event the Thread we are about to start does NOT release the CPU.
          */
         status = pthread_attr_setschedpolicy(&can_thread, SCHED_RR);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK);
        
         /*
          * Set the CAN Thread Priority.
          */
         can_thread_parameters.sched_priority = sched_get_priority_min(SCHED_RR);
         status = pthread_attr_setschedparam(&can_thread, &can_thread_parameters);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK);
  
#ifdef SET_STACK_NOTLAZY
         /*
          * Allocate the entire thread stack up front.
          */
         status = pthread_attr_setstacklazy(&dispatcher_thread, PTHREAD_STACK_NOTLAZY);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK);
#endif /* SET_STACK_NOTLAZY */    
         
         /*
          * Set the CAN Thread Scheduling Inheritance Policy.
          */
         status = pthread_attr_setinheritsched(&can_thread, PTHREAD_EXPLICIT_SCHED);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK);
     
         /*
          * Create the AOCS Process CAN Thread.
          */
         status = pthread_create(&can_thread_id, &can_thread, ((void* (*)(void* ))(aocs_can_thread)), NULL);
         assert(status == EOK);
      }
     
      /*
       * Create the AOCS Message Dispatcher Thread.
       */
      {
         tGDEF_INT32        status                       = 0;
         pthread_t          dispatcher_thread_id         = 0;
         pthread_attr_t     dispatcher_thread            = {0};      
         struct sched_param dispatcher_thread_parameters = {0};     
  
         /*
          * Configure the parameters of the AOCS Message Dispatcher Thread.
          */
         status = pthread_attr_init(&dispatcher_thread);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK);
        
         /*
          * Set the Message Dispatcher Thread Name Attribute.
          */
         status = pthread_attr_setname(&dispatcher_thread, AOCS_PROCESS_DISPATCHER_THREAD_NAME);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK);
        
         /*
          * Set the Message Dispatcher Thread Detach State Attribute. 
          */
         status = pthread_attr_setdetachstate(&dispatcher_thread, PTHREAD_CREATE_DETACHED);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK);
  
         /*
          * Set the Message Dispatcher Thread Scheduling Policy.
          * William Hoey Note:
          * Set Round Robin Scheduling to ensure that equal Priority Threads get access to the CPU in the event the Thread we are about to start does NOT release the CPU.
          */
         status = pthread_attr_setschedpolicy(&dispatcher_thread, SCHED_RR);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK);
        
         /*
          * Set the Message Dispatcher Thread Priority.
          */
         dispatcher_thread_parameters.sched_priority = sched_get_priority_min(SCHED_RR);
         status = pthread_attr_setschedparam(&dispatcher_thread, &dispatcher_thread_parameters);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK);
  
#ifdef SET_STACK_NOTLAZY
         /*
          * Allocate the entire thread stack up front.
          */
         status = pthread_attr_setstacklazy(&dispatcher_thread, PTHREAD_STACK_NOTLAZY);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK);
#endif /* SET_STACK_NOTLAZY */
        
         /*
          * Set the Message Dispatcher Thread Scheduling Inheritance Policy.
          */
         status = pthread_attr_setinheritsched(&dispatcher_thread, PTHREAD_EXPLICIT_SCHED);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK);
  
         /*
          * Create the AOCS Message Dispatcher Thread.
          */
         status = pthread_create(&dispatcher_thread_id, &dispatcher_thread, ((void* (*)(void* ))(aocs_message_dispatcher_thread)), aocs_shell_configuration);
         /*lint --e(792) void cast of void expression */
         assert(status == EOK);
      }

      /* Initialise the task */
      TaskInit();   
   
      /*
       * Register with the Downlink Telemetry Handler
       */
      downlink_telemetry_register(AOCS_downlink_telemetry_handler, ((const tGDEF_UINT8*)(PROCESS_NAME)), DEFAULT_DOWNLINK_TELEMETRY_RATE, PUBLIC_TELEMETRY_CHANNEL, DOWNLINK_TELEMETRY_SOURCE_UDP_PORT, DOWNLINK_TELEMETRY_DESTINATION_UDP_PORT);
            
      /*
       * Register with the SKED library
       */
      SXAP_InitLibShared(PROCESS_NAME, CANADDR_ADCS_PROCESS, REGISTERED_SKEDS, get_log_file_configuration_structure(AOCS_PROCESS_LOG_FILE), sched_get_priority_min(SCHED_RR), getCANReg(), AOCS_PROCESS_SKED_QUEUE_IDENTIFIER, eCANS_LO_PRI_Q);
      
      /*
       * Register the function to be called when normal program termination is requested.
       * Successive calls to atexit create a register of functions that are executed in last-in, first-out (LIFO) order.
       */
      atexit(TaskExit);
              
      /*
       * This thread has completed the GPS Process Initialisation.
       */
      pthread_exit(NULL);
   }
   
   /*
    * RETURN
    */
   return OK;
}


/*!
 * \brief      TaskExit
 *
 *             AOCS Task exit function
 *
 *
 ******************************************************************************/
static void TaskExit(void)
{
   ts_aocs_shell_configuration *aocs_shell_configuration = &aocs_shell_configuration_table;
   tGDEF_INT32                 status                    = EERROR;
   
   /*lint --e(506) Constant value Boolean */
   assert(aocs_shell_configuration != NULL);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_head == FIREWALL_PATTERN);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_tail == FIREWALL_PATTERN);
   
   /*
    * Close the CAN communication channel ensuring that any allocated resources are returned.
    */
   CANA_DeRegister_RTP(getCANReg());

   /*
    * Cancel and Delete the AOCS Shell PSS, Actuation and AIM Check Timers then Distroy the AOCS Shell Protection Mutex. 
    */
   {
      status = timer_cancel(aocs_shell_configuration->aocs_shell_pps_timer_id);
      /*lint --e(792) void cast of void expression */
      assert(status == EOK);
   
      status = timer_delete(aocs_shell_configuration->aocs_shell_pps_timer_id);
      /*lint --e(792) void cast of void expression */
      assert(status == EOK);
      
      status = timer_cancel(aocs_shell_configuration->aocs_shell_actuation_timer_id);
      /*lint --e(792) void cast of void expression */
      assert(status == EOK);
      
      status = timer_delete(aocs_shell_configuration->aocs_shell_actuation_timer_id);
      /*lint --e(792) void cast of void expression */
      assert(status == EOK);
      
#ifdef PERFORM_CAN_TIME_SYNCHRONISATION_WITHIN_AOCS_SHELL
      status = timer_cancel(aocs_shell_configuration->aocs_shell_aim_check_timer_id);
      /*lint --e(792) void cast of void expression */
      assert(status == EOK);
      
      status = timer_delete(aocs_shell_configuration->aocs_shell_actuation_timer_id);
      /*lint --e(792) void cast of void expression */
      assert(status == EOK);
#endif /* PERFORM_CAN_TIME_SYNCHRONISATION_WITHIN_AOCS_SHELL */
      
      do
      {
         /*
          * Distroy the Protection Mutex freeing the resources it might hold.
          */
         status = pthread_mutex_destroy(&aocs_shell_configuration->aocs_shell_protection_mutex);
      } while(status != EOK);
   }
   
   /*
    * Close the downlink telemetry communication channel ensuring that any allocated resources are returned.
    */
   downlink_telemetry_close(PUBLIC_TELEMETRY_CHANNEL);
   
   /*
    * Close the Spacecraft Logging communication channel ensuring that any allocated resources are returned.
    */    
   log_file_close(get_log_file_configuration_structure(AOCS_PROCESS_LOG_FILE));

   /*
    * Unlink and Remove the Message Queue from the Name Space.
    */
   {
      tGDEF_INT8  dispatcher_message_queue_name[DISPATCHER_QUEUE_NAME_SIZE] = {0};
      
      strncpy(dispatcher_message_queue_name, MESSAGE_QUEUE_PREFIX, sizeof(dispatcher_message_queue_name));
      strcat(dispatcher_message_queue_name, DISPATCHER_MESSAGE_QUEUE);

      /*
       * Unlink and Remove POSIX Message Queue from the Name Space
       */
     (void)mq_unlink(dispatcher_message_queue_name);     
   }
   
   /* 
    * Close the log file
    */
   EWOD_WriteMessage("AOCS Process Exit");
   EWOD_CloseLog();
   return;
}

/*!
 * \brief      TaskInit
 *
 *             AOCS Task initialisation function
 *
 *
 ******************************************************************************/
static void TaskInit(void)
{
   
#ifdef USE_LOGGING_LIBRARY
   /*
    * Record within the AOCS Log the necessary file revision information.
    */
   aocs_process_log_file_revision_information();
#endif
   
   EWOD_WriteMessage("AOCS Process Started\n");

   /* ensure mode is in standby */
   AMOH_ForceMode(MODE_SBM);

   /* initialise telemetry */
   ATTC_Init();

   /* initialise AOCS variables etc. */
   AINT_Init(GDEF_TRUE);

   /*
    * RETURN
    */
   return;
}

#ifdef USE_LOGGING_LIBRARY
/*!
 *  \brief Routine to add file revision information into the Safety Log File
 *  \param  void - None
 *  \return void - None
 */

static void aocs_process_log_file_revision_information(void)
{
   tGDEF_INT8  CvsTag[] = "CVSTAG: " CVSTAG;

   tGDEF_INT8 MissionTimeStamp[] = "Compiled on " __DATE__ " at " __TIME__;
   log_entry(REC_COMMENT, PROCESS_NAME, sizeof(PROCESS_NAME));
   log_entry(REC_COMMENT, CvsTag, strlen(CvsTag));
   log_entry(REC_COMMENT, MissionTimeStamp, sizeof(MissionTimeStamp));
#ifdef FLIGHTSW_RELEASE_TAG
   log_entry(REC_COMMENT, FLIGHTSW_RELEASE_TAG, sizeof(FLIGHTSW_RELEASE_TAG));
#else
#pragma message ("Please define FLIGHTSW_RELEASE_TAG in your mission.h to match your CVS tag")
#endif

   /*
    * RETURN
    */
   return;
} /* static void aocs_process_log_file_revision_information(void) */
#endif

/*!
 *  \brief Routine to register the AOCS Process with the CAN Server.
 *
 *         Registers Telecommand and Telemetry service queues and a generic receive message queue.
 *         Block waits for Received CAN messages.
 *
 *  \param  void - None
 *  \return void - None
 */
void aocs_can_thread(void)
{
   /*
    * Repeatedly read messages received on the AOCS Process CAN Message Queue.
    */
   for(;;)
   {
      teCANA_APIRes           eRes         = eCANA_OK;
      tsCANS_MsgsToCANClients stCanMsg     = {0};
    
      /* Block wait for CAN messages */
      eRes = CANA_GetMsgsFromQueue(&stCANReg, AOCS_PROCESS_CAN_RECEIVE_QUEUE_IDENTIFIER, eCANS_WAIT, &stCanMsg);
      if(eRes == eCANA_OK)
      {
         /*
          * Process the received message
          */
         switch(stCanMsg.eMsgType)
         {
            case eCANS_CLIENT_DATAGRAM:
               {
                  switch(stCanMsg.uMsgData.stCans_datagram.u1Type)
                  {
                     case CANI_Telecommand_Request:
                        {
                           /*
                            *  Record the CAN source and CAN destination address information only with the CAN telecommand Channel and CAN telecommand Value
                            */
                           tsTcChanValue *can_telecommand_data = ((tsTcChanValue*)(stCanMsg.uMsgData.stCans_datagram.a1Buffer));
                           /*lint --e(792) void cast of void expression */
                           assert(can_telecommand_data != NULL);
               
                           {
                              tsCANS_datagram stReplyDatagram = {0};
                              tGDEF_UINT16 channel            = can_telecommand_data->u2Chan;
                              tGDEF_UINT32 value              = can_telecommand_data->u4Value;
               
                              /*
                               * We have received a telecommand - must be from another RTP client, as its the only way to receive a Raw datagram
                               */
               
                              /*
                               * Process the packet and generate a response
                               */
                              can_telecommand_data->u4Value = ATTC_ProcessCmd(channel, value);
                              stReplyDatagram               = stCanMsg.uMsgData.stCans_datagram;              /* Its the (updated) value field & channel we want) */
                              stReplyDatagram.u1Src         = CANADDR_ADCS_PROCESS;
                              stReplyDatagram.u1Dest        = stCanMsg.uMsgData.stCans_datagram.u1Src;
                              stReplyDatagram.u1Length      = CANI_DGM_BUF_SZ;
               
                              if(can_telecommand_data->u4Value != GDEF_SUCCESS)
                              {
                                 stReplyDatagram.u1Type = CANI_TC_Nak_Response;
                              }
                              else
                              {
                                 stReplyDatagram.u1Type = CANI_TC_Ack_Response;
                              }
               
                              /*
                               * Send the response
                               */
                              CANA_send_datagram(&stCANReg, eCANS_LO_PRI_Q, &stReplyDatagram);
                           }
                        }
                        break;
         
                     default:
                        (void)snprintf(EWOD_sLogMsg, sizeof(EWOD_sLogMsg), "Unknown datagram received: %d\n", stCanMsg.uMsgData.stCans_datagram.u1Type);
                        EWOD_WriteMessage(EWOD_sLogMsg);
#ifdef REALTIME_DEBUG
                        (void)printf("%s %s\r\n", PROCESS_NAME, EWOD_sLogMsg);
#endif /* REALTIME_DEBUG */
                        break;
                  } /* switch(stCanMsg.uMsgData.stCans_datagram.eType) */
               }
               break;
      
            case eCANS_CLIENT_TELECOMMAND:
               {
                  tsCANS_TcTlmSingle stTcResponse = {0};
         
                  /* 
                   * Copy/Set the common return values 
                   */
                  tGDEF_UINT16 u2Chan  = stCanMsg.uMsgData.stCans_client_tc.stTcTlm.u2Chan;
                  tGDEF_UINT32 u4Value = stCanMsg.uMsgData.stCans_client_tc.stTcTlm.u4Value;
         
                  stTcResponse.u1Src          = CANADDR_ADCS_PROCESS;
                  stTcResponse.u4TransId      = stCanMsg.uMsgData.stCans_client_tc.u4TransId;
                  stTcResponse.stTcTlm.u1Dest = stCanMsg.uMsgData.stCans_client_tc.u1Src;
                  stTcResponse.stTcTlm.u2Chan = u2Chan;
         
#ifdef REALTIME_DEBUG
                  (void)printf("Received TC %d %d %d\n",u2Chan,u4Value,stTcResponse.u4TransId);
#endif /* REALTIME_DEBUG */
                  
                  /*
                   * Process the CAN Telecommand Received
                   */
                  stTcResponse.stTcTlm.u4Value = ATTC_ProcessCmd(u2Chan, u4Value);
                  if(stTcResponse.stTcTlm.u4Value != GDEF_SUCCESS)
                  {
#ifdef REALTIME_DEBUG
                     (void)printf("TC NAK C:%d V:%d",u2Chan,u4Value);
#endif /* REALTIME_DEBUG */
                     stTcResponse.stTcTlm.e1ReturnStatus = eCANS_TRAN_TC_NAK & 0x000000FF;
                  }
                  else
                  {
#ifdef REALTIME_DEBUG
                     (void)printf("OK %d\n",stTcResponse.stTcTlm.e1ReturnStatus);
#endif /* REALTIME_DEBUG */
                    stTcResponse.stTcTlm.e1ReturnStatus = eCANS_TRAN_TC_OK & 0x000000FF;
                  }

                  /*
                   * Send the response over the CANBUS
                   */
                  CANA_tc_response(&stCANReg, &stTcResponse);
               }
               break;

            case eCANS_TELECOMMAND_RETURNS:
               {
                  tGDEF_UINT32 node = stCanMsg.uMsgData.stCans_transaction_replies.asTcTlmResponses[0].u1Dest;

#ifdef REALTIME_DEBUG
                  if(node == CANADDR_AIM0)
                  {
                    (void)printf("TC ACK from: %d, @ %d\n",node, time(NULL));
                  }
#endif
                  if (HMGR_MtqActuateCmdSent == GDEF_FALSE)
                  {
                    /* 
                     * no command was sent in this second 
                     */
                    ACON_ClearTimer(TIMER_MTQ_ACK);
                  }
                  
                  if((node == CANADDR_AIM0) || (node == CANADDR_AIM1))
                  {
                    ACON_SetTimer(TIMER_MTQ_ACK);
                  }
                  else if((node == CANADDR_WHEEL0) || (node == CANADDR_WHEEL1) || (node == CANADDR_WHEEL2) ||(node == CANADDR_WHEEL3))
                  {
                    ACON_SetTimer(TIMER_WHEEL_ACK);
                  }
         
                  HMGR_numAcksPending--;
               }
               break;

            case eCANS_TELECOMMAND_MSG_ACK:
               {
                  /*
                   * Some nodes return an intermidiate eCANS_TRAN_WAITING_CAN before the final ACK (above case), check on that just in case
                   */
                  if(stCanMsg.uMsgData.stCans_client_telecommands.asTc[0].e1ReturnStatus != eCANS_TRAN_WAITING_CAN)
                  {
                     (void)snprintf(EWOD_sLogMsg, sizeof(EWOD_sLogMsg), "Did not receive waiting CAN %d\n", stCanMsg.uMsgData.stCans_client_telecommands.asTc[0].e1ReturnStatus);        
                     EWOD_WriteMessage(EWOD_sLogMsg);
                  }
               }
               break;

            case eCANS_TELECOMMAND_MSG_NAK:
               {
#ifdef REALTIME_DEBUG
                  (void)printf("TC NAK S:%d D:%d C:%d\n",stCanMsg.uMsgData.stCans_client_telecommands.u1Src,stCanMsg.uMsgData.stCans_client_telecommands.asTc[0].u1Dest,stCanMsg.uMsgData.stCans_client_telecommands.asTc[0].u2Chan);
#endif /* REALTIME_DEBUG */
         
                  HMGR_numNaks++;
               }
               break;

            case eCANS_CLIENT_TELEMETRY_REQUEST:
               {
                  tsCANS_TcTlmResponses_from_client stTcResponse = {0};
                  tGDEF_UINT32                      *pData       = NULL;
                  teGDEF_FUNC_STATUS                status       = GDEF_FAILURE;
                  tGDEF_UINT8                       loop_count   = 0;
                  
                  stTcResponse.u1Src     = CANADDR_ADCS_PROCESS;
                  stTcResponse.u4TransId = stCanMsg.uMsgData.stCans_client_telemetry_request.u4TransId;
                  stTcResponse.u1Count   = stCanMsg.uMsgData.stCans_client_telemetry_request.u1Count;
         
                  for(; loop_count < stCanMsg.uMsgData.stCans_client_telemetry_request.u1Count; loop_count++)
                  {
                     /* 
                      * Copy/Set the common return values 
                      */
                     tGDEF_UINT16 u2Chan = stCanMsg.uMsgData.stCans_client_telemetry_request.asTlm[loop_count].u2Chan;
         
                     stTcResponse.asTcTlm[loop_count].u1Dest = stCanMsg.uMsgData.stCans_client_telemetry_request.u1Src;
                     stTcResponse.asTcTlm[loop_count].u2Chan = u2Chan;
                    
                     /*
                      * Process the CAN Telemetry Request Received
                      */
                     status = ATTC_TlmHandler(u2Chan, &pData);
                     stTcResponse.asTcTlm[loop_count].u4Value = *pData;
         
                     if(status != GDEF_SUCCESS)
                     {
                        stTcResponse.asTcTlm[loop_count].e1ReturnStatus = eCANS_TRAN_TLM_NAK & 0x000000FF;
                     }
                     else
                     {
                        stTcResponse.asTcTlm[loop_count].e1ReturnStatus = eCANS_TRAN_TLM_32BIT & 0x000000FF;
                     }
                  }
                  
                  /*
                   * Send the tlm response over the CANBUS
                   */
                  CANA_tlm_response_frame(&stCANReg, &stTcResponse);
               }
               break;

#ifdef TEST_ON_DEBUG
            case eCANS_TELEMETRY_UNSOLICITED:
#else
            case eCANS_STREAMED_DATA:
#endif
               {
                  /*
                   * Process the received Message, can only be STREAM type.
                   */
                  switch(stCanMsg.uMsgData.stCans_datagram.u1Src)
                  {
                     /*
                      *  Process incoming STREAM (note: AOCS people by STREAM, they mean ctrl byte 32, but 32 is unsol tlm for GPS people and
                      *  30 is for STREAM)
                      *
                      *  Ctrl B    |   30       |   32
                      *  ----------------------------------
                      *  AOCS      | Not used   |  STREAM
                      *  ----------------------------------
                      *  GPS       | STREAM     |  Unsol Tlm
                      */
                     case CANADDR_AIM0:
                        /*
                         * Set 1st "unsol tlm received" timer
                         */
                        if(setTimerFlag && HMGR_isUnsolSampleCycle)
                        {
                           ACON_SetTimer(TIMER_FIRST_UNSOL);
                          setTimerFlag = GDEF_FALSE;
                        }
            
                        if(HMGR_isUnsolSampleCycle)
                        {
                           ACON_SetTimer(TIMER_LAST_UNSOL);
                        }
            
                        UTLM_ProcAimTlm(AIM0, (tGDEF_UCHAR*)&(stCanMsg.uMsgData.stCans_datagram.a1Buffer)); /* Process the received telemetry */
           
#ifdef PERFORM_CAN_TIME_SYNCHRONISATION_WITHIN_AOCS_SHELL
                        {
                           ts_aocs_shell_configuration *aocs_shell_configuration = &aocs_shell_configuration_table;
                           tGDEF_INT32                 status                    = EERROR;
                          
                           /*lint --e(506) Constant value Boolean */
                           assert(aocs_shell_configuration != NULL);
                           /*lint --e(792) void cast of void expression */
                           assert(aocs_shell_configuration->firewall_head == FIREWALL_PATTERN);
                           /*lint --e(792) void cast of void expression */
                           assert(aocs_shell_configuration->firewall_tail == FIREWALL_PATTERN);
                           
                           status = timer_settime(aocs_shell_configuration->aocs_shell_aim_check_timer_id, CLOCK_REALTIME, &aocs_shell_configuration->aocs_shell_aim_check_timer, NULL);
                           /*lint --e(792) void cast of void expression */
                           assert(status == EOK); 
                        }
#endif /* PERFORM_CAN_TIME_SYNCHRONISATION_WITHIN_AOCS_SHELL */
                        break;

                     case CANADDR_AIM1:
                        /*
                         * Set 1st "unsol tlm received" timer
                         */
                        if (setTimerFlag && HMGR_isUnsolSampleCycle)
                        {
                           ACON_SetTimer(TIMER_FIRST_UNSOL);
                           setTimerFlag = GDEF_FALSE;
                        }
            
                        if (HMGR_isUnsolSampleCycle)
                        {
                           ACON_SetTimer(TIMER_LAST_UNSOL);
                        }
                        UTLM_ProcAimTlm(AIM1, (tGDEF_UCHAR*)&(stCanMsg.uMsgData.stCans_datagram.a1Buffer));
                        break;

                     case CANADDR_WHEEL0:
                        /*
                         * Set 1st "unsol tlm received" timer
                         */
                        if(setTimerFlag && HMGR_isUnsolSampleCycle)
                        {
                           ACON_SetTimer(TIMER_FIRST_UNSOL);
                           setTimerFlag = GDEF_FALSE;
                        }
            
                        if(HMGR_isUnsolSampleCycle)
                        {
                           ACON_SetTimer(TIMER_LAST_UNSOL);
                        }
                        UTLM_ProcWheelTlm(WHEEL0, (tGDEF_UCHAR*)&(stCanMsg.uMsgData.stCans_datagram.a1Buffer));
                        break;
            
                     case CANADDR_WHEEL1:
                        /*
                         * Set 1st "unsol tlm received" timer
                         */
                        if(setTimerFlag && HMGR_isUnsolSampleCycle)
                        {
                           ACON_SetTimer(TIMER_FIRST_UNSOL);
                           setTimerFlag = GDEF_FALSE;
                        }
            
                        if(HMGR_isUnsolSampleCycle)
                        {
                           ACON_SetTimer(TIMER_LAST_UNSOL);
                        }
                        UTLM_ProcWheelTlm(WHEEL1, (tGDEF_UCHAR*)&(stCanMsg.uMsgData.stCans_datagram.a1Buffer));
                        break;
            
                     case CANADDR_WHEEL2:
                        /*
                         * Set 1st "unsol tlm received" timer
                         */
                        if(setTimerFlag && HMGR_isUnsolSampleCycle)
                        {
                           ACON_SetTimer(TIMER_FIRST_UNSOL);
                           setTimerFlag = GDEF_FALSE;
                        }
            
                        if(HMGR_isUnsolSampleCycle)
                        {
                           ACON_SetTimer(TIMER_LAST_UNSOL);
                        }
                        UTLM_ProcWheelTlm(WHEEL2, (tGDEF_UCHAR*)&(stCanMsg.uMsgData.stCans_datagram.a1Buffer));
                        break;

                     case CANADDR_WHEEL3:
                        /*
                         * Set 1st "unsol tlm received" timer
                         */
                        if(setTimerFlag && HMGR_isUnsolSampleCycle)
                        {
                           ACON_SetTimer(TIMER_FIRST_UNSOL);
                           setTimerFlag = GDEF_FALSE;
                        }
            
                        if(HMGR_isUnsolSampleCycle)
                        {
                           ACON_SetTimer(TIMER_LAST_UNSOL);
                        }
                        UTLM_ProcWheelTlm(WHEEL3, (tGDEF_UCHAR*)&(stCanMsg.uMsgData.stCans_datagram.a1Buffer));
                        break;

#ifdef _INC_STAR_TRACKER_
                     case CANADDR_DPU0:
                        if (setTimerFlag && HMGR_isUnsolSampleCycle)
                        {
                           ACON_SetTimer(TIMER_FIRST_UNSOL);
                           setTimerFlag = GDEF_FALSE;
                        }
            
                        if(HMGR_isUnsolSampleCycle)
                        {
                           ACON_SetTimer(TIMER_LAST_UNSOL);
                        }
                        UTLM_ProcDpuTlm(DPU0, &(stCanMsg.uMsgData.stCans_unsol_telemetry_return.stTlm.u4Value));
                        break;
#endif /* _INC_STAR_TRACKER_ */

#ifdef _INC_SECOND_DPU_ /* No Gyro in TDS, only 1 DPU */
                     case CANADDR_DPU1:
                        if(setTimerFlag && HMGR_isUnsolSampleCycle)
                        {
                           ACON_SetTimer(TIMER_FIRST_UNSOL);
                           setTimerFlag = GDEF_FALSE;
                        }
            
                        if(HMGR_isUnsolSampleCycle)
                        {
                           ACON_SetTimer(TIMER_LAST_UNSOL);
                        }
                        UTLM_ProcDpuTlm(DPU1,  aocs_message.message_data.can_datagram.buffer);
                        break;
            
                     case CANADDR_GYR:
                        ttcState.tlm.gyr = aocs_message.telemetry_data.tlm.gyr;
                        if(setTimerFlag && HMGR_isUnsolSampleCycle)
                        {
                           ACON_SetTimer(TIMER_FIRST_UNSOL);
                           setTimerFlag = GDEF_FALSE;
                        }
            
                        if(HMGR_isUnsolSampleCycle)
                        {
                           ACON_SetTimer(TIMER_LAST_UNSOL);
                        }
                        break;
#endif /* _INC_SECOND_DPU_ */
                     default:
                        (void)snprintf(EWOD_sLogMsg, sizeof(EWOD_sLogMsg), "AOCS_AocsShell unknown node -%d\n",stCanMsg.uMsgData.stCans_datagram.u1Src);
                        EWOD_WriteMessage(EWOD_sLogMsg);
                        break;
                  }/* END case: STREAM_DATA*/
               }
               break;

            default:
               break;
         } /* END case: CAN_MSG_DATAGRAM_TYPE */
      }
   }

   /*
    * NOT REACHED
    */
   /*lint --e(792) void cast of void expression */
   /*lint --e(527) unreachable */
   assert(GDEF_FALSE);
}

/*!
 * \brief      InitAocs
 *
 *             Initialises a new AOCS cycle
 *
 *
 ******************************************************************************/
void InitAocs(void)
{
   /*
    * Update the ARO
    */
   AROH_Update();

   /*
    * Allow a mode change for new cycle if drive file has been loaded
    */
   if (AFIH_driveFileNum != AFIH_NO_DRIVE_FILE)
   {
      AMOH_UpdateMode();
   }
   /*
    * Initialise data gathering for new cycle
    */
   UTLM_InitDataGathering();

   setTimerFlag = GDEF_TRUE;

   /*
    * RETURN
    */
   return;
}

/*---------------------------------------------------------------------------
 * Unit test utilities & wrappers - start
 */

/*---------------------------------------------------------------------------
 * Unit test utilities & wrappers - end
 */

/*---------------------------------------------------------------------------
 * End of File
 */
