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
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/source/AOCS_AocsFSM.c,v $
 * Revision    : $Revision: 1.14 $
 *
 * History:
 *
 * $Log: AOCS_AocsFSM.c,v $
 * Revision 1.14  2016/03/17 15:57:36  ytrichakis
 * Create FSM thread before CAN and check for EEROR in dispatcher queue
 *
 * Revision 1.10  2014/09/17 10:53:20  ytrichakis
 * Do not send cmds to AIM when in SAFE SBM too
 *
 * Revision 1.9  2014/09/15 12:56:50  ytrichakis
 * Changes for TDS-1 Flight to include accepting drive file from automation process and removing sending sync cmds to AIM when in SBM
 *
 * Revision 1.8  2013/09/03 13:35:21  ytrichakis
 * Removed calls to logging library as now all logs are done in the internel ADYYMMDD file
 *
 * Revision 1.7  2013/07/18 15:56:53  ytrichakis
 * fixed wheel bug(DR#14548) and applied the same solution in case of failed AIM
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
 *
 ******************************************************************************/


/*---------------------------------------------------------------------------
 * Includes
 */
/*lint -elib(???) ignore library files */
#include <assert.h>
#include <errno.h>
#include <mqueue.h>
#include <pthread.h>
#include <sched.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <signal.h>
#ifdef AOCS_SYSTEM_PROFILER
#include <wvLib.h>
#endif
/*lint +elib(???) ignore library files */
#include "Adcs_IntDefs.h"
#include "CANS_Interface.h"
#include "CANS_API.h"
#include "CANA_ServerApiLib.h"

#include "GDEF_GlobDefs.h"
#include "mission.h"
#include "Adcs_mission.h"
#include "endian_transposition.h"

#include "AOCS_AocsShell.h"
#include "HMGR_HardwareManager.h"
#include "EWOD_EwodHandler.h"
#include "ACON_AocsConfig.h"
#include "AROH_AroHandler.h"
#include "AINT_AdcsInterface.h"
#include "AOCS_AocsFSM.h"
#include "AOCS_Message.h"

/*-----------------------------------------------------------------------
 * Defines and Macros
 */

/*---------------------------------------------------------------------------
 * Typedefs
 */

/*---------------------------------------------------------------------------
 * Private Function Prototypes (declared as static; used only within this module)
 */
static tGDEF_INT16 pps_received(ts_aocs_shell_configuration *const aocs_shell_configuration);
static tGDEF_INT16 all_pps_received(ts_aocs_shell_configuration *const aocs_shell_configuration);
static tGDEF_INT16 RunAOCS(ts_aocs_shell_configuration *const aocs_shell_configuration);
static tGDEF_INT16 Actuate(ts_aocs_shell_configuration *const aocs_shell_configuration);

/*---------------------------------------------------------------------------
 * Local Data (declared as static; used only within this module)
 */
static tGDEF_UINT32 AOCS_ticksNow                            = 0;
static tGDEF_UINT8  AOCS_maxCycle                            = AOCS_DEFAULT_CYCLE_TIME;
static tGDEF_INT8   No_of_PPS_sent                           = 0;

#ifdef PERFORM_CAN_TIME_SYNCHRONISATION_WITHIN_AOCS_SHELL
static tGDEF_UINT8    No_all_PPS_received                    = 0;
static tGDEF_UINT8    AIM_PPS			                         = 0;
static tGDEF_UINT16   wake_up_aim							       = 100;
static teGDEF_BOOLEAN sync_flag                              = GDEF_FALSE;
#define SEND_SYNC_TIME (15)
typedef struct
{
   time_t       seconds;
   tGDEF_UINT16 milliseconds; 
}__attribute__((__packed__)) ts_time_service_can_synchronisation_message_overlay;
#endif /* PERFORM_CAN_TIME_SYNCHRONISATION_WITHIN_AOCS_SHELL */

static te_aocs_shell_fsm_state aocs_shell_state_table [AOCS_FSM_NUMBER_OF_EVENTS][AOCS_FSM_NUMBER_OF_STATES] =
{
   /*
    * RUNAOCS ---------------------------------
    * CHECKTLM --------------------------      |
    * IDLE-------------------------      |     |
    *                              |     |     |
    */
   /* PPS_RECEIVED           */ {IDLE, IDLE, IDLE},
   /* ALL_PPS_RECEIVED       */ {TLMR, IDLE, IDLE},
   /* ADCS_MESSAGE           */ {IDLE, RUNA, IDLE},
   /* ACTUATE                */ {IDLE, IDLE, IDLE}
};

static tGDEF_INT16 (*aocs_shell_event_table [AOCS_FSM_NUMBER_OF_EVENTS][AOCS_FSM_NUMBER_OF_STATES])(ts_aocs_shell_configuration *const aocs_shell_configuration) =
{
   /*
    * RUNAOCS ------------------------------------------------
    * CHECKTLM --------------------------------------         |
    * IDLE-----------------------------              |        |
    *                                  |             |        |
    */
   /* PPS_RECEIVED           */ {pps_received,     NULL,    NULL   },
   /* ALL_PPS_RECEIVED       */ {all_pps_received, NULL,    NULL   },
   /* ADCS_MESSAGE           */ {NULL,             RunAOCS, NULL   },
   /* ACTUATE                */ {NULL,             NULL,    Actuate}
};


/*---------------------------------------------------------------------------
 * Global Data
 */
time_t         AOCS_timeNow            = 0;
teGDEF_BOOLEAN HMGR_isUnsolSampleCycle = GDEF_FALSE;

/*---------------------------------------------------------------------------
 * External Function Prototypes
 */
extern tsCANS_RegRtn *getCANReg(void);
extern tsAOCS_TTC    ttcState;
extern mqd_t         dispatcher_thread_communications_channel;
/*---------------------------------------------------------------------------
 * Public Functions
 */

/*!
 *  \brief Routine action Events into the AOCS Finite State Machine.
 *  \param  ts_aocs_shell_configuration *const - Pointer to the AOCS Configuration Structure
 *  \return tGDEF_UINT8                         - Success / Failure Indication (NOT Implemented)
 */
tGDEF_INT16 aocs_shell_process_event(ts_aocs_shell_configuration *const aocs_shell_configuration)
{
   tGDEF_INT32 return_code = 0;
   tGDEF_INT32 status      = EERROR;

   /*lint --e(506) Constant value Boolean */
   assert(aocs_shell_configuration != NULL);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_head == FIREWALL_PATTERN);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_tail == FIREWALL_PATTERN);
   
#ifdef AOCS_FSM_FLIGHT_RECORDER
   (void)printf("%s: Entered Function [aocs_shell_process_event]\r\n",PROCESS_NAME);
   (void)printf("%s: [aocs_shell_process_event] Event %s\r\n",PROCESS_NAME, display_current_finite_state_machine_event(aocs_shell_configuration->aocs_shell_fsm_event));
   (void)printf("%s: [aocs_shell_process_event] State %s\r\n",PROCESS_NAME, display_current_finite_state_machine_state(aocs_shell_configuration->aocs_shell_fsm_state));
#endif /* AOCS_FSM_FLIGHT_RECORDER */
   
   /* 
    * AOCS Protection Mutex Strategy:
    * 
    * We will obtain the AOCS Protection Mutex at this point.
    * 
    * The design intent is that the only method to execute Finite State Machine Event Handling Routine's is through this Finite State Machine Event Handler.  
    *  
    * If SET_PTHREAD_MUTEX_RECURSIVE is defined we will (as a Belt & Braces approach) each Finite State Machine Event Handler Routine will also attempt to obtain the GPS Protection Mutex.
    */
   status = pthread_mutex_lock(&aocs_shell_configuration->aocs_shell_protection_mutex);
   /*lint --e(792) void cast of void expression */
   assert(status == EOK);
   
   {
      te_aocs_shell_fsm_state current_state = aocs_shell_configuration->aocs_shell_fsm_state;

      /*
       * Move to the new state
       */
      aocs_shell_configuration->aocs_shell_fsm_state = aocs_shell_state_table [aocs_shell_configuration->aocs_shell_fsm_event] [aocs_shell_configuration->aocs_shell_fsm_state] ;

      /*
       * Perform the next required Event
       */
      if(aocs_shell_event_table[aocs_shell_configuration->aocs_shell_fsm_event] [current_state] != NULL)
      {
         return_code = (*(aocs_shell_event_table [aocs_shell_configuration->aocs_shell_fsm_event] [current_state]))(aocs_shell_configuration);
      }
      else
      {
         /*
          * If an event has a null handler, it is a "can't happen" event, therefore, log a message
          */

         /*
          * Add an entry into the AOCS Shell Process Log.
          */
         snprintf(EWOD_sLogMsg, sizeof(EWOD_sLogMsg), "Invalid STATE: %d EVENT %d\n", current_state, aocs_shell_configuration->aocs_shell_fsm_event);
         
         EWOD_WriteMessage(EWOD_sLogMsg);
#ifdef REALTIME_DEBUG
         (void)printf("Invalid STATE: %d, EVENT: %d", current_state,aocs_shell_configuration->aocs_shell_fsm_event);
#endif /* REALTIME_DEBUG */
      } /* if(memory_dump_event_table[memory_dump_configuration->memory_dump_fsm_event] [current_state] != NULL) */
   }
   
   /*
    * Release the AOCS Shell Protection Mutex.
    */
   status = pthread_mutex_unlock(&aocs_shell_configuration->aocs_shell_protection_mutex);
   /*lint --e(792) void cast of void expression */
   assert(status == EOK);

#ifdef AOCS_FSM_FLIGHT_RECORDER
   (void)printf("%s: Exit Function [aocs_shell_process_event]\r\n",PROCESS_NAME);
#endif /* AOCS_FSM_FLIGHT_RECORDER */

   /*
    * RETURN
    */
   return return_code;
}

/*!
 *  \brief Routine to deliver an EVENT to the AOCS Finite State Machine.
 *  \param  ts_aocs_shell_configuration *const - Pointer to the AOCS Configuration Structure
 *  \return tGDEF_UINT8                        - Success / Failure Indication (NOT Implemented)
 */
tGDEF_INT16 aocs_shell_deliver_event(ts_aocs_shell_configuration *const aocs_shell_configuration)
{
   tGDEF_INT16 return_code = 0;

   /*lint --e(506) Constant value Boolean */
   assert(aocs_shell_configuration != NULL);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_head == FIREWALL_PATTERN);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_tail == FIREWALL_PATTERN);

   {
      ts_aocs_message aocs_shell_message = {0};
      tGDEF_INT32     status             = EERROR;

      aocs_shell_message.aocs_message_type                 = eAOCSTimerMessage;
      aocs_shell_message.aocs_message_data.message_pointer = ((void*)aocs_shell_configuration);

      status = mq_send(dispatcher_thread_communications_channel, ((const char *)(&aocs_shell_message)), sizeof(ts_aocs_message), 0);
      /*lint --e(792) void cast of void expression */
      assert(status == EOK);
   }

   /*
    * RETURN
    */
   return return_code;
}

/*!
 *  \brief Routine handle received pulse
 *  \param  ts_aocs_shell_configuration *const  - Pointer to the AOCS shell Configuration Structure
 *  \return tGDEF_UINT8                         - Success / Failure Indication (NOT Implemented)
 */
static tGDEF_INT16 pps_received(ts_aocs_shell_configuration *const aocs_shell_configuration)
{
   struct timespec aocs_posix_timer = {0};
   tGDEF_INT16     return_code      = 0;

   /*lint --e(506) Constant value Boolean */
   assert(aocs_shell_configuration != NULL);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_head == FIREWALL_PATTERN);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_tail == FIREWALL_PATTERN);
   
   /*
    * Synchronise the AOCS units every 60sec
    */
   clock_gettime(CLOCK_REALTIME, &aocs_posix_timer);
   AOCS_timeNow  = aocs_posix_timer.tv_sec;
   AOCS_ticksNow = aocs_posix_timer.tv_nsec / NSEC_TO_TICKS;

   if(aocs_posix_timer.tv_nsec > NSEC_TO_TICKS)
   {
      tGDEF_INT32 status = EERROR;
      
      status = timer_cancel(aocs_shell_configuration->aocs_shell_pps_timer_id);
      /*lint --e(792) void cast of void expression */
      assert(status == EOK);
      
      /*
       * Resynchronise if there was a big time jump (due to OBCSync usually)
       */
      aocs_shell_configuration->aocs_shell_pps_timer.it_value.tv_sec  = 0;
      aocs_shell_configuration->aocs_shell_pps_timer.it_value.tv_nsec = NSEC_TO_SEC - aocs_posix_timer.tv_nsec;

      status = timer_settime(aocs_shell_configuration->aocs_shell_pps_timer_id, CLOCK_REALTIME, &aocs_shell_configuration->aocs_shell_pps_timer, NULL);
      /*lint --e(792) void cast of void expression */
      assert(status == EOK);
   }
   
   No_of_PPS_sent++;

#ifdef PERFORM_CAN_TIME_SYNCHRONISATION_WITHIN_AOCS_SHELL
   /*
    * When we send timesync we need to send TC 5,1 to both AIMs again to tell to everyone that a new cycle starts now for everyone.
    */
   if((sync_flag == GDEF_TRUE) && ((ttcState.mode.current != MODE_SBM) && (ttcState.mode.current != MODE_SAFE)))
   {
      tsTcTlmNodeValue          stTc           = {0};
      tsCANA_TcTlmRequestParams stTcReqParams  = {0};
      teCANA_APIRes             CANA_APIRes    = 0;
      
      /*
       * As agreed with AJ, once we send the sync cmd, go back to 0 PPS so that everyone starts from fresh
       */
      No_of_PPS_sent--;
      sync_flag = GDEF_FALSE;
      
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
      
      if (ttcState.algConfig.mag.failedAIM != 2)
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
#endif

   /*
    * Initialise parameters for a new cycle
    */
   if(AOCS_maxCycle <= No_of_PPS_sent)
   {
      aocs_shell_configuration->aocs_shell_fsm_event         = ALL_PPS_RECEIVED;
      aocs_shell_configuration->aocs_shell_event_action_time = time(NULL);
      aocs_shell_deliver_event(aocs_shell_configuration);
      No_of_PPS_sent = 0;
   }

   /*
    * Last second before calling algs set usol tlm timers
    */
   if((AOCS_maxCycle-1) == No_of_PPS_sent)
   {
      HMGR_isUnsolSampleCycle = GDEF_TRUE;
   }
   else
   {
      HMGR_isUnsolSampleCycle = GDEF_FALSE;
   }

   if(((AOCS_maxCycle-1) == No_of_PPS_sent) && (AOCS_maxCycle > 1))
   {
      /*
       * Clear UTLM buffers at penultimate cycle before calling algs
       */
      UTLM_InitDataGathering();
   }
   
   /*
    * RETURN
    */
   return return_code;
}


/*!
 *  \brief Routine handle the case when all required PPS are received
 *  \param  ts_aocs_shell_configuration *const  - Pointer to the AOCS shell Configuration Structure
 *  \return tGDEF_UINT8                         - Success / Failure Indication (NOT Implemented)
 */
static tGDEF_INT16 all_pps_received(ts_aocs_shell_configuration *const aocs_shell_configuration)
{
   tGDEF_INT16 return_code = 0;
   
   /*lint --e(506) Constant value Boolean */
   assert(aocs_shell_configuration != NULL);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_head == FIREWALL_PATTERN);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_tail == FIREWALL_PATTERN);

   /*
    * check if all expected unsolicited data has been received
    */
   if(UTLM_IsAllDataReceived() != GDEF_TRUE)
   {
      AROH_SetDecFlag(AROH_UNSOL_TLM_ERR);
   }

   aocs_shell_configuration->aocs_shell_fsm_event         = AOCS_MESSAGE;
   aocs_shell_configuration->aocs_shell_event_action_time = time(NULL);
   aocs_shell_deliver_event(aocs_shell_configuration);

   /*
    * RETURN
    */
   return return_code;
}

/*!
 *  \brief Routine to call the AOCS function, check for errors, and implement the ARO counter
 *  \param  ts_aocs_shell_configuration *const  - Pointer to the AOCS shell Configuration Structure
 *  \return tGDEF_UINT16                        - Success / Failure Indication (NOT Implemented)
 */
static tGDEF_INT16 RunAOCS(ts_aocs_shell_configuration *const aocs_shell_configuration)
{
   teGDEF_FUNC_STATUS aocsStatus = GDEF_FAILURE;  /* return value from AOCS_Go() */
   tGDEF_INT32        status     = EERROR;
   
   /*lint --e(506) Constant value Boolean */
   assert(aocs_shell_configuration != NULL);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_head == FIREWALL_PATTERN);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_tail == FIREWALL_PATTERN);

   /* Set RUN START Timer */
   ACON_SetTimer(TIMER_START_ALGS);
#ifdef AOCS_SYSTEM_PROFILER
   wvEvent(101,NULL,0);
#endif
   /* If we haven't received an ACK for all commands, set the ARO counter */
   if (HMGR_CheckNoAckErrors() != GDEF_TRUE)
   {
#ifdef REALTIME_DEBUG
      (void)printf("Set ACK Flag, not all ACKs received\r\n");
#endif /* REALTIME_DEBUG */
      AROH_SetDecFlag(AROH_CMD_ACK_ERR);
   }
   
#ifdef PERFORM_CAN_TIME_SYNCHRONISATION_WITHIN_AOCS_SHELL
   if ( (ttcState.mode.current != MODE_SBM) && (ttcState.mode.current != MODE_SAFE) )
   {
      No_all_PPS_received++;
      AIM_PPS++;
      /*
       * When we are here for the "SEND_SYNC_TIME"th, ie 15*4 = 60sec
       * send sync message to Node 0.
       */
      if (No_all_PPS_received >= SEND_SYNC_TIME)
      {
         /*
          * Distribute the current OBC Time over the Spacecraft CANBUS.
          */
         struct timespec aocs_posix_timer = {0};
          
         /*
          * Obtain the current OBC System which we will distribuate to the rest of the Spacecraft over the CANBUS.
          */
         if(clock_gettime(CLOCK_REALTIME, &aocs_posix_timer) != ERROR)
         {
            tsCANS_datagram                                     CANS_datagram                                     = {0};
            teCANA_APIRes                                       CANA_APIRes                                       = 0;
            ts_time_service_can_synchronisation_message_overlay *time_service_can_synchronisation_message_overlay = (ts_time_service_can_synchronisation_message_overlay*)CANS_datagram.a1Buffer;
            
            CANS_datagram.u1Src    = CANADDR_ADCS_PROCESS;
            CANS_datagram.u1Dest   = CANADDR_BROADCAST;
            CANS_datagram.u1Length = sizeof(CANS_datagram.a1Buffer);
            CANS_datagram.u1Type   = CANI_Time_Set_Command;

            /*
             * Build the Time Synchronisation CAN Datagram.
             */
            time_service_can_synchronisation_message_overlay->seconds      = endian_transpose_signed_thirty_two_bit_integer(aocs_posix_timer.tv_sec);
            time_service_can_synchronisation_message_overlay->milliseconds = (tGDEF_UINT16)endian_transpose_signed_sixteen_bit_integer((NANOSECONDS_TO_MILLISECONDS(aocs_posix_timer.tv_nsec)));
             
            /*
             * Transmit the Time Synchronisation CAN Datagram over the Spacecraft CANBUS.
             */
            CANA_APIRes = CANA_send_datagram(getCANReg(), eCANS_LO_PRI_Q, &CANS_datagram);
            /*lint --e(792) void cast of void expression */
            assert(CANA_APIRes == eCANA_OK);

#ifdef REALTIME_DEBUG
             (void)printf("Send datagram with s:%d, ms:%d\n",time_service_posix_timer.tv_sec,time_service_posix_timer.tv_nsec);
#endif /* REALTIME_DEBUG */
          } /* if(clock_gettime(CLOCK_REALTIME, &time_service_posix_timer) != ERROR) */
         
         No_all_PPS_received = 0;
      }
      
      if (AIM_PPS >= wake_up_aim)
      {
          tsTcTlmNodeValue          stTc           = {0};
          tsCANA_TcTlmRequestParams stTcReqParams  = {0};
          teCANA_APIRes             CANA_APIRes    = 0;
          sync_flag = GDEF_TRUE;
          
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
          if (ttcState.algConfig.mag.failedAIM != 1)
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
           
          if (ttcState.algConfig.mag.failedAIM != 2)
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
          AIM_PPS = 0;
      	
      }
   }

#endif
   
   /* prepare data for algorithms */
   HMGR_PrepareData();
   /* RUN THE AOCS ALGORITHMS */
   aocsStatus = AINT_AlgManager();
   /*
    * what to do? previously it was always returning TRUE even if algs failed..
    */
#ifdef AOCS_SYSTEM_PROFILER
   wvEvent(102,NULL,0);
#endif
   /* Set RUN FINISH Timer */
   ACON_SetTimer(TIMER_END_ALGS);

   InitAocs();
   
   /*
    * Start the 200ms delay before we fire the actuator, as requested by AOCS.
    */
   status = timer_settime(aocs_shell_configuration->aocs_shell_actuation_timer_id, 0, &aocs_shell_configuration->aocs_shell_actuation_timer, NULL);
   /*lint --e(792) void cast of void expression */
   assert(status == EOK);  
   
   /*
    * RETRUN
    */
   return GDEF_TRUE;
}

/*!
 *  \brief Routine to send the commands to the actuator and update the AOCS ewods
 *  \param  ts_aocs_shell_configuration *const  - Pointer to the AOCS shell Configuration Structure
 *  \return tGDEF_UINT16                        - Success / Failure Indication (NOT Implemented)
 */
static tGDEF_INT16 Actuate(ts_aocs_shell_configuration *const aocs_shell_configuration)
{
   tGDEF_INT16 return_code = 0;
   
   /*lint --e(506) Constant value Boolean */
   assert(aocs_shell_configuration != NULL);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_head == FIREWALL_PATTERN);
   /*lint --e(792) void cast of void expression */
   assert(aocs_shell_configuration->firewall_tail == FIREWALL_PATTERN);

   /*
    * Call this only if Runaocs  was succesfull and we are ticks<500ms. I need to finish runaocs, get time and check that the last ms value
    * of the cycle was not >500 and still in the current second, ie 0 for me since i reset PPS when runaocs is called
    * , if it is >500 || sec>0 then set ARO and dont do HMGR_Actuate() but set AROH_ACTUATE_ERR
    *
    * YT Update: Not needed anymore since 750's proc power guarantees algs will finish before 500ms!! (max measured time was 1.3ms for CPM)
    */
   HMGR_Actuate();

   /* 
    * Update the EWOD
    */
   EWOD_Update();

   return return_code;
}

/*!
 *  \brief Routine to set the AOCS period
 *  \param  tGDEF_UINT8  - period requested
 *  \return void         - None
 */
void AOCS_SetPeriod(tGDEF_UINT8 period)
{
   snprintf(EWOD_sLogMsg, sizeof(EWOD_sLogMsg), "AOCS Period updated to %d\n", period);
  
   EWOD_WriteMessage(EWOD_sLogMsg);
   AOCS_maxCycle = period;
   
   /*
    * RETURN
    */
   return;
}

/*!
 *  \brief Routine to return the AOCS period
 *  \param  void         - None
 *  \return tGDEF_UINT8  - current period
 */
tGDEF_UINT8 AOCS_GetPeriod(void)
{
   return AOCS_maxCycle;
}

void set_wakeup_AIM_time(tGDEF_UINT16 time)
{
	wake_up_aim = time;
}

