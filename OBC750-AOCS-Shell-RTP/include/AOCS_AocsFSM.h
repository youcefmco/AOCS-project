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
 * Last Update : $Date: 2015/11/17 14:22:06 $
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/include/AOCS_AocsFSM.h,v $
 * Revision    : $Revision: 1.5 $
 *
 * History:
 *
 * $Log: AOCS_AocsFSM.h,v $
 * Revision 1.5  2015/11/17 14:22:06  ytrichakis
 * Updated for new messgae handling + sked
 *
 * Revision 1.3  2013/05/24 13:03:17  ytrichakis
 * Fixed AIM0 problem and added TC return handling from SKED RTP
 *
 * Revision 1.2  2013/04/23 15:49:20  ytrichakis
 * Added block wait till SKED msgQ is ready to open
 *
 *
 ******************************************************************************/

#ifndef __AOCS_AOCSFSM_H
#define __AOCS_AOCSFSM_H

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
#define NSEC_TO_TICKS                  (10000000)
#define NANOSECONDS_IN_ONE_MILLISECOND (0x000F4240)
#define NANOSECONDS_TO_MILLISECONDS(ns)   \
   ns / NANOSECONDS_IN_ONE_MILLISECOND

/*---------------------------------------------------------------------------
 * Typedefs (shared within this module)
 */
typedef enum
{
   IDLE,                                                      /*!< AOCS Shell Idle State                               */
   TLMR,                                                      /*!< AOCS Shell All Pulses Received-check TLM State      */
   RUNA,                                                      /*!< AOCS Shell Run AOCS cycle State                     */
   AOCS_FSM_NUMBER_OF_STATES                                  /*!< Number Of AOCS Shell FSM States                     */
} te_aocs_shell_fsm_state;

typedef enum
{
   PPS_RECEIVED,                                              /*!< PPS Received Event                                  */
   ALL_PPS_RECEIVED,                                          /*!< All PPS Received Event                              */
   AOCS_MESSAGE,                                              /*!< AOCS Cycle to Start Event                           */
   ACTUATE,                                                   /*!< Issue Actuate command Event                         */
   AOCS_FSM_NUMBER_OF_EVENTS                                  /*!< Number Of AOCS Shell Events                         */
} te_aocs_shell_fsm_event;

typedef struct
{
   tGDEF_UINT32               firewall_head;                  /*!< Firewall Pattern to help detect Memory Corruption   */
   pthread_mutex_t            aocs_shell_protection_mutex;    /*!< AOCS Shell Protection Mutex                         */
   te_aocs_shell_fsm_state    aocs_shell_fsm_state;           /*!< AOCS Shell FSM State                                */
   te_aocs_shell_fsm_event    aocs_shell_fsm_event;           /*!< AOCS Shell FSM Event                                */
   time_t                     aocs_shell_event_action_time;   /*!< AOCS Shell FSM Event Action Time                    */
#ifdef FSM_TIMER   
   timer_t                    aocs_shell_posix_timer_id;      /*!< POSIX FSM Timer ID                                  */
   struct itimerspec          aocs_shell_posix_timer;         /*!< POSIX FSM Timer                                     */
#endif /* FSM_TIMER */
   timer_t                    aocs_shell_pps_timer_id;        /*!< AOCS Shell PPS Timer ID                             */
   struct itimerspec          aocs_shell_pps_timer;           /*!< AOCS Shell PPS Timer                                */
   timer_t                    aocs_shell_actuation_timer_id;  /*!< AOCS Shell Actuation Timer ID                       */
   struct itimerspec          aocs_shell_actuation_timer;     /*!< AOCS Shell Actuation Timer                          */
#ifdef PERFORM_CAN_TIME_SYNCHRONISATION_WITHIN_AOCS_SHELL 
   timer_t                    aocs_shell_aim_check_timer_id;  /*!< AOCS Shell AIM Check Timer ID                       */
   struct itimerspec          aocs_shell_aim_check_timer;     /*!< AOCS Shell AIM Check Timer                          */
#endif /* PERFORM_CAN_TIME_SYNCHRONISATION_WITHIN_AOCS_SHELL */   
   tGDEF_UINT32               firewall_tail;                  /*!< Firewall Pattern to help detect Memory Corruption   */
} ts_aocs_shell_configuration;

/*---------------------------------------------------------------------------
 * Data - (shared within this module)
 */
extern time_t            AOCS_timeNow;
extern teGDEF_BOOLEAN    HMGR_isUnsolSampleCycle;

/*---------------------------------------------------------------------------
 * Module Function Prototypes
 */

/*!
 *  \brief Routine action Events into the AOCS Finite State Machine.
 *  \param  ts_aocs_shell_configuration *const - Pointer to the AOCS Configuration Structure
 *  \return tGDEF_UINT8                         - Success / Failure Indication (NOT Implemented)
 */
extern tGDEF_INT16 aocs_shell_process_event(ts_aocs_shell_configuration *const aocs_shell_configuration);

/*!
 *  \brief Routine to deliver an EVENT to the AOCS Finite State Machine.
 *  \param  ts_aocs_shell_configuration *const - Pointer to the AOCS Configuration Structure
 *  \return tGDEF_UINT8                        - Success / Failure Indication (NOT Implemented)
 */
extern tGDEF_INT16 aocs_shell_deliver_event(ts_aocs_shell_configuration *const aocs_shell_configuration);

void        AOCS_SetPeriod(tGDEF_UINT8 period);
tGDEF_UINT8 AOCS_GetPeriod(void);
void        set_wakeup_AIM_time(tGDEF_UINT16 time);

#ifdef __cplusplus
/* *INDENT-OFF* */
}
/* *INDENT-ON* */
#endif

#endif /* __AOCS_AOCSFSM_H */

/*!--------------------------------------------------------------------------
 * End of file
 */
