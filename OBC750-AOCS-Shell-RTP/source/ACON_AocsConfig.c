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
 * Last Update : $Date: 2016/03/14 10:47:46 $
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/source/ACON_AocsConfig.c,v $
 * Revision    : $Revision: 1.5 $
 *
 * History:
 *
 * $Log: ACON_AocsConfig.c,v $
 * Revision 1.5  2016/03/14 10:47:46  ytrichakis
 * Register with the SKED library
 *
 * Revision 1.2  2013/04/17 14:17:39  ytrichakis
 * Check in with fixing the identation only
 *
 * Revision 1.1  2013/04/11 13:42:21  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <mqueue.h>
#include <pthread.h>

#include "GDEF_GlobDefs.h"
#include "Adcs_mission.h"
#include "Adcs_ttcDefs.h"

#include "AOCS_aocsShell.h"
#include "AOCS_AocsFSM.h"
//#include "EWOD_EwodHandler_OBC750.h"
#include "EWOD_EwodHandler.h"

#include "CANS_Interface.h"
#include "CANS_API.h"
#include "CANA_ServerApiLib.h"

#include "AINT_AdcsInterface.h"
#include "ATTC_AocsTTC.h"
#include "AMOH_AocsModeHandler.h"
#include "AROH_AroHandler.h"
#include "RCON_Reconfigure.h"
#include "HMGR_HardwareManager.h"
#include "ACON_AocsConfig.h"
#include "AFIH_Filehandler.h" // to check if config file exists

#ifdef _INC_GEOLOC
#include "AGEO_AocsGeolocation.h"
#endif

/*-----------------------------------------------------------------------
 * Defines and Macros
 */
#define ACON_NUM_TIMERS 6
#define MAX_TIMER_TICKS (255)

/*---------------------------------------------------------------------------
 * Typedefs
 */
typedef struct
{
   tGDEF_INT16 ticks;
   tGDEF_INT32 secs;
} tsTimeStamp;

/*---------------------------------------------------------------------------
 * Private Function Prototypes (declared as static; used only within this module)
 */
static void ACON_UpdateTimersA(const tsTimeStamp * pRef);
static void ACON_UpdateTimersB(const tsTimeStamp * pRef);
/*static void ACON_SetSampleRate(teACON_RATE_TYPE type, tGDEF_UINT8 value);
static tGDEF_UINT8 ACON_GetRate(teACON_RATE_TYPE type, tGDEF_UINT8 value);*/

/*---------------------------------------------------------------------------
 * Local Data (declared as static; used only within this module)
 */
static tGDEF_UINT8  ACON_aocsRate[NUM_MODES] = AOCS_RATE;

/*---------------------------------------------------------------------------
 * Global Data
 */
tsTimeStamp  ACON_Timer[ACON_NUM_TIMERS];
tGDEF_UINT32 ACON_TimerTlm[NUM_TIMERS_TLM];

/*---------------------------------------------------------------------------
 * Public Functions
 */


/* \brief      ACON_GetShellStatus
 *
 *          Gets the Shell Status
 *
 * \return status bit mask
 *
 ******************************************************************************/
tGDEF_UINT32 ACON_GetShellStatus()
{
   tGDEF_UINT32 status = 0UL;
   tGDEF_UINT16 modeNo = 0;
   tGDEF_UINT16 bitNo  = 0;
   tGDEF_UINT32 dummy  = 0;
   tGDEF_UINT32 retVal = 0;

   /* shell parameters bits 0 - 14 */
   status  |= (tGDEF_UINT32) (AINT_pTtc->test.isEnabled      & 0x01);
   status  |= (tGDEF_UINT32) (AINT_GetModeReqState()   & 0x01) << 1;
   status  |= (tGDEF_UINT32) (RCON_GetState()  & 0X07) << 2;
#ifdef INC_GEOLOC
   status  |= (tGDEF_UINT32) (AGEO_GetState()  & 0x01) << 5;
#endif
   status  |= (tGDEF_UINT32) (AMOH_GetMode()           & 0x0F) << 6;
   status  |= (tGDEF_UINT32) (AOCS_GetPeriod()         & 0x0F) << 10;
   status  |= (tGDEF_UINT32) (AROH_isReconfigEnabled   & 0x01) << 14;
   status  |= (AROH_errors & 0x1F) << 15;
   status  |= (tGDEF_UINT32) (AMOH_GetSafeMode() &0x01) << 20;
   status  |= (tGDEF_UINT32) (AMOH_GetUnsolModeFlag() &0x01) << 21;

   bitNo = 22;
   for (modeNo = 0; modeNo < NUM_MODES; modeNo++)
   {
      retVal = (tGDEF_UINT32)AFIH_GetFnum(modeNo,&dummy);
      status |= (tGDEF_UINT32)((retVal & 0x01) << bitNo );
      bitNo ++;
   }

   for (modeNo = 0; modeNo < NUM_SAFE_MODES; modeNo++)
   {
      retVal = (tGDEF_UINT32)AFIH_GetFnum(modeNo + ACON_SAFE_SKED_OFFSET,&dummy);
      status |= (tGDEF_UINT32)((retVal & 0x01) << bitNo );
      bitNo ++;
   }

   return status;
}


/* \brief      ACON_UpdateTimersA
 *
 *          Updates timers
 *
 * \return status bit mask
 *
 ******************************************************************************/
void ACON_UpdateTimersA(const tsTimeStamp * pRef)
{
   tGDEF_UINT8 timerNo = 0;
   tGDEF_INT16 ticks   = 0;

   /* reset timer tlm */
   ACON_TimerTlm[TIMERS_A] = 0UL;

   /* update the timer */
   for(timerNo = 0; timerNo < 4; timerNo++)
   {
      /* check timer secs is greater than ref */
      if(ACON_Timer[timerNo].secs >= pRef->secs)
         ticks = ACON_Timer[timerNo].ticks + (tGDEF_INT16) (ACON_Timer[timerNo].secs - pRef->secs) * 100;
      else
         ticks =  MAX_TIMER_TICKS;

      /* store tick value */
      ACON_TimerTlm[TIMERS_A] |= (tGDEF_UINT32) (ticks & 0x00FF) << (8 * timerNo);
   }
}

/* \brief      ACON_UpdateTimersB
 *
 *
 *
 * \return status bit mask
 *
 ******************************************************************************/
void ACON_UpdateTimersB(const tsTimeStamp * pRef)
{
   tGDEF_UINT8 timerNo = 0;
   tGDEF_INT16 ticks   = 0;

   /* reset timer tlm */
   ACON_TimerTlm[TIMERS_B] = 0UL;

   /* only valid for unsolicted modes */
   if(GDEF_TRUE == AMOH_isUnsolMode)
   {
      /* timer B values */
      for(timerNo = 4; timerNo < ACON_NUM_TIMERS; timerNo++)
      {
         /* check if we have recorded a valid timer */
         if(ACON_Timer[timerNo].secs >= (pRef->secs-1))
            ticks = ACON_Timer[timerNo].ticks + (tGDEF_INT16) (ACON_Timer[timerNo].secs - (pRef->secs -1)) * 100;
         else
            ticks = MAX_TIMER_TICKS;

         /* store tick value */
         ACON_TimerTlm[TIMERS_B]  |= (tGDEF_UINT32) (ticks & 0x00FF) << (8 * (timerNo -4));
      }
   }
}



/* \brief     ACON_SetTimer
 *
 *          Stores the time in ticks for a given timer
 *                  no of ticks relative to 0 ticks from the second of the first timer)
 *
 *
 ******************************************************************************/
void ACON_SetTimer(tGDEF_UINT8 timerNo)
{
   struct timespec tm;

   if(timerNo == TIMER_START_ALGS)
   {
      /* update timer tlm A using last alg start timer as a reference */
      ACON_UpdateTimersA(&ACON_Timer[TIMER_START_ALGS]);

      /* update the timer */
      clock_gettime(CLOCK_REALTIME,&tm);
      ACON_Timer[timerNo].secs  = tm.tv_sec;
      ACON_Timer[timerNo].ticks = tm.tv_nsec / NSEC_TO_TICKS;

      /* update timer tlm B using update alg start timer as a reference */
      ACON_UpdateTimersB(&ACON_Timer[TIMER_START_ALGS]);
   }
   else
   {
      /* update the timer */
      clock_gettime(CLOCK_REALTIME,&tm);
      ACON_Timer[timerNo].secs  = tm.tv_sec;
      ACON_Timer[timerNo].ticks = tm.tv_nsec / NSEC_TO_TICKS;
   }
}
/* \brief     ACON_ClearTimer
 *
 *          Clear timer to start time
 *
 *
 ******************************************************************************/
void ACON_ClearTimer(tGDEF_UINT8 timerNo)
{
   ACON_Timer[timerNo].secs = ACON_Timer[TIMER_START_ALGS].secs;
   ACON_Timer[timerNo].ticks = 0;
}

/* \brief     Stes the rate for the given mode
 *
 *
 * \return void
 ******************************************************************************/
void ACON_SetRate(tGDEF_UINT8 mode)
{
   if(mode >= NUM_MODES)
      return;

   EWOD_SetPeriod(mode);
   AOCS_SetPeriod(ACON_aocsRate[mode]);
   AINT_SetSampleRate(ACON_aocsRate[mode]);
}


