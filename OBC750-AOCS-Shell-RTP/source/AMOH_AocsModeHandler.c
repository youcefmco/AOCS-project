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
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/source/AMOH_AocsModeHandler.c,v $
 * Revision    : $Revision: 1.8 $
 *
 * History:
 *
 * $Log: AMOH_AocsModeHandler.c,v $
 * Revision 1.8  2016/03/14 10:47:46  ytrichakis
 * Register with the SKED library
 *
 * Revision 1.5  2013/09/06 15:47:57  ytrichakis
 * removed printf
 *
 * Revision 1.4  2013/07/18 15:56:53  ytrichakis
 * fixed wheel bug(DR#14548) and applied the same solution in case of failed AIM
 *
 * Revision 1.3  2013/06/24 13:13:25  ytrichakis
 * Removed TODO's as agreed between YT and AJ (07/06/13)
 *
 * Revision 1.2  2013/04/17 14:17:40  ytrichakis
 * Check in with fixing the identation only
 *
 * Revision 1.1  2013/04/11 13:42:21  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "GDEF_GlobDefs.h"
#include "Adcs_mission.h"

#include "AINT_AdcsInterface.h"
#include "ACON_AocsConfig.h"
#include "HMGR_HardwareManager.h"
#include "AOCS_AocsShell.h"
#include "AROH_AroHandler.h"
//#include "EWOD_EwodHandler_OBC750.h"
#include "EWOD_EwodHandler.h"
#include "AMOH_AocsModeHandler.h"


/*-----------------------------------------------------------------------
 * Defines and Macros
 */
#define AMOH_TIME_OFF 0

/*---------------------------------------------------------------------------
 * Typedefs
 */

/*---------------------------------------------------------------------------
 * Private Function Prototypes (declared as static; used only within this module)
 */
static void          SetAocsMode(void);
static unsigned char CheckValidModetransition(unsigned char newMode);

/*---------------------------------------------------------------------------
 * Local Data (declared as static; used only within this module)
 */
/* Current mode */
static tGDEF_UINT8 AMOH_mode = MODE_SBM;

/* var to store the requested mode before a transition */
static tGDEF_UINT8 AMOH_reqMode = MODE_SBM;

/* flag to indicate if this is a safe mode*/
static teGDEF_BOOLEAN AMOH_isSafeMode = GDEF_FALSE;

static tGDEF_UINT8 AMOH_transState = AMOH_TRANS_IDLE;

/* mode lookup tables */
static teGDEF_BOOLEAN AMOH_unsolModeLookup[NUM_MODES]                = AOCS_UNSOL_MODES;
static const char     AMOH_modeTransitionTable[NUM_MODES][NUM_MODES] = AOCS_MODE_TRANSISTION_TABLE;

/*---------------------------------------------------------------------------
 * Global Data
 */
/* flag to indicate current mode is unsolicted mode */
teGDEF_BOOLEAN AMOH_isUnsolMode = GDEF_TRUE;

/*!
 * \brief      <function brief description>
 *
 *             <function detailed description>
 *
 * \param  <param name>  <param description>
 * \return <return value description>
 *
 ******************************************************************************/
tGDEF_UINT8 AMOH_GetMode()
{
   return AMOH_mode;
}

#ifdef _DONT_USE_
teGDEF_BOOLEAN AMOH_GetResetFlag()
{
   return AMOH_isModeReset;
}

void AMOH_ClearResetFlag()
{
   AMOH_isModeReset = GDEF_FALSE;
}
#endif

/*!
 * \brief      <get requested mode number>
 *
 *             <function detailed description>
 *
 * \param  <param name>  <param description>
 * \return <return value description>
 *
 ******************************************************************************/
tGDEF_UINT8 AMOH_GetReqMode()
{
   return AMOH_reqMode;
}

/*!
 * \brief      <get flag to tell if it is unsol mode>
 *
 *             <function detailed description>
 *
 * \param  <param name>  <param description>
 * \return <return value description>
 *
 ******************************************************************************/
teGDEF_BOOLEAN AMOH_GetUnsolModeFlag()
{
   return AMOH_isUnsolMode;
}

/*!
 * \brief      <get array of unsolicited modes - used to check back as unsolicited mode array can be changed by TC>
 *
 *             <function detailed description>
 *
 * \param  <param name>  <param description>
 * \return <return value description>
 *
 ******************************************************************************/
tGDEF_UINT32 AMOH_GetUnsolModeArray()
{
   tGDEF_UCHAR  modeNo = 0;
   tGDEF_UINT32 retVal = 0;

   for (modeNo = 0; modeNo < NUM_MODES; modeNo++)
   {
      retVal |= (0x01 & AMOH_unsolModeLookup[modeNo]) << modeNo;
   }

   return retVal;
}

/*!
 * \brief      <set values in unsol mode array>
 *
 *             <function detailed description>
 *
 * \param  <param name>  <param description>
 * \return <return value description>
 *
 ******************************************************************************/
void AMOH_SetUnsolMode( tGDEF_UINT8 mode, teGDEF_BOOLEAN value)
{
   if (value <= 1)
   {
      AMOH_unsolModeLookup[mode] = value;
   }
}

teGDEF_BOOLEAN AMOH_GetSafeMode()
{
   return AMOH_isSafeMode;
}

teGDEF_FUNC_STATUS AMOH_SetSafeMode( teGDEF_BOOLEAN value , tGDEF_UINT8 mode)
{
   /* if we set from OPS mode to safe mode then check that the current mode has a safe mode version */
   if (GDEF_TRUE == value)
   {
      if (mode >= NUM_SAFE_MODES)
         return GDEF_FAILURE;

   }

   AMOH_isSafeMode = value;

   /*re-initialise the mode*/
   AMOH_transState = AMOH_TRANS_REQ;

   return GDEF_SUCCESS;
}

/*!
 * \brief      <function brief description>
 *
 * sets the safe mode flag (if no safe mode version exists of current mode then rejects)
 * will also re-initialise a mode transition
 *             <function detailed description>
 *
 * \param  <param name>  <param description>
 * \return <return value description>
 *
 ******************************************************************************/
teGDEF_FUNC_STATUS AMOH_SafeModeTransition(tGDEF_UINT8 mode)
{
   teGDEF_FUNC_STATUS retVal = GDEF_SUCCESS;
   /* reset flags in shell */
   AINT_Init(GDEF_FALSE);

   /* set the safe state flag */
   retVal = AMOH_SetSafeMode(GDEF_TRUE, mode);
   if ( retVal != GDEF_SUCCESS)
   {
      return retVal;
   }

   /* update the mode */
   AMOH_ForceMode(mode);

   return retVal;
}

/*!
 * \brief      <function brief description>
 *
 *             <function detailed description>
 *
 * \param  <param name>  <param description>
 * \return <return value description>
 *
 ******************************************************************************/
void AMOH_UpdateMode()
{
   tGDEF_UINT8 SkedMode = 0;

   switch(AMOH_transState)
   {

   case AMOH_TRANS_IDLE:
      /* nothing to do so just return */
      break;

   case AMOH_TRANS_REQ:
   {
#ifdef REALTIME_DEBUG
      (void)printf("AMOH_TRANS_REQ\n");
#endif /* REALTIME_DEBUG */
      /* no config required in standby OPS, config might be required in safe standby (i.e. logging telemetry) */
      if( (MODE_SBM == AMOH_reqMode) && (GDEF_FALSE == AMOH_isSafeMode) )
      {
         EWOD_SetDefaultTlmType(MODE_SBM);
         /* standby mode so just update shell params */
         AMOH_transState = AMOH_TRANS_SHELL;
      }
      else
      {
         if (GDEF_FALSE == AMOH_isSafeMode )
         {
            SkedMode = AMOH_reqMode;
         }
         else
         {
            SkedMode = AMOH_reqMode + ACON_SAFE_SKED_OFFSET;
         }

         /* kick off the hw config sked for the mode */
         if(GDEF_SUCCESS != ACON_StartSked(SkedMode))
         {
#ifdef REALTIME_DEBUG
            (void)printf("Start SKED fail!\n");
#endif /* REALTIME_DEBUG */
            strcpy(EWOD_sLogMsg, "Unable to run config sked for mode [" );
            sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d",AMOH_reqMode);
            strcat(EWOD_sLogMsg, "] isSafeMode = " );
            sprintf( &EWOD_sLogMsg[strlen(EWOD_sLogMsg)],"%d",(int)AMOH_isSafeMode);
            strcat(EWOD_sLogMsg, "\n" );
            EWOD_WriteMessage(EWOD_sLogMsg);

            /* AJ - abort or carry on?? */
            AMOH_transState = AMOH_TRANS_IDLE;
         }
         else
         {
#ifdef REALTIME_DEBUG
            (void)printf("config started for mode %d\n",AMOH_reqMode);
#endif /* REALTIME_DEBUG */
            strcpy(EWOD_sLogMsg, "Started Config sked for mode [" );
            sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", AMOH_reqMode);
            strcat(EWOD_sLogMsg, "] isSafeMode = " );
            sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d",(int)AMOH_isSafeMode);
            strcat(EWOD_sLogMsg, "\n" );
            EWOD_WriteMessage(EWOD_sLogMsg);
            /* update state to config */
            AMOH_transState = AMOH_TRANS_CONFIG;
         }
      }
   }
   break;

   case AMOH_TRANS_CONFIG:
   {
#ifdef REALTIME_DEBUG
      (void)printf("AMOH_TRANS_CONF\n");
#endif /* REALTIME_DEBUG */
      // check configuration sked complete
      if(GDEF_FALSE == ACON_GetSkedStatus())
      {
#ifdef REALTIME_DEBUG
         (void)printf("config completed\n");
#endif /* REALTIME_DEBUG */
         strcpy(EWOD_sLogMsg, "Config sked completed for mode [" );
         sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d",AMOH_reqMode);
         strcat(EWOD_sLogMsg, "]\n" );
         EWOD_WriteMessage(EWOD_sLogMsg);
         /* update state to config */
         AMOH_transState = AMOH_TRANS_SHELL;
      }

      //AJ - timeout??

   }
   break;

   case AMOH_TRANS_SHELL:
   {
#ifdef REALTIME_DEBUG
      (void)printf("AMOH_TRANS_SHELL\n");
#endif /* REALTIME_DEBUG */
      /* set shell parameters */
      SetAocsMode();

      /* set init flag for algs */
      AINT_pTtc->mode.isReset = GDEF_TRUE;
      AINT_pTtc->mode.current  = AMOH_mode;
      AINT_pTtc->mode.isSafeMode = AMOH_isSafeMode;
      AINT_pTtc->mode.isUnsolicited = AMOH_isUnsolMode;
#ifdef _NOT_NEEDED_
      AINT_pTtc->mode.enableSnsPtr = AINT_GetSensorEnableFlags();
      AINT_pTtc->mode.enableActPtr = AINT_GetActuatorEnableFlags();
#endif
      strcpy(EWOD_sLogMsg, "Shell updated to mode [" );
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d",AMOH_mode);
      strcat(EWOD_sLogMsg, "]\n" );
#ifdef REALTIME_DEBUG
      (void)printf("Shell updated to %d\n",AMOH_mode);
#endif /* REALTIME_DEBUG */
      EWOD_WriteMessage(EWOD_sLogMsg);
      /* if in Standby mode then complete transition */
      if(AMOH_mode == MODE_SBM)
      {
         /* isReset flag needs to be cleared (unless aint_algmanager is adapted to go into the section where flag clears in standby)
          SF need to check if this flag is reset in standby ! */
#ifdef _NOT_NEEDED
         AINT_pTtc->mode.isReset = GDEF_FALSE;
         AMOH_transState = AMOH_TRANS_IDLE;
         break;
#endif
      }

      /* update state to config */
      AMOH_transState = AMOH_TRANS_ALGS;
   }
   /* fall through to trans algs immediately */

   case AMOH_TRANS_ALGS:
   {
#ifdef REALTIME_DEBUG
      (void)printf("AMOH_TRANS_ALGS\n");
#endif /* REALTIME_DEBUG */

      if(AINT_pTtc->mode.isReset == GDEF_FALSE)
      {
         strcpy(EWOD_sLogMsg, "Transition completed to mode [" );
         sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d",AMOH_mode);
         strcat(EWOD_sLogMsg, "]\n" );
#ifdef REALTIME_DEBUG
         printf("Transition to %d\n",AMOH_mode);
#endif
         EWOD_WriteMessage(EWOD_sLogMsg);

         /* update state to idle */
         AMOH_transState = AMOH_TRANS_IDLE;
      }

   }
   break;

   default:
   {
      strcpy(EWOD_sLogMsg, "Invalid trans state [" );
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d",AMOH_transState);
      strcat(EWOD_sLogMsg, "]\n" );
      EWOD_WriteMessage(EWOD_sLogMsg);

      AMOH_transState = AMOH_TRANS_IDLE;
   }
   break;
   }
}

/*! * \brief      <function brief description>
 *
 *             <function detailed description>
 *
 * \param  <param name>  <param description>
 * \return <return value description>
 *
 ******************************************************************************/
teGDEF_FUNC_STATUS AMOH_RequestMode(tGDEF_UINT8 newMode)
{
   time_t timeNow = 0;
   /* get current time */
   timeNow = time(NULL);

   /* check if already performing a mode transistion */
   if(AMOH_transState != AMOH_TRANS_IDLE)
   {
#ifdef REALTIME_DEBUG
      (void)printf("Invalid transition!\n");
#endif /* REALTIME_DEBUG */
      strcpy(EWOD_sLogMsg, "Unable to process mode trans req: [" );
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d",AMOH_mode);
      strcat(EWOD_sLogMsg, "] -> [" );
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)],"%d",newMode);
      strcat(EWOD_sLogMsg, "]\n" );
      EWOD_WriteMessage(EWOD_sLogMsg);

      return GDEF_FAILURE;
   }


   /* check if is different to current value */
   if (newMode != AMOH_mode)
   {
      /* Set the requested mode if it is a valid transistion */
      if (GDEF_TRUE == CheckValidModetransition(newMode))
      {
         /* set the request mode */
         AMOH_reqMode = newMode;

         /* set the transition state to requested */
         AMOH_transState = AMOH_TRANS_REQ;

         /* Initialise the requested mode */
         /* AJ - done on updateMode?? HMGR_InitMode(AMOH_reqMode); */
         /* print the load status */
         strcpy(EWOD_sLogMsg, "VALID mode trans req: [" );
         sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d",AMOH_mode);
         strcat(EWOD_sLogMsg, "] -> [" );
         sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d",newMode);
         strcat(EWOD_sLogMsg, "]\n" );
         EWOD_WriteMessage(EWOD_sLogMsg);
      }
      else
      {
#ifdef REALTIME_DEBUG
         (void)printf("Invalid request mode!\n");
#endif /* REALTIME_DEBUG */
         strcpy(EWOD_sLogMsg, "INVALID mode trans req: [" );
         sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d",AMOH_mode);
         strcat(EWOD_sLogMsg, "] -> [" );
         sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d",newMode);
         strcat(EWOD_sLogMsg, "]\n" );
         EWOD_WriteMessage(EWOD_sLogMsg);
      }
   }
   else
   {
      /* mode to be re-initialised so set time to present time */
      /* AMOH_transTime = timeNow; */
      /* mode to be re-initialised so set status to req */
      AMOH_transState = AMOH_TRANS_REQ;
   }

   /* return success */
   return GDEF_SUCCESS;
}


unsigned char CheckValidModetransition(unsigned char newMode)
{
   if (newMode > MAX_MODE_VAL)
   {
      return 0;
   }

   return AMOH_modeTransitionTable[AMOH_mode][newMode];
}

/* sets the mode regardless of requests or valid transistions */
void AMOH_ForceMode(unsigned char newMode)
{
   /* if curent mode transition occuruing then abort */
   ACON_AbortSked();

   /* print the load status */
   strcpy(EWOD_sLogMsg, "Force Mode change to " );
   sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d",newMode);
   strcat(EWOD_sLogMsg, "\n" );
   EWOD_WriteMessage(EWOD_sLogMsg);

   //other mode so update in usual manner
   AMOH_reqMode = newMode;
   AMOH_transState = AMOH_TRANS_REQ;
   AMOH_UpdateMode();



}

void SetAocsMode(void)
{
   /* check if a mode transition */
   if (AMOH_reqMode != AMOH_mode)
   {
      /* update the mode */
      AMOH_mode = AMOH_reqMode;

      /* reset the ARO Counter */
      AROH_Reset();

      /* hold off ARO during the mode transistion */
      AROH_SetHoldOff(AMOH_mode);
   }
   else
   {

   }

   /* update the sample time - AJ wait till after configuration? */
   ACON_SetRate(AMOH_mode);

   /* set flags for algs - needs to be done before */

   /* signal to units the new mode */
   HMGR_InitMode(AMOH_mode);

   /* set up sampling telemetry */
   if (AMOH_isUnsolMode)
   {
      /* initialise the unsolicted telemetry */
      UTLM_InitMode(AMOH_mode);
   }

   /* update EWOD logging for mode */
   EWOD_InitMode(AMOH_mode);

}

/*Find out if there is a config file registered for each of the safe modes*/
teGDEF_BOOLEAN AMOH_isAllSafeSkedsRegistered(void)
{
   tGDEF_UINT8 modeNo = 0;

   for (modeNo = 0; modeNo < NUM_SAFE_MODES; modeNo++)
   {
      /*check if there is a file registered for each safe mode*/
      if ( ACON_isSkedRegistered(modeNo + ACON_SAFE_SKED_OFFSET) == GDEF_FALSE)
      {
         strcpy(EWOD_sLogMsg, "AMOH_isAllSafeSkedsRegistered - missing sked for safe mode " );
         sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d",modeNo);
         strcat(EWOD_sLogMsg, "\n" );
         EWOD_WriteMessage(EWOD_sLogMsg);
         return GDEF_FALSE;
      }
   }

   return GDEF_TRUE;
}
