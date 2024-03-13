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
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/source/ATTC_AocsTTC.c,v $
 * Revision    : $Revision: 1.12 $
 *
 * History:
 *
 * $Log: ATTC_AocsTTC.c,v $
 * Revision 1.12  2016/03/17 15:57:36  ytrichakis
 * Create FSM thread before CAN and check for EEROR in dispatcher queue
 *
 * Revision 1.8  2013/09/02 15:51:33  ytrichakis
 * removed printfs
 *
 * Revision 1.7  2013/07/18 15:56:53  ytrichakis
 * fixed wheel bug(DR#14548) and applied the same solution in case of failed AIM
 *
 * Revision 1.6  2013/06/24 13:13:25  ytrichakis
 * Removed TODO's as agreed between YT and AJ (07/06/13)
 *
 * Revision 1.5  2013/06/21 13:54:39  ytrichakis
 * Send command 5 only to non-failed AIMs. Also set ewod closing tie to 1 day
 *
 * Revision 1.4  2013/05/24 13:03:27  ytrichakis
 * Fixed AIM0 problem and added TC return handling from SKED RTP
 *
 * Revision 1.3  2013/04/23 15:49:21  ytrichakis
 * Added block wait till SKED msgQ is ready to open
 *
 * Revision 1.2  2013/04/17 14:17:41  ytrichakis
 * Check in with fixing the identation only
 *
 * Revision 1.1  2013/04/11 13:42:21  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#include "GDEF_GlobDefs.h"
#include "Adcs_IntDefs.h"
#include "Adcs_mission.h"
#include "Adcs_ttcDefs.h"
#include "mission.h"

#include "ACON_Sked.h"
#include "AFIH_Filehandler.h"
#include "AOCS_AocsShell.h"
#include "AOCS_AocsFSM.h"
#include "ACON_AocsConfig.h"
#include "AMOH_AocsModeHandler.h"
#include "AINT_AdcsInterface.h"
#include "AROH_AroHandler.h"
//#include "EWOD_EwodHandler_OBC750.h"
#include "EWOD_EwodHandler.h"
#include "HMGR_HardwareManager.h"
#include "HINT_Aim.h"
#include "ATTC_AocsTTC.h"
#include "RCON_Reconfigure.h"


/*-----------------------------------------------------------------------
 * Defines and Macros
 */
/* index for first shell index */
#define ATTC_NUM_SHELL_TLM (MAX_SHELL_TLM_CHAN +1)

/*---------------------------------------------------------------------------
 * Typedefs
 */

/*---------------------------------------------------------------------------
 * Private Function Prototypes (declared as static; used only within this module)
 */
static teGDEF_FUNC_STATUS ATTC_ProcShellCmd(tGDEF_UINT16 cmd, tGDEF_UINT32 value);
static teGDEF_FUNC_STATUS ATTC_ProcAlgCmd(tGDEF_UINT16 cmd, tGDEF_UINT32 value);
static tGDEF_UINT32 *     GetShellTlm(tGDEF_UINT16 chan);

/*---------------------------------------------------------------------------
 * Local Data (declared as static; used only within this module)
 */
static tGDEF_UINT32 ATTC_shellTlm[ATTC_NUM_SHELL_TLM]               = {0};
static const  tGDEF_UINT16 ATTC_shellChanLookup[ATTC_NUM_SHELL_TLM] = SHELL_TLM_LOOKUP;

/*---------------------------------------------------------------------------
 * Global Data
 */


void ATTC_Init(void)
{
   tGDEF_UINT8 index = 0;

   /* ensure shell telemetry is up to date */
   for (index = 0; index < ATTC_NUM_SHELL_TLM; index++)
   {
      GetShellTlm(ATTC_shellChanLookup[index]);
   }
}


/*!
 * \brief      Returns Tlm requests
 *
 *             <function detailed description>
 *
 * \param  <param name>  <param description>
 * \return <return value description>
 *
 ******************************************************************************/
teGDEF_FUNC_STATUS ATTC_TlmHandler(tGDEF_UINT16 chan, tGDEF_UINT32 ** ppData)
{
   teGDEF_FUNC_STATUS retVal = GDEF_FAILURE;

   /* initialise output data */
   *ppData = NULL;

   if (chan >= SENSOR_TLM_OFFSET)
   {
      *ppData = AINT_GetSensorTlmData(chan);
   }
   else if (chan >= ALG_TLM_OFFSET)
   {
      *ppData = AINT_GetAlgTlmData(chan);
   }
   else if (chan >= ALGCMD_TLM_OFFSET)
   {
      *ppData = AINT_GetAlgCmdTlmData(chan);
   }
   else
   {
      *ppData = GetShellTlm(chan);
   }

   if (*ppData != NULL)
   {
      retVal = GDEF_SUCCESS;
   }

   /* telemetry channel not found so return 0 and log error */
   return retVal;
}

/*!
 * \brief       Process the commands received via CAN or ATCP
 *
 *             <function detailed description>
 *
 * \param  <param name>  <param description>
 * \return <return value description>
 *
 ******************************************************************************/
teGDEF_FUNC_STATUS ATTC_ProcessCmd(tGDEF_UINT16 cmd, tGDEF_UINT32 value)
{
   teGDEF_FUNC_STATUS retVal = GDEF_FAILURE;

   if(cmd == CMD_UPLOAD_FILE)
   {
      AFIH_ProcUploadedFile(value);
   }
   else if(cmd == CMD_LOAD_DRIVE_FILE)
   {
      retVal =  AINT_LoadDrive(value);
   }
   else if (cmd == CMD_EXIT_TASK)
   {
   	exit(0);
      retVal = GDEF_SUCCESS;
   }
   else if (cmd < ATTC_ALG_CMD_OFFSET)
   {
      retVal = ATTC_ProcShellCmd(cmd, value);
   }
   else
   {
      retVal = ATTC_ProcAlgCmd(cmd, value);
   }

   /* Log command */
   strcpy(EWOD_sLogMsg, "ATTC_ProcessCmd: cmd [");
   sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", cmd);
   strcat(EWOD_sLogMsg, "], val [ " );
   sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", value);
   strcat(EWOD_sLogMsg, "]\n" );
   EWOD_WriteMessage(EWOD_sLogMsg);

   return retVal;
}

teGDEF_FUNC_STATUS ATTC_ProcShellCmd(tGDEF_UINT16 cmd, tGDEF_UINT32 value)
{
   /*unpack data from value if needed*/
   /*modes*/
   tGDEF_UINT8 mode12 = (tGDEF_UINT8) (value >> 12);
   tGDEF_UINT8 mode8  = (tGDEF_UINT8) (value >> 8);
   tGDEF_UINT8 mode4  = (tGDEF_UINT8) (value >> 4) ;

   /*unit No*/
   tGDEF_UINT8 unitNo = (tGDEF_UINT8) ((0x00F0UL & value) >> 4); /* bits 4-7 */

   /*cast arg in different format*/
   tGDEF_UINT16   arg16   = (tGDEF_UINT16)   (value & 0x0000FFFF);
   tGDEF_UINT16   arg12   = (tGDEF_UINT16)   (value & 0x00000FFF);
   tGDEF_UINT8    arg8    = (tGDEF_UINT8)    (value & 0x000000FF);
   teGDEF_BOOLEAN argbool = (teGDEF_BOOLEAN) (value & 0x00000001);

   switch (cmd)
   {
   case CMD_TEST_STATE:
      AINT_SetTestState(arg16);
      break;
#ifdef _INC_AUTO_TRANS_
   case CMD_AUTO_MODE_STATE:
      AINT_SetModeReqState(argbool);
      break;
#endif
#if 0
   case CMD_DNLK_RATE:
      ADLK_SetRate(value);
      break;
#endif
   case CMD_SET_ARO_P:
      AROH_SetCounterP(arg16);
      break;
   case CMD_SET_ARO_Q:
      AROH_SetCounterQ(arg16);
      break;
   case CMD_SET_ARO_MAX:
      AROH_SetCounterMax(mode12, arg12);
      break;
   case CMD_ARO_HOLDOFF:
      AROH_SetHoldoffVal(mode12, arg12);
      break;
   case CMD_RECONFIG_STATE:
      RCON_SetState(arg8);
      break;
   case CMD_ENABLE_RECONFIG:
      AROH_Enable(argbool);
      break;
   case CMD_FDIR_CHECKS:
      AINT_SetFDIREnableFlags(mode8,argbool);
      break;
   case CMD_EWOD_PERIOD:
      EWOD_SetWodPeriodArray(mode8, arg8);
      break;
   case CMD_EWOD_ADD_TYPE:
      EWOD_AddTlmType(mode8, arg8);
      break;
   case CMD_EWOD_DEL_TYPE:
      EWOD_RemoveTlmType(mode8, arg8);
      break;
   case CMD_EWOD_DEFAULT_TYPE:
      EWOD_SetDefaultTlmType(arg8);
      break;
#if 0
   case CMD_SYNCH_PERIOD:
      ACOM_SetSynchPeriod((time_t) value);
      break;
#endif
   case CMD_CLOSE_LOG:
      EWOD_CloseLog();
      break;
   case CMD_ENABLE_LOG_ZIP:
      EWOD_ZipFile(argbool);       /* zip log file when finished */
      break;
   case CMD_AOCS_MODE:
      /* request mode change only if have a drive file loaded and all config skeds for safe mode are loaded */
      if ( (AFIH_NO_DRIVE_FILE != AFIH_driveFileNum) && (AMOH_isAllSafeSkedsRegistered() == GDEF_TRUE) )
      {
         AMOH_RequestMode(arg8);
      }
      else
      {
         EWOD_WriteMessage("ATTC_AocsTTC - Unable to request mode\n");
      }
      break;
   case CMD_FORCE_AOCS_MODE:
      if (value <= MAX_MODE_VAL)
      {
         AMOH_ForceMode(arg8);
      }
      break;
   case CMD_SAFE_MODE:
      AMOH_SetSafeMode(argbool, AMOH_GetMode());
      break;
   case CMD_SET_UNSOL_MODE:
      AMOH_SetUnsolMode(mode4, argbool);
      break;
   case CMD_LOG_MODE_ENABLE_FLAGS:
      AINT_Set_LogMode(mode4, argbool);
      break;
   case CMD_SAMPLE_PERIOD:
      AOCS_SetPeriod(arg8);
      break;
#if 0
   case CMD_GO_DELAY:
      AOCS_SetStart(arg8);
      break;
#endif
   case CMD_SNS_ENABLE_MTM:
      AINT_SetSensorEnableState(HMGR_SNS_MTM, mode8, unitNo, argbool);
      break;
   case CMD_SNS_ENABLE_SUNSENSOR:
      AINT_SetSensorEnableState(HMGR_SNS_SAS, mode8, unitNo, argbool);
      break;
   case CMD_SNS_ENABLE_MWHEEL:
      AINT_SetSensorEnableState(HMGR_SNS_MWHL, mode8, unitNo, argbool);
      break;

#if _FC_NUM_SWHL != 0
   case CMD_SNS_ENABLE_SWHEEL:
      AINT_SetSensorEnableState(HMGR_SNS_SWHL, mode8, unitNo, argbool);
      break;
#endif

#if _FC_NUM_FSS != 0
   case CMD_SNS_ENABLE_FSS:
      AINT_SetSensorEnableState(HMGR_SNS_FSS, mode8, unitNo, argbool);
      break;
#endif

#if _FC_NUM_GYR != 0
   case CMD_SNS_ENABLE_GYR:
      AINT_SetSensorEnableState(HMGR_SNS_GYR, mode8, unitNo, argbool);
      break;
#endif
   case CMD_LOG_MTM:
      HMGR_SetSensorLogState(HMGR_SNS_MTM, unitNo, argbool);
      break;
   case CMD_LOG_SUNSENSOR:
      HMGR_SetSensorLogState(HMGR_SNS_SAS, unitNo, argbool);
      break;
   case CMD_LOG_MWHEEL:
      HMGR_SetSensorLogState(HMGR_SNS_MWHL, unitNo, argbool);
      break;
#if _FC_NUM_SCAM != 0
   case CMD_LOG_CHU:
      HMGR_SetSensorLogState(HMGR_SNS_SCAM, unitNo, argbool);
      break;
#endif

#if _FC_NUM_GPS != 0
   case CMD_LOG_GPS:
      HMGR_SetSensorLogState(HMGR_SNS_GPS, unitNo, argbool);
      break;
#endif

#if _FC_NUM_FSS != 0
   case CMD_LOG_FSS:
      HMGR_SetSensorLogState(HMGR_SNS_FSS, unitNo, argbool);
      break;
#endif

#if _FC_NUM_SWHL != 0
   case CMD_LOG_SWHEEL:
      HMGR_SetSensorLogState(HMGR_SNS_SWHL, unitNo, argbool);
      break;
#endif

#if _FC_NUM_GYR != 0
   case CMD_LOG_GYR:
      HMGR_SetSensorLogState(HMGR_SNS_GYR, unitNo, argbool);
      break;
#endif
   case CMD_ACT_ENABLE_MTQ:
      AINT_SetActuatorEnableState(HMGR_ACT_MTQ, mode8, unitNo, argbool);
      break;
   case CMD_ACT_ENABLE_MWHL:
      AINT_SetActuatorEnableState(HMGR_ACT_MWHL, mode8, unitNo, argbool);
      break;
#if _FC_NUM_SWHL != 0
   case CMD_ACT_ENABLE_SWHL:
      AINT_SetActuatorEnableState(HMGR_ACT_SWHL, mode8, unitNo, argbool);
      break;
#endif
   case CMD_SET_FAILED_MWHL:
      AINT_SetFailedWheel(arg8);
      break;
   case CMD_SET_FAILED_AIM:
      AINT_SetFailedAIM(arg8);
      break;
   case CMD_SET_ALG_DELAY:
      SetAlgDelay(mode12, arg12);
      break;
   case CMD_START_CONFIG_SKED:
      ACON_StartSked(value);
      break;
   case CMD_ABORT_CONFIG_SKED:
      ACON_AbortSked();
      break;
   case CMD_AIM_FRAME_SIZE:
      HINT_UpdateAimPeriod(arg8);
      break;
   case CMD_AIM_SAMPLE_TIME:
      HINT_SetAimSampleTime(arg16);
      break;
   case CMD_SET_WAKEUP_AIM_TIME:
	  set_wakeup_AIM_time(arg16);
	  break;
   case CMD_AIM_ACTUATE_TIME:
      HINT_SetAimActuateTime(arg16);
      break;
   case CMD_REGISTER_SAFE_SKED:
	   ACON_SKEDFilenumber[MODE_SBM + ACON_SAFE_SKED_OFFSET] = arg16;
      break;
   case CMD_REGISTER_DTM_SKED:
	   ACON_SKEDFilenumber[MODE_DTM] = arg16;
      break;
   case CMD_REGISTER_YTM_SKED:
	   ACON_SKEDFilenumber[MODE_YTM] = arg16;
      break;
   case CMD_REGISTER_CPM_SKED:
	   ACON_SKEDFilenumber[MODE_CPM] = arg16;
      break;
   case SKED_COMPLETED:
	   ACON_SetSkedStatus(GDEF_FALSE);
      break;
   case TC_SHOW_COMPILE_TIME:
   	  {
   		printf("Compiled on %s @ %s\r\n", __DATE__,__TIME__);
   	  }
   	  break;
#if 0
   case CMD_REGISTER_DRIVE_FILE:
      ACON_DriveFilenumber   = arg16;
#endif
      break;
   default:
      return GDEF_FAILURE;

   }

   return GDEF_SUCCESS;
}

teGDEF_FUNC_STATUS ATTC_ProcAlgCmd(tGDEF_UINT16 cmd, tGDEF_UINT32 value)
{
#ifdef _INC_STAR_TRACKER_
   tGDEF_UINT8 valByte[4];
#endif

   switch (cmd)
   {
   case CMD_SET_ATTITUDE:
      AINT_pAttitude->initFlag = GDEF_TRUE;
      break;
   case CMD_ROLL_REF:
      AINT_pAttitude->cmd[GDEF_X_AXIS] = (tGDEF_INT16) value;
      break;
   case CMD_PITCH_REF:
      AINT_pAttitude->cmd[GDEF_Y_AXIS] = (tGDEF_INT16) value;
      break;
   case CMD_YAW_REF:
      AINT_pAttitude->cmd[GDEF_Z_AXIS] = (tGDEF_INT16) value;
      break;
#ifdef _INC_AUTO_TRANS_
   case CMD_GO_TO_CPM:
      AINT_pAlgConfig->mode.go_to_CPM = (tGDEF_UCHAR) value;
      break;
#endif

#ifdef _NOT_NEEDED_
   case CMD_YAW_STEERING:
      AINT_pAlgConfig->enable_yaw_steering = (tGDEF_UCHAR) value;
      break;
   case CMD_MAX_GUIDE_RATE:
      AINT_pAlgConfig->maxGuideRate = (tGDEF_UINT16) value;
      break;
   case CMD_YTM_RATE:
      AINT_pAlgConfig->YTM_YtumbleRate = (tGDEF_UINT16) value;
      AINT_pAlgConfig->setYTM_YtumbleRate = GDEF_TRUE;
      break;
#endif
   case CMD_Y_AXIS_BIAS:
      AINT_pAlgConfig->wheel.YbiasMomentum = (tGDEF_UINT16) value;
      break;
   case CMD_MTM_OAK_INIT:
      AINT_pAlgConfig->cpmest.Enable_MagSunInit_by_MagOnly = (tGDEF_UCHAR) value;
      break;
   case CMD_DIPOLE:
      AINT_pAlgConfig->cpmest.Enable_Dipole = (tGDEF_UINT8) value;
      break;
   case CMD_MAGSUN_YTM:
      AINT_pAlgConfig->cpmest.Enable_MagSun_WhenYTM = (tGDEF_UCHAR) value;
      break;
   case CMD_RATE_EST_INIT:
      AINT_pAlgConfig->rate.KRF_Initialise = (tGDEF_UCHAR) value;
      break;
   case CMD_MAGSUN_INIT:
      AINT_pAlgConfig->cpmest.MagSun_Initialise = (tGDEF_UCHAR) value;
      break;
   case CMD_MAGSUN_COV_ONLY_RESET:
      AINT_pAlgConfig->cpmest.MagSun_Only_Cov_Reset = (tGDEF_UCHAR) value;
      break;
   case CMD_MAGSUN_COLINEAR_CHECK:
      AINT_pAlgConfig->cpmest.Enable_MagSun_Colinear_Check = (tGDEF_UCHAR) value;
      break;

#ifdef _INC_STAR_TRACKER_
   case CMD_SCAM_BLINDING:
      valByte[0] = 0x01 & (tGDEF_UINT8) value; /* flag val */
      valByte[1] = 0xFF & (tGDEF_UINT8) (value >> 8); /* scam no */

      if (valByte[1] == 0)
         AINT_pAlgConfig->str.isBlinding[0] = (tGDEF_UCHAR) value;
      else
         AINT_pAlgConfig->str.isBlinding[1] = (tGDEF_UCHAR) value;
      break;

   case CMD_SCAM_CLOSED_LOOP:
      AINT_pAlgConfig->str.isStrClosedLoop = 0x01 & (tGDEF_UCHAR) value;
      break;
#endif

      /*
        case CMD_GPS_NAV:
           pConfig->Enable_GPS_Navigation = (tGDEF_UINT8) value;
           break;
        case CMD_COMPLEX_ORBIT_MODEL:
           pConfig->Enable_CS22 = (tGDEF_UINT8) value;
           break;
      */

   case CMD_DISABLE_MTQ:
      AINT_pAlgConfig->mag.Disable_MTR  = (tGDEF_UINT16) value;
      break;
   case CMD_RESET_ENABLE_FLAGS:
      AINT_SetEnableStateFromDatabase((tGDEF_CHAR)(0x03 & value));
      break;

#ifdef _INC_PROP_

#ifdef _DONT_USE_
   case CMD_PROP0_FIRE:
      HMGR_PropHandler(PROP_ACTION_FIRE_PROP0, value);
      break;
#endif
   case CMD_PROP_DIST_CORR:
      HMGR_PropHandler(PROP_ACTION_CORR, value);
      break;

   case CMD_FIRING_ATT:
      HMGR_PropHandler(PROP_ACTION_ATT, value);
      break;
#endif

   case CMD_100SP_WHL_SPEED:
#ifdef _INC_PD_WHL_
      AINT_pAlgConfig->wheel.pdSpeed     = (tGDEF_INT32) value;
#endif

      break;

   case CMD_ENABLE_BACKGROUND_ATT:
      if(value > 0)
      {
         AINT_pAlgConfig->mode.EnableBackgroundAttitude = GDEF_TRUE;
         /* Log command */
         EWOD_WriteMessage("Background Attitude Enabled\n");
      }
      else
      {
         AINT_pAlgConfig->mode.EnableBackgroundAttitude = GDEF_FALSE;
         EWOD_WriteMessage("Background Attitude Disabled\n");
      }
      break;

   case CMD_SET_BACKGROUND_ATT:
      AINT_pAttitude->BackgroundAttitudeYaw = (tGDEF_INT16) value;
      break;


   default:
      return GDEF_FAILURE;
   }


   return GDEF_SUCCESS;
}



tGDEF_UINT32 * GetShellTlm(tGDEF_UINT16 chan)
{
   tGDEF_UINT32 callVal = 0;

   /* update shell tlm */
   switch (chan)
   {
   case TLM_DRIVE_FNUM:
      callVal = AFIH_driveFileNum;
      break;
   case TLM_EWOD_MASK:
      callVal = EWOD_GetModeMask();
      break;
   case TLM_ARO:
      callVal = AROH_GetStatus();
      break;
   case TLM_SHELL_STATUS:
      callVal = ACON_GetShellStatus();
      break;
   case TLM_HW_STATUS:
      callVal = HMGR_GetStatus(HMGR_HERITAGE_STATUS);
      break;
   case TLM_HW_STATUS_EXT:
      callVal = HMGR_GetStatus(HMGR_ADD_UNIT_STATUS);
      break;
   case TLM_TIMERS_A:
      callVal =  ACON_TimerTlm[TIMERS_A];
      break;
   case TLM_TIMERS_B:
      callVal = ACON_TimerTlm[TIMERS_B];
      break;
#ifdef _NOT_NEEDED
   case TLM_HW_STATUS:
      callVal = HMGR_GetStatus(HMGR_ADD_UNIT_STATUS);
      dataId = 3;
      break;
#endif
      /* config sked fnums */
   case TLM_DTM_CONFIG:
      if(AFIH_GetFnum(MODE_DTM,&callVal) != GDEF_SUCCESS)
      {
         callVal = AINT_INVALID_32BIT_TLM;
      }
      break;

   case TLM_YTM_CONFIG:
      if(AFIH_GetFnum(MODE_YTM,&callVal) != GDEF_SUCCESS)
      {
         callVal = AINT_INVALID_32BIT_TLM;
      }
      break;
   case TLM_CPM_CONFIG:
      if(AFIH_GetFnum(MODE_CPM,&callVal) != GDEF_SUCCESS)
      {
         callVal = AINT_INVALID_32BIT_TLM;
      }
      break;
   case TLM_SAFE_CONFIG:
      if(AFIH_GetRev(MODE_SAFE,&callVal) != GDEF_SUCCESS)
      {
         callVal = AINT_INVALID_32BIT_TLM;
      }
      break;
      /* config sked fnums */
   case TLM_DTM_CONFIG_REV:
      if(AFIH_GetRev(MODE_DTM,&callVal) != GDEF_SUCCESS)
      {
         callVal = AINT_INVALID_32BIT_TLM;
      }
      break;
   case TLM_YTM_CONFIG_REV:
      if(AFIH_GetRev(MODE_YTM,&callVal) != GDEF_SUCCESS)
      {
         callVal = AINT_INVALID_32BIT_TLM;
      }
      break;
   case TLM_CPM_CONFIG_REV:
      if(AFIH_GetRev(MODE_CPM,&callVal) != GDEF_SUCCESS)
      {
         callVal = AINT_INVALID_32BIT_TLM;
      }
      break;
   case TLM_SAFE_CONFIG_REV:
      if(AFIH_GetRev(MODE_SAFE,&callVal) != GDEF_SUCCESS)
      {
         callVal = AINT_INVALID_32BIT_TLM;
      }
      break;
   case TLM_UNSOL_MODE_ARRAY:
      callVal = AMOH_GetUnsolModeArray();
      break;
   case TLM_ENABLE_SNS_FLAG:
      callVal = AINT_GetPackedSensorEnableFlagsArray();
      break;
   case TLM_ENABLE_ACT_FLAG:
      callVal = AINT_GetPackedActuatorEnableFlagsArray();
      break;
   default:
      return NULL;
   }

   /* store data */
   ATTC_shellTlm[chan] = callVal;

   /* return address of data */
   return &ATTC_shellTlm[chan];
}



