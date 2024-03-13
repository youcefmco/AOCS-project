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
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/source/HMGR_Cmd.c,v $
 * Revision    : $Revision: 1.10 $
 *
 * History:
 *
 * $Log: HMGR_Cmd.c,v $
 * Revision 1.10  2016/03/14 10:47:46  ytrichakis
 * Register with the SKED library
 *
 * Revision 1.7  2013/09/06 13:40:22  ytrichakis
 * removed unguarded printfs
 *
 * Revision 1.6  2013/09/03 13:35:21  ytrichakis
 * Removed calls to logging library as now all logs are done in the internel ADYYMMDD file
 *
 * Revision 1.5  2013/07/18 15:56:53  ytrichakis
 * fixed wheel bug(DR#14548) and applied the same solution in case of failed AIM
 *
 * Revision 1.4  2013/05/24 13:03:27  ytrichakis
 * Fixed AIM0 problem and added TC return handling from SKED RTP
 *
 * Revision 1.3  2013/04/23 15:49:22  ytrichakis
 * Added block wait till SKED msgQ is ready to open
 *
 * Revision 1.2  2013/04/17 14:17:41  ytrichakis
 * Check in with fixing the identation only
 *
 * Revision 1.1  2013/04/11 13:42:21  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/


#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include <time.h>
#include <limits.h>
#include <String.h>
#include <math.h>
#include <mqueue.h>
#include <semaphore.h>

#include "GDEF_GlobDefs.h"
#include "Adcs_mission.h"
#include "mission.h"

#include "AINT_AdcsInterface.h"
//#include "EWOD_EwodHandler_OBC750.h"
#include "EWOD_EwodHandler.h"
#include "ACON_AocsConfig.h"
#include "AMOH_AocsModeHandler.h"
#include "AROH_AroHandler.h"
#include "ATTC_AocsTTC.h"
#include "AOCS_AocsShell.h"
#include "HINT_AIM.h"
#include "HINT_MWheel.h"
#include "HMGR_Globals.h"

#include "CANS_Interface.h"
#include "CANS_API.h"
#include "CANA_ServerApiLib.h"

#if _FC_NUM_SWHL != 0
#include "HINT_SWheel.h"
#endif

/*-----------------------------------------------------------------------
 * External Function Prototypes
 */
extern tsCANS_RegRtn *getCANReg(void);

/*---------------------------------------------------------------------------
 * Local Data (declared as static; used only within this module)
 */
#ifdef _INC_PROP_

//fire command for prop controller
#define PROPCMD_FIRE (6)

#define NUM_PROP_FIRING_ATTITUDES  (4)
const tGDEF_CHAR HMGR_sPropFiringAtt[NUM_PROP_FIRING_ATTITUDES+1][10] = {"NONE", "INC_SMA", "DEC_SMA", "INC_I", "DEC_I"};

teGDEF_FUNC_STATUS HMGR_PropHandler(tGDEF_UINT8 action, tGDEF_UINT32 value)
{
   teGDEF_FUNC_STATUS retVal = GDEF_FAILURE;

   tsALG_PROP_CONFIG *pProp = &(AINT_GetAlgConfig()->prop);
   tGDEF_UINT8 propNo;


   switch(action)
   {
   case PROP_ACTION_DISABLE:
      pProp->usePropCorr = GDEF_OFF;
      break;


#ifdef _DONT_USE_
//fired via the saferty task now
   case PROP_ACTION_FIRE_PROP0:
      if (AMOH_GetMode() == MODE_CPM)
      {
         HMGR_ConfigCanCmd(CANADDR_PROPULSION_CONTROLLER0, PROPCMD_FIRE, value);
         EWOD_WriteMessage("HMGR_PropHandler:Fired Prop 0  value [%d]\n", value);
         pProp->usePropCorr = GDEF_ON;
         retVal = GDEF_SUCCESS;
      }
      else
      {
         /* log error */
         EWOD_WriteMessage("HMGR_PropHandler ERROR: Unable to fire Prop 0: Incorrect mode");
         pProp->usePropCorr = GDEF_OFF;

      }
      break;

   case PROP_ACTION_FIRE_PROP1:
      if (AMOH_GetMode() == MODE_CPM)
      {
         HMGR_ConfigCanCmd(CANADDR_PROPULSION_CONTROLLER1, PROPCMD_FIRE, value);
         EWOD_WriteMessage("HMGR_PropHandler:Fired Prop 1  value [%d]\n", value);
         pProp->usePropCorr = GDEF_ON;

         retVal = GDEF_SUCCESS;
      }
      else
      {
         /* log error */
         EWOD_WriteMessage("HMGR_PropHandler ERROR: Unable to fire Prop 1: Incorrect mode");
         pProp->usePropCorr = GDEF_OFF;
      }
      break;
#endif

   case PROP_ACTION_ATT:
   {
      //check value is allowed
      if(value <= NUM_PROP_FIRING_ATTITUDES)
      {
         pProp->firingAttitude = (tGDEF_UCHAR) value;
         EWOD_WriteMessage("HMGR_PropHandler Firing attitude set to %s [%d]\n", HMGR_sPropFiringAtt[value], value);


         retVal = GDEF_SUCCESS;
      }
      else
      {
         EWOD_WriteMessage("HMGR_PropHandler ERROR: Invalid attitude setting [%d]\n", value);
      }
   }
   break;

   case PROP_ACTION_CORR:
   {
      pProp->usePropCorr = (value > 0)  ? GDEF_ON : GDEF_OFF;
      retVal = GDEF_SUCCESS;
   }
   break;


   }
   return retVal;
}


#endif

/*!
 * \brief      Actuate
 *
 *             Send out the actuator commands.
 *             This function converts the data provided by the AOCS algorithms
 *             into the commands required by the AOCS hardware.
 *
 * \param  pState  Pointer to the state struct
 *
 ******************************************************************************/
void HMGR_Actuate(void)
{
   tsALG_ENABLE_ACTUATOR *pEnableFlag = AINT_GetActuatorEnableFlags();

   /* no action in standby mode */
   if(MODE_SBM == HMGR_mode)
   {
      return;
   }

   /* actuate mtq */
   HINT_ActuateMtq(AINT_pAlgCmd->torquer, AINT_pAlgCmd->torquerValid, pEnableFlag->mtq);

   /* actuate microsat wheels */
   HINT_ActuateMWheels(AINT_pAlgCmd->wheel, AINT_pAlgCmd->wheelValid, pEnableFlag->whl);

#if _FC_NUM_SWHL != 0
#ifdef _SWHL_100SP_
   /* actuate microsat wheels */
   HINT_ActuateSWheels(AINT_pAlgCmd->swheel, AINT_pAlgCmd->swheelValid, pEnableFlag->swhl);
#else
   HINT_ActuateSWheels(AINT_pAlgCmd->swheel, AINT_pAlgCmd->swheelValid, pEnableFlag->swhl,
	   pSEnableFlag->aimb, AMOH_GetUnsolModeFlag());
#endif
#endif
}

/*!
 * \brief      HMGR_Cmd_Update
 *
 *             Send out any regular TC to units (i.e. sending PVT to star tracker).
 *
 * \param  pState  Pointer to the state struct
 *
 ******************************************************************************/
#if _FC_NUM_DPU != 0
void HMGR_Cmd_Update()
{

   tsALG_ENABLE_SENSOR *pEnableFlag = AINT_GetSensorEnableFlags();

   // velocity information for the DPU
   HINT_DPUSendVelocityInfo(AINT_pAlgTlm->velocity, &AINT_pAlgTlm->velocity_valid, pEnableFlag->dpu);

}
#endif

/*!
 * \brief      HMGR_AreAcksPending
 *
 *             determines if there are still acks pending from actuate cmds
 *
 * \return Boolean value
 *
 ******************************************************************************/
teGDEF_BOOLEAN HMGR_CheckNoAckErrors(void)
{
   teGDEF_BOOLEAN retVal = GDEF_TRUE;

   /* check if have naks or timeouts */
   if (HMGR_numNaks != 0)
   {
      /* log error */
      strcpy(EWOD_sLogMsg, "Num NAKS found ");
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", HMGR_numNaks);
      strcat(EWOD_sLogMsg, "\n");
      EWOD_WriteMessage(EWOD_sLogMsg);
#ifdef REALTIME_DEBUG
      (void)printf("NACKS in check: %d\n",HMGR_numNaks);
#endif /* REALTIME_DEBUG */
      /* reset NAK counter */
      HMGR_numNaks = 0;

      /* set return vlue to false */
      retVal = GDEF_FALSE;
   }

   /* check if messages still pending */
   if (HMGR_numAcksPending != 0)
   {
#ifdef REALTIME_DEBUG
      (void)printf("cmd not out %d\n",HMGR_numAcksPending);
#endif /* REALTIME_DEBUG */
      /* log error */
      strcpy(EWOD_sLogMsg, "Acks pending: ");
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", HMGR_numAcksPending);
      strcat(EWOD_sLogMsg, "\n");
      EWOD_WriteMessage(EWOD_sLogMsg);

      /* set return vlue to false */
      retVal = GDEF_FALSE;
   }

   return retVal;
}

/*!
 * \brief       HMGR_ActuateCanCmd
 *
 *             Sends Actuation CAN command with given HINT default params
 *
 *
 ******************************************************************************/
void HMGR_ActuateCanCmd(tGDEF_UINT8 node, tGDEF_UINT16 cmdNo, tGDEF_UINT32 val)
{
   teCANA_APIRes             eRes          = 0;
   tsTcTlmNodeValue          stTc          = {0};
   tsCANA_TcTlmRequestParams stTcReqParams = {0};
   /*
    * Build and send the Telecommand to the actuator
    */
   stTc.u1Dest  = node;
   stTc.u2Chan  = cmdNo;
   stTc.u4Value = val;

   stTcReqParams.psRegInfo      = (tsCANS_RegRtn*)getCANReg();
   stTcReqParams.eSendQPriority = eCANS_LO_PRI_Q;
   stTcReqParams.eWait          = eCANS_NO_WAIT;
   stTcReqParams.u1Src          = CANADDR_ADCS_PROCESS;
   stTcReqParams.u4RetryCount   = DEFAULT_AOCS_TC_RETRY_COUNT;
   stTcReqParams.u4RtnQIdx      = DEFAULT_DISPATCHER_Q;
   stTcReqParams.u4Timeout      = DEFAULT_TELECOMMAND_REQUEST_TIMEOUT;

   eRes = CANA_send_tc(&stTcReqParams, &stTc);
   HMGR_numAcksPending++; /* increment the total number of cmds send */

   if (eCANA_OK != eRes)
   {
      strcpy(EWOD_sLogMsg, "HMGR_ActuateCanCmd: error in CANA_send_tc " );
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", eRes);
      strcat(EWOD_sLogMsg, "\n" );
      EWOD_WriteMessage(EWOD_sLogMsg);

#ifdef REALTIME_DEBUG
      (void)printf("HMGR_ActuateCanCmd: error in CANA_send_tc %d",eRes);
#endif /* REALTIME_DEBUG */
   }

}

/*!
 * \brief     HMGR_ConfigCanCmd
 *
 *             Sends Configuration CAN command with given HINT default params
 *
 *
 ******************************************************************************/
void HMGR_ConfigCanCmd(tGDEF_UINT8 node, tGDEF_UINT16 cmdNo, tGDEF_UINT32 val)
{
   teCANA_APIRes             eRes          = 0;
   tsTcTlmNodeValue          stTc          = {0};
   tsCANA_TcTlmRequestParams stTcReqParams = {0};

   /*
    * Build and send the Telecommand to the actuator
    */
   stTc.u1Dest  = node;
   stTc.u2Chan  = cmdNo;
   stTc.u4Value = val;

   stTcReqParams.psRegInfo      = (tsCANS_RegRtn*)getCANReg();
   stTcReqParams.eSendQPriority = eCANS_LO_PRI_Q;
   stTcReqParams.eWait          = eCANS_NO_WAIT;
   stTcReqParams.u1Src 			  = CANADDR_ADCS_PROCESS;
   stTcReqParams.u4RetryCount	  = DEFAULT_AOCS_TC_RETRY_COUNT;
   stTcReqParams.u4RtnQIdx 	  = DEFAULT_DISPATCHER_Q;
   stTcReqParams.u4Timeout 	  = DEFAULT_TELECOMMAND_REQUEST_TIMEOUT;

   eRes = CANA_send_tc(&stTcReqParams, &stTc);
   HMGR_numAcksPending++; /* increment the total number of cmds send */

   if (eCANA_OK != eRes)
   {
      strcpy(EWOD_sLogMsg, "HMGR_ConfigCanCmd: error in CANA_send_tc " );
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", eRes);
      strcat(EWOD_sLogMsg, "\n" );
#ifdef REALTIME_DEBUG
      (void)printf("HMGR_ConfigCanCmd: error in CANA_send_tc %d", eRes);
#endif /* REALTIME_DEBUG */
   }

}

