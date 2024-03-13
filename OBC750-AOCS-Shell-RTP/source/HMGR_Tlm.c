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
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/source/HMGR_Tlm.c,v $
 * Revision    : $Revision: 1.6 $
 *
 * History:
 *
 * $Log: HMGR_Tlm.c,v $
 * Revision 1.6  2016/03/14 10:47:46  ytrichakis
 * Register with the SKED library
 *
 * Revision 1.3  2013/07/18 15:56:53  ytrichakis
 * fixed wheel bug(DR#14548) and applied the same solution in case of failed AIM
 *
 * Revision 1.2  2013/04/17 14:17:41  ytrichakis
 * Check in with fixing the identation only
 *
 * Revision 1.1  2013/04/11 13:42:22  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <limits.h>

#include "GDEF_GlobDefs.h"
#include "Adcs_mission.h"
#include "mission.h"

#include "ACON_AocsConfig.h"
#include "AMOH_AocsModeHandler.h"
#include "AINT_AdcsInterface.h"
//#include "EWOD_EwodHandler_OBC750.h"
#include "EWOD_EwodHandler.h"
#include "HINT_AIM.h"
#include "HINT_MWheel.h"

#if _FC_NUM_SWHL != 0
#include "HINT/HINT_SWheel.h"
#endif

#if _FC_NUM_SCAM != 0
#include "HINT_Dpu.h"
#endif

#if _FC_NUM_GPS != 0
#include "HINT_Gps.h"
#endif

#if _FC_NUM_FSS != 0
#include "HINT/HINT_AIM_DB.h"
#endif

#if _FC_NUM_GYR != 0
#ifdef _INC_ALTAIR_APS_EXP_
#include "HINT/HINT_ALTAIR_APS_EXP.h"
#endif
#endif

#include "HMGR_Globals.h"

/*-----------------------------------------------------------------------
 * Defines and Macros
 */
#define MAX_TLM_PACKET_SIZE (50)
/*** UTLM definitions ***/
#define NUM_UTLM_TYPES      (7)
#define UTLM_WHEEL_OFFSET   (3)
#define HMGR_TLM_MASK_OFF   (0x00)

/*---------------------------------------------------------------------------
 * Typedefs
 */
#ifdef _SOLICITED_
typedef struct
{
   tGDEF_UINT8     node;
   tGDEF_UINT8     unitNo;
   tpfUTLM_HANDLER func;
} tsHMGR_UTLM_LOOKUP;
#endif

/*---------------------------------------------------------------------------
 * Private Function Prototypes (declared as static; used only within this module)
 */
static void HMGR_SetTlmMask(tGDEF_UCHAR * pMask, tGDEF_UCHAR isEnabled[], teGDEF_BOOLEAN isLogged[], tGDEF_UINT8 numUnits);

/*---------------------------------------------------------------------------
 * Local Data (declared as static; used only within this module)
 */
static tsCANS_TcTlmSingle HMGR_tlmPacket[MAX_TLM_PACKET_SIZE] = {{0}};
static tGDEF_UINT16       HMGR_numReqTlmChans                 = 0;
static tsHMGR_CAN_TLM     HMGR_tlm[NUM_HW_SENSOR_TYPES]       = {{0}};

#ifdef _SOLICITED_
static tGDEF_UINT16              HMGR_numTlmChans                     = 0;
static tGDEF_UINT16              HMGR_tlmIndex                        = 0;
static tsHMGR_CAN_TLM            HMGR_recvTlm                         = {{0}};
static tsCANS_TcTlmSingle        HMGR_recvTlmBuf[MAX_TLM_PACKET_SIZE] = {0};
static tGDEF_UINT32              UTLM_recvFrame[NUM_UTLM_TYPES]       = {0};
static const tsHMGR_UTLM_LOOKUP  HMGR_procUtlmLookup[NUM_UTLM_NODES]  = PROC_UTLM_LOOKUP_TABLE;

static teGDEF_BOOLEAN HMGR_InitSampleFlag = GDEF_FALSE;
#endif

static tGDEF_UCHAR HMGR_mtmTlmMask  = HMGR_TLM_MASK_OFF;
static tGDEF_UCHAR HMGR_sasTlmMask  = HMGR_TLM_MASK_OFF;
static tGDEF_UCHAR HMGR_mwhlTlmMask = HMGR_TLM_MASK_OFF;

#if _FC_NUM_SCAM != 0
static tGDEF_UCHAR HMGR_scamTlmMask  = HMGR_TLM_MASK_OFF;
#endif
#if _FC_NUM_FSS != 0
static tGDEF_UCHAR HMGR_fssTlmMask  = HMGR_TLM_MASK_OFF;
#endif

#if _FC_NUM_SWHL != 0
static tGDEF_UCHAR HMGR_swhlTlmMask  = HMGR_TLM_MASK_OFF;
#endif

#if _FC_NUM_GYR != 0
static tGDEF_UCHAR HMGR_gyrTlmMask  = HMGR_TLM_MASK_OFF;
#endif

/*---------------------------------------------------------------------------
 * Global Data
 */

static void HMGR_SetTlmMask(tGDEF_UCHAR * pMask, tGDEF_UCHAR isEnabled[], teGDEF_BOOLEAN isLogged[], tGDEF_UINT8 numUnits)
{
   tGDEF_UINT8 unitNo = 0;

   /* initialise the mask to zero */
   *pMask = 0x00;

   for(unitNo=0; unitNo < numUnits; unitNo++)
   {

      /* Check if mtm is enabled or logging */
      if(isEnabled[unitNo] || isLogged[unitNo])
      {
         /* set bit in mask for mtm no */
         *pMask |= (0x01 << unitNo);
      }
   }
}

#ifdef _SOLICITED_
void HMGR_ReqInitSample(void)
{
   HMGR_InitSampleFlag = GDEF_TRUE;
}
#endif


void HMGR_InitSample(void)
{
   tsALG_ENABLE_SENSOR *pSnsEnableFlag = AINT_GetSensorEnableFlags();
   tGDEF_UINT8 i = 0;

   /* initialise tlm variables */
   HMGR_numReqTlmChans              = 0;
   HMGR_tlm[HMGR_SNS_SAS].numChans  = 0;
   HMGR_tlm[HMGR_SNS_MTM].numChans  = 0;
   HMGR_tlm[HMGR_SNS_MWHL].numChans = 0;

#if _FC_NUM_GYR != 0
   HMGR_tlm[HMGR_SNS_GYR].numChans = 0;
#endif

#if _FC_NUM_SCAM != 0
   HMGR_tlm[HMGR_SNS_SCAM].numChans = 0;
#endif

#if _FC_NUM_GPS != 0
   HMGR_tlm[HMGR_SNS_GPS].numChans = 0;
#endif

#if _FC_NUM_FSS != 0
   HMGR_tlm[HMGR_SNS_FSS].numChans = 0;
#endif


   /* Check if not in unsolicted mode */
   if ( GDEF_FALSE == AMOH_isUnsolMode)
   {
      /*** sample magnetometers ***/
      //set tlm mask
      HMGR_SetTlmMask(&HMGR_mtmTlmMask, pSnsEnableFlag->mtm, HMGR_logMtmFlag, _FC_NUM_MTM);

      //build up tlm packet
      HMGR_tlm[HMGR_SNS_MTM].index    =  HMGR_numReqTlmChans;
      HMGR_tlm[HMGR_SNS_MTM].numChans = HINT_InitSampleMtm(&HMGR_tlmPacket[HMGR_numReqTlmChans], HMGR_mtmTlmMask);
      HMGR_numReqTlmChans += HMGR_tlm[HMGR_SNS_MTM].numChans;

      /*** sample sunsensors ***/
      //set tlm mask
      HMGR_SetTlmMask(&HMGR_sasTlmMask, pSnsEnableFlag->sas,HMGR_logSasFlag, _FC_NUM_SAS);

      //build up tlm packet
      HMGR_tlm[HMGR_SNS_SAS].index    =  HMGR_numReqTlmChans;
      HMGR_tlm[HMGR_SNS_SAS].numChans = HINT_InitSampleSas(&HMGR_tlmPacket[HMGR_numReqTlmChans], HMGR_sasTlmMask);
      HMGR_numReqTlmChans += HMGR_tlm[HMGR_SNS_SAS].numChans;


      /*** sample microsat wheels ***/
      //set mwheel mask
      HMGR_SetTlmMask(&HMGR_mwhlTlmMask, pSnsEnableFlag->whl, HMGR_logMwhlFlag, _FC_NUM_MWHL);

      //build up tlm packet
      HMGR_tlm[HMGR_SNS_MWHL].index =  HMGR_numReqTlmChans;
      HMGR_tlm[HMGR_SNS_MWHL].numChans =  HINT_InitSampleMWhl(&HMGR_tlmPacket[HMGR_numReqTlmChans], HMGR_mwhlTlmMask);
      HMGR_numReqTlmChans += HMGR_tlm[HMGR_SNS_MWHL].numChans;


#if _FC_NUM_SWHL != 0

      /*** sample small sat wheels ***/
      //set swheel mask
      HMGR_SetTlmMask(&HMGR_swhlTlmMask, pSnsEnableFlag->swhl, HMGR_logSwhlFlag, _FC_NUM_SWHL);

      //build up tlm packet
      HMGR_tlm[HMGR_SNS_SWHL].index =  HMGR_numReqTlmChans;
      HMGR_tlm[HMGR_SNS_SWHL].numChans =  HINT_InitSampleSWhl(&HMGR_tlmPacket[HMGR_numReqTlmChans], HMGR_swhlTlmMask);
      HMGR_numReqTlmChans += HMGR_tlm[HMGR_SNS_SWHL].numChans;
#endif


#if _FC_NUM_FSS != 0
      /*** sample fine sunsensors ***/
      //set tlm mask
      HMGR_SetTlmMask(&HMGR_fssTlmMask, pSnsEnableFlag->fss,HMGR_logFssFlag, _FC_NUM_FSS);

      //build up tlm packet
      HMGR_tlm[HMGR_SNS_FSS].index =  HMGR_numReqTlmChans;
      HMGR_tlm[HMGR_SNS_FSS].numChans = HINT_InitSampleFSS(&HMGR_tlmPacket[HMGR_numReqTlmChans], HMGR_fssTlmMask);
      HMGR_numReqTlmChans += HMGR_tlm[HMGR_SNS_FSS].numChans;
#endif


#if _FC_NUM_GYR != 0
      /*** sample gyros ***/
      //set tlm mask
      HMGR_SetTlmMask(&HMGR_gyrTlmMask, pSnsEnableFlag->gyr, HMGR_logGyrFlag, _FC_NUM_GYR);

      //build up tlm packet
      HMGR_tlm[HMGR_SNS_GYR].index =  HMGR_numReqTlmChans;
      HMGR_tlm[HMGR_SNS_GYR].numChans = HINT_InitSampleGyro(&HMGR_tlmPacket[HMGR_numReqTlmChans], HMGR_gyrTlmMask);
      HMGR_numReqTlmChans += HMGR_tlm[HMGR_SNS_GYR].numChans;
#endif

   }
   else
   {
      /*in unsolicited mode request dummy data for units that would otherwise switch can-bus i.e. STR*/
#if _FC_NUM_DPU != 0
      /*** sample DPU ***/
      //set tlm mask
      HMGR_SetTlmMask(&HMGR_scamTlmMask, pSnsEnableFlag->dpu, HMGR_logScamFlag, _FC_NUM_DPU);

      //build up tlm packet
      HMGR_tlm[HMGR_SNS_SCAM].index =  HMGR_numReqTlmChans;
      HMGR_tlm[HMGR_SNS_SCAM].numChans = HINT_InitSampleDpu(&HMGR_tlmPacket[HMGR_numReqTlmChans], HMGR_scamTlmMask);
      HMGR_numReqTlmChans += HMGR_tlm[HMGR_SNS_SCAM].numChans;
#endif
   }

   /* log requested telemetry AJ - just make debug */
   for (i = 0; i < HMGR_numReqTlmChans; i++)
   {
      strcpy(EWOD_sLogMsg, "HMGR_Init_Sample: HMGR_tlmPacket[" );
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d",i);
      strcat(EWOD_sLogMsg, "].node=" );
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d",HMGR_tlmPacket[i].stTcTlm.u1Dest);
      strcat(EWOD_sLogMsg, "\n" );
      EWOD_WriteMessage(EWOD_sLogMsg);

      strcpy(EWOD_sLogMsg, "HMGR_Init_Sample: HMGR_tlmPacket[" );
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", i);
      strcat(EWOD_sLogMsg, "].channel=" );
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", HMGR_tlmPacket[i].stTcTlm.u1Dest);
      strcat(EWOD_sLogMsg, "\n" );
      EWOD_WriteMessage(EWOD_sLogMsg);
   }

}



/*!
 * \brief      Sample
 *
 *             Set up sampling telemetry
 *
 * \param  pState  Pointer to the state struct
 *
 ******************************************************************************/
#ifdef _SOLICITED_
teGDEF_FUNC_STATUS HMGR_Sample(void)
{

   teGDEF_FUNC_STATUS retVal = GDEF_FAILURE;
   tGDEF_INT16 err;


   /* check if need to initialise the sampling */
   if(HMGR_InitSampleFlag == GDEF_TRUE)
   {
      HMGR_InitSample();
      HMGR_InitSampleFlag = GDEF_FALSE;
   }

   if(HMGR_numTlmChans != 0)
   {
      strcpy(EWOD_sLogMsg, "ERROR: HMGR_Sample - num chans " );
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", HMGR_numTlmChans);
      strcat(EWOD_sLogMsg, "\n" );
      EWOD_WriteMessage(EWOD_sLogMsg);
   }

   /* reset num of channels */
   HMGR_numTlmChans = 0;


   //watchdog requests
   //Implement a kick if required

   /*** NOT IN Unsol mode so continue ***/
   /*if in unsolicited mode then data will be set to invalid by UTLM_InitDataGathering just after call to algs*/
   if (GDEF_FALSE == AMOH_isUnsolMode)
   {
      /* set tlm data for algorithms invalid */
      AINT_SetDataInvalid();
   }

   if(HMGR_numReqTlmChans > 0)
   {
      /* send tlm requests AJ - check timeout */
      err = can_tlm_frame(HMGR_numReqTlmChans, HMGR_tlmPacket, HMGR_TLM_TIMEOUT, NOWAIT);

      // detect error in can_tlm_frame
      if(err >=0)
      {
         HMGR_tlmIndex = (tGDEF_UINT16) err;
         retVal = GDEF_SUCCESS;
      }
      else
      {
         strcpy(EWOD_sLogMsg, "HMGR_Sample: error in can_tlm_frame " );
         sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", HMGR_numReqTlmChans);
         strcat(EWOD_sLogMsg, "\n" );
         EWOD_WriteMessage(EWOD_sLogMsg);

         //error sending tlm requests
         HMGR_tlmIndex = 0;
         return GDEF_FAILURE;
      }
   }

#ifdef REALTIME_DEBUG
   /* Log requests */
   strcpy(EWOD_sLogMsg, "Tlm reqs = " );
   sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)],"%d", HMGR_numReqTlmChans );
   strcat(EWOD_sLogMsg, "\n" );
//  printk(EWOD_sLogMsg);
#endif


   // set the number of channels for which a response is expected
   HMGR_numTlmChans = HMGR_numReqTlmChans;

   return retVal;

   return 0;
}
#endif


/*!
 * \brief     HMGR_PrepareData
 *
 *              Prepares data for algorithms
 *
 *\return function status
 ******************************************************************************/
teGDEF_FUNC_STATUS HMGR_PrepareData(void)
{
   tsALG_ENABLE_SENSOR *pSnsEnableFlag = AINT_GetSensorEnableFlags();
   teGDEF_FUNC_STATUS retVal = GDEF_SUCCESS;

   /* store mtm data */
   if(UTLM_SetMagData(&(AINT_pSensorTlm->mag),pSnsEnableFlag->mtm) != GDEF_SUCCESS)
      retVal = GDEF_FAILURE;

   /* set the sunsensor data  */
   if(UTLM_SetSasData(&(AINT_pSensorTlm->sas), pSnsEnableFlag->sas) != GDEF_SUCCESS)
      retVal = GDEF_FAILURE;

   /* update the microsat wheel data */
   if(UTLM_SetWheelData(&(AINT_pSensorTlm->wheel),pSnsEnableFlag->whl) != GDEF_SUCCESS)
      retVal = GDEF_FAILURE;

#if _FC_NUM_SWHL != 0
   /* update the microsat wheel data */
   if(UTLM_SetSWheelData(AINT_pSensorTlm->swheel) != GDEF_SUCCESS)
      retVal = GDEF_FAILURE;

#endif

#if _FC_NUM_SCAM != 0
   if(UTLM_SetStarCamData(&(AINT_pSensorTlm->starcam),pSnsEnableFlag->scam) != GDEF_SUCCESS)
      retVal = GDEF_FAILURE;
#endif



#if _FC_NUM_GPS != 0
   /* set available GPS data */
   UTLM_SetGpsData(&(AINT_pSensorTlm->gps),pSnsEnableFlag->gps);
#endif

   return retVal;
}


#ifdef _SOLICITED_
void HMGR_CheckSample(void)
{

   tGDEF_INT16 retVal;
   tGDEF_UINT8 numRecv = 0;

   tsALG_SENSOR_TLM *pSnsTlm = AINT_GetSensorTlm();

   if(HMGR_numTlmChans != 0)
   {
      //check tlm requests
      retVal = can_tlm_check((tGDEF_INT16*)(&HMGR_recvTlm.index), HMGR_recvTlmBuf);

      //did any tlm get returned
      if(retVal == 0)
         return;

      //check if the correct  number of channels was returned
      if((HMGR_recvTlm.index == HMGR_tlmIndex) ) // this is the packet that was requested
      {
         HMGR_recvTlm.numChans = retVal;

         //check the number of channels agrees
         if(HMGR_recvTlm.numChans != HMGR_numTlmChans)
         {
            strcpy(EWOD_sLogMsg, "Error on num chans in can_tlm_check " );
            sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", HMGR_recvTlm.numChans);
            strcat(EWOD_sLogMsg, "\n" );
            EWOD_WriteMessage(EWOD_sLogMsg);

            HMGR_numTlmChans = 0;
            return;
         }

         /***Process the telemetry ***/

         //check if any mtm tlm is pending
         if(HMGR_tlm[HMGR_SNS_MTM].numChans > 0)
         {
            if(HINT_SetMtmSampleTlm(&HMGR_recvTlmBuf[HMGR_tlm[HMGR_SNS_MTM].index], HMGR_mtmTlmMask, &(pSnsTlm->mag)) == GDEF_SUCCESS)
            {
               /* reduce the number of tlm chans pending */
               HMGR_numTlmChans -=  HMGR_tlm[HMGR_SNS_MTM].numChans;
            }
         }

         //check if any wheels tlm is pending
         if (HMGR_tlm[HMGR_SNS_MWHL].numChans > 0)
         {
            if(HINT_SetMWhlSampleTlm(&HMGR_recvTlmBuf[HMGR_tlm[HMGR_SNS_MWHL].index], HMGR_mwhlTlmMask, &(pSnsTlm->wheel)) == GDEF_SUCCESS)
            {
               /* reduce the number of tlm chans pending */
               HMGR_numTlmChans -=  HMGR_tlm[HMGR_SNS_MWHL].numChans;
            }
         }

         //check if any sun sensor tlm is pending
         if(HMGR_tlm[HMGR_SNS_SAS].numChans > 0)
         {
            if(HINT_SetSasSampleTlm(&HMGR_recvTlmBuf[HMGR_tlm[HMGR_SNS_SAS].index], HMGR_sasTlmMask, &(pSnsTlm->sas)) == GDEF_SUCCESS)
            {
               /* reduce the number of tlm chans pending */
               HMGR_numTlmChans -=  HMGR_tlm[HMGR_SNS_SAS].numChans;
            }

         }

#if  _FC_NUM_SWHL != 0
         //check if any wheels tlm is pending
         if (HMGR_tlm[HMGR_SNS_SWHL].numChans > 0)
         {
            if(HINT_SetSWhlSampleTlm(&HMGR_recvTlmBuf[HMGR_tlm[HMGR_SNS_SWHL].index], HMGR_mwhlTlmMask, pSnsTlm->swheel) == GDEF_SUCCESS)
            {
               /* reduce the number of tlm chans pending */
               HMGR_numTlmChans -=  HMGR_tlm[HMGR_SNS_SWHL].numChans;
            }
         }
#endif

#if _FC_NUM_SCAM != 0
         // check if any starcam solicited telemetry is pending
         if (HMGR_tlm[HMGR_SNS_SCAM].numChans >0)
         {
            if(HINT_SetDpuSampleTlm(&HMGR_recvTlmBuf[HMGR_tlm[HMGR_SNS_SCAM].index], HMGR_scamTlmMask, &(pSnsTlm->starcam)) == GDEF_SUCCESS)
            {
               /* reduce the number of tlm chans pending */
               HMGR_numTlmChans -=  HMGR_tlm[HMGR_SNS_SCAM].numChans;
            }
         }
#endif

#if _FC_NUM_FSS != 0
         // check if any starcam solicited telemetry is pending
         if (HMGR_tlm[HMGR_SNS_FSS].numChans >0)
         {
            if(HINT_SetFssSampleTlm(&HMGR_recvTlmBuf[HMGR_tlm[HMGR_SNS_FSS].index], HMGR_fssTlmMask, pSnsTlm->fss) == GDEF_SUCCESS)
            {
               /* reduce the number of tlm chans pending */
               HMGR_numTlmChans -=  HMGR_tlm[HMGR_SNS_FSS].numChans;
            }
         }
#endif


#if _FC_NUM_GYR != 0
         // check if any starcam solicited telemetry is pending
         if (HMGR_tlm[HMGR_SNS_GYR].numChans >0)
         {
            if(HINT_SetGyroSampleTlm(&HMGR_recvTlmBuf[HMGR_tlm[HMGR_SNS_GYR].index], HMGR_gyrTlmMask, pSnsTlm->gyr) == GDEF_SUCCESS)
            {
               /* reduce the number of tlm chans pending */
               HMGR_numTlmChans -=  HMGR_tlm[HMGR_SNS_GYR].numChans;
            }
         }
#endif


      }
      else
      {
         //log error as in correct tlm index?
      }

   }

}
#endif

void HMGR_GetTlmData(tsCANS_TcTlmSingle *pTlm, tGDEF_UINT8 numBytes, tGDEF_UINT32 * pData, teGDEF_BOOLEAN * pIsValid)
{
   /* update the hint telemetry */
   if ((pTlm->stTcTlm.e1ReturnStatus == eCANS_TRAN_TIMEOUT) ||
         (pTlm->stTcTlm.e1ReturnStatus  == eCANS_TRAN_TLM_NAK))
   {
      *pIsValid = GDEF_FALSE;

      if(numBytes == 4)
         *pData = AINT_INVALID_32BIT_TLM;
      else
         *pData = AINT_INVALID_16BIT_TLM;

   }
   else
   {
      *pData = pTlm->stTcTlm.u4Value;
      *pIsValid = GDEF_TRUE;
   }
}

teGDEF_FUNC_STATUS HMGR_ProcUnsolTlm(tGDEF_UINT8 node, tGDEF_UINT16 chan, tGDEF_UINT8 TS, tGDEF_UCHAR * pData)
{
   teGDEF_FUNC_STATUS retVal = GDEF_FAILURE;
#if _FC_NUM_GPS != 0
   teGDEF_BOOLEAN isNewFrame = GDEF_FALSE;
   static tGDEF_UINT8 lastTS = 0xFF;



   if (node == CANADDR_GPS_TLM)
   {
      // Ensure that all data is from the same data set - GPS toggles TS bit
      if (TS != lastTS)
      {
         // new gps frame so reset receive mask
         isNewFrame = GDEF_TRUE;
         lastTS = TS;
      }

      // process the telemetry
      UTLM_ProcGpsTlm(chan, pData, isNewFrame);
      retVal = GDEF_SUCCESS;
   }
   else
   {
      strcpy(EWOD_sLogMsg, "HMGR_ProcUnsolTlm unknown node" );
      itoa(node, &EWOD_sLogMsg[strlen(EWOD_sLogMsg)], 10);
      strcat(EWOD_sLogMsg, "\n" );
      EWOD_WriteMessage(EWOD_sLogMsg);
   }

#endif

   return retVal;
}


teGDEF_FUNC_STATUS HMGR_SetSensorLogState(teHMGR_HW_SENSORS sensor, tGDEF_UINT8 snsNo, teGDEF_BOOLEAN state)
{
   switch(sensor)
   {
   case HMGR_SNS_MTM:
      if (snsNo >= _FC_NUM_MTM)
         return GDEF_FAILURE;
      HMGR_logMtmFlag[snsNo] = state;
      break;

   case HMGR_SNS_SAS:
      if (snsNo >= _FC_NUM_SAS)
         return GDEF_FAILURE;
      HMGR_logSasFlag[snsNo] = state;
      break;

   case HMGR_SNS_MWHL:
      if (snsNo >= _FC_NUM_MWHL)
         return GDEF_FAILURE;
      HMGR_logMwhlFlag[snsNo] = state;
      break;

#if _FC_NUM_SWHL != 0
   case HMGR_SNS_SWHL:
      if (snsNo >= _FC_NUM_SWHL)
         return GDEF_FAILURE;

      HMGR_logSwhlFlag[snsNo] = state;
      break;

#endif

#if _FC_NUM_FSS != 0

   case HMGR_SNS_FSS:
      if (snsNo >= _FC_NUM_FSS)
         return GDEF_FAILURE;

      HMGR_logFssFlag[snsNo] = state;
      break;
#endif

#if _FC_NUM_GYR != 0

   case HMGR_SNS_GYR:
      if (snsNo >= _FC_NUM_GYR)
         return GDEF_FAILURE;

      HMGR_logGyrFlag[snsNo] = state;
      break;
#endif


#if _FC_NUM_GPS != 0

   case HMGR_SNS_GPS:
      if (snsNo >= _FC_NUM_GPS)
         return GDEF_FAILURE;

      HMGR_logGpsFlag[snsNo] = state;
      break;
#endif

#if _FC_NUM_SCAM != 0

   case HMGR_SNS_SCAM:
      if (snsNo >= _FC_NUM_SCAM)
         return GDEF_FAILURE;

      HMGR_logScamFlag[snsNo] = state;
      break;
#endif
   default:
      /* invalid sensor type */
      return GDEF_FAILURE;
   }

#ifdef _SOLICITED_
   //request the sampling be initialised - so new logging can begin
   HMGR_ReqInitSample();
#endif

   return GDEF_SUCCESS;
}



/*!
 * \brief     UTLM_InitDataGathering
 *
 *              Initialises data gathering for a new AOCS cycle
 *
 *\return boolean flag
 ******************************************************************************/
void UTLM_InitDataGathering(void)
{
   /* reset data variables, received buffers and duplicate buffers */

#if _FC_NUM_AIM != 0
   UTLM_AIMInitDataGathering();
#endif

#if _FC_NUM_DPU != 0
   UTLM_ScamInitDataGathering();
#endif

#if _FC_NUM_GPS != 0
   UTLM_GpsInitDataGathering();
#endif

#if _FC_NUM_MWHL !=0
   UTLM_MWInitDataGathering();
#endif

#if _FC_NUM_SWHL != 0
   UTLM_SWInitDataGathering();
#endif

   /* set tlm data for algorithms invalid */
   AINT_SetDataInvalid();
}


/*!
 * \brief     UTLM_IsAllDataReceived
 *
 *              Checks if all expected unsolicted frames are received
 *
 *\return boolean flag
 ******************************************************************************/
teGDEF_BOOLEAN UTLM_IsAllDataReceived(void)
{
   teGDEF_BOOLEAN retVal = GDEF_FALSE;

   /*check we have all aim & wheel data */
   if((UTLM_IsAllAimDataReceived()   == GDEF_TRUE) &&
         (UTLM_IsAllMWhlDataReceived() == GDEF_TRUE)
#if _FC_NUM_SCAM != 0
         && (UTLM_IsAllScamDataReceived() == GDEF_TRUE)
#endif
     )
   {
      retVal = GDEF_TRUE;
   }

   return retVal;
}


/*!
 * \brief     UTLM_InitMode
 *
 *              initialises the module for a given mode
 *
 * \param mode mode number
 *
 ******************************************************************************/
void UTLM_InitMode(tGDEF_UINT8 mode)
{
   tsALG_ENABLE_SENSOR *pSnsEnableFlag = AINT_GetSensorEnableFlags();

   UTLM_AIMInit(); // clear the expected telemetry mask

   /* initialise expected frames for the AIM */
   //set tlm masks
   HMGR_SetTlmMask(&HMGR_mtmTlmMask, pSnsEnableFlag->mtm, HMGR_logMtmFlag, _FC_NUM_MTM);
   UTLM_MtmInit(HMGR_mtmTlmMask);

   HMGR_SetTlmMask(&HMGR_sasTlmMask, pSnsEnableFlag->sas, HMGR_logSasFlag,  _FC_NUM_SAS);
   UTLM_SasInit(HMGR_sasTlmMask);

   //set mwheel mask
   HMGR_SetTlmMask(&HMGR_mwhlTlmMask, pSnsEnableFlag->whl, HMGR_logMwhlFlag, _FC_NUM_MWHL);
   UTLM_MWhlInit(HMGR_mwhlTlmMask);

#if _FC_NUM_SCAM != 0
   //set scam mask
   HMGR_SetTlmMask(&HMGR_scamTlmMask, pSnsEnableFlag->scam, HMGR_logScamFlag, _FC_NUM_MWHL);
   UTLM_ScamInit(HMGR_scamTlmMask);
#endif


   /* ensure that data gathering is already initialised */
   UTLM_InitDataGathering();

}



