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
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/source/HINT_AIM.c,v $
 * Revision    : $Revision: 1.7 $
 *
 * History:
 *
 * $Log: HINT_AIM.c,v $
 * Revision 1.7  2016/03/14 10:47:46  ytrichakis
 * Register with the SKED library
 *
 * Revision 1.4  2013/06/04 10:09:03  ytrichakis
 * AOCS Delivery 4th June for flight rehearsal
 *
 * Revision 1.3  2013/05/24 13:03:27  ytrichakis
 * Fixed AIM0 problem and added TC return handling from SKED RTP
 *
 * Revision 1.2  2013/04/17 14:17:41  ytrichakis
 * Check in with fixing the identation only
 *
 * Revision 1.1  2013/04/11 13:42:21  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/


#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <limits.h>
#include <Math.h>
#include <mqueue.h>
#include <pthread.h>

#include "HINT_AIM.h"
#include "GDEF_GlobDefs.h"
#include "CANS_Interface.h"
#include "CANS_API.h"
#include "CANA_ServerApiLib.h"

#include "mission.h"
#include "Adcs_mission.h"

#include "ACON_AocsConfig.h"
#include "AMOH_AocsModeHandler.h"
#include "AINT_AdcsInterface.h"
#include "ATTC_AocsTTC.h"
//#include "EWOD_Ewodhandler_OBC750.h"
#include "EWOD_EwodHandler.h"
#include "HMGR_Globals.h"
#include "HMGR_HardwareManager.h"


/*-----------------------------------------------------------------------
 * Defines and Macros
 */
/*number of devices supported by the AIM*/
#define AIM_NUM_SAS (2)
#define AIM_NUM_MTM (1)
#define AIM_NUM_MTQ (3)

/* AIM unsolicted tlm groups */
#define UTLM_SAS_RAW_GROUP_ID (0)
#define UTLM_SAS_LIM_GROUP_ID (1)
#define UTLM_MTM_GROUP_ID     (2)

#define UTLM_NUM_SAS_GROUPS   (2)
#define UTLM_NUM_MTM_GROUPS   (1)

#define UTLM_MAX_SAS_GROUP_ID UTLM_SAS_LIM_GROUP_ID

/* bit mask of sequence number for each sas group */
#define UTLM_SAS_SEQNO_LOOKUP      \
   {                                \
   /*SASNO/GROUP NO:  0      1    */\
  /* SAS0 */        {0x03, 0x08},   \
  /* SAS1 */        {0x06, 0x10},   \
   }

#define UTLM_MTM_SEQNO (5)

/* Not used at the moment as already defined in  UTLM_SAS_SEQNO_LOOKUP*/
/*#define SASA_EXP_MASK 0x00000003UL
#define SASB_EXP_MASK 0x00000006UL */

/*not used at the moment but maybe in some other mission this table is useful*/
/* seq corresponding to the group */
#define AIM_GROUP_SEQNO_LOOKUP                                                  \
{                                                                           \
/*GROUP NO:             0          1           2            3           4           5            6           7               */\
/* SEQ MASK */     0x00000007, 0x00000018, 0x00000020, 0x00000000, 0x00000000, 0x00000000,  0x00000000, 0x00000000,\
}
/*
Group-0: SS-A and SS-B (Frames 0,1,2)
Group-1: SS-A and SS_B with limits (Frames 3,4)
Group-2: MTM Combined (Frame 5)
*/
#define NUM_AIM_UTLM_GROUPS (8)
#define SECS_TO_20MS 		(50)
#define MAX_AIM_SAMPLE_TIME (999)

#ifdef _DONT_USE_
/* bit mask for each group no associated with a given node */
#define AIM_MODE_GROUP_LOOKUP                                            \
{                                                        \
/* MODE:               STB   DTM  YTM   CPM */ \
/* GROUP MASK */      0x00, 0x00, 0x00, 0x00,\
}
const tGDEF_UINT8  aimGroupLookup[NUM_MODES]           = AIM_MODE_GROUP_LOOKUP;

#define AIM_CONFIG_STAGE0 (0)
#define AIM_CONFIG_STAGE1 (1)
#endif

/*---------------------------------------------------------------------------
 * Typedefs
 */
typedef struct
{
   tGDEF_UINT16  azA;
   tGDEF_UINT16  azB;
   tGDEF_UINT16  elA;
   tGDEF_UINT16  elB;
} tsSAS_DATA;

/*---------------------------------------------------------------------------
 * Private Function Prototypes (declared as static; used only within this module)
 */
static void         ProcMtmData(long * pMagTlm, tGDEF_UINT32 * pData, teGDEF_BOOLEAN * pIsValid);
static void         Generate_SAS_To_AIM_Mapping(void);
static tGDEF_UINT32 SetMtqCmdVal(tGDEF_INT16 cmd[NUM_AXIS], tGDEF_UINT8 isValid[NUM_AXIS]);
/* YT TODO Not needed?
 * static void HINT_SetOpAimUnsolNode();
 * static tGDEF_UINT8 UTLM_GetAimGroupMask(tGDEF_UINT8 mode);
 * static void UTLM_SetSasTlmGroup(tGDEF_UINT8 groupId);
 */

/*---------------------------------------------------------------------------
 * Local Data (declared as static; used only within this module)
 */
/* mapping of SAS to AIM channels */
static const tGDEF_UINT8 HINT_aimSasMap[_FC_NUM_AIM][AIM_NUM_SAS] = AIM_SAS_MAP;
static tGDEF_UINT8       HINT_SasAimMap[_FC_NUM_SAS][AIM_NUM_SAS] = {{0}}; /* this map is filled by shell and gives for each sun sensor the AIMID,SASID on that AIM */

/*
 * SF+YT TODO: not sure if HINT_opAimNode needed
 * static tGDEF_UINT8 HINT_opAimNode            = CANADDR_AIM0;
 */
static tGDEF_UINT8 HINT_aimNode[_FC_NUM_AIM] = {CANADDR_AIM0, CANADDR_AIM1}; /*AJ - move initialiser */
static tGDEF_UINT8 HINT_aimFrameSize         = AIM_DEFAULT_FRAME_SIZE;
static tGDEF_UINT32 HINT_maxMtqCmd           = AIM_DEFAULT_MAX_UNSOL_CMD;

/* Solicited TLm vars */
static const tGDEF_UINT16 HINT_mtmTlmChan                                  = MTM_TLM_PACKETS;
static const tGDEF_UINT16 HINT_sasTlmChan[AIM_NUM_SAS][NUM_SAS_FRAMES]     = SAS_TLM_PACKETS;
static const tGDEF_UINT32 sasSeqNoLookup[AIM_NUM_SAS][UTLM_NUM_SAS_GROUPS] = UTLM_SAS_SEQNO_LOOKUP;
static const tGDEF_UINT32 aimSeqNoLookup[NUM_AIM_UTLM_GROUPS]              = AIM_GROUP_SEQNO_LOOKUP;

/* utlm data storage */
static tsSAS_DATA  UTLM_sasData[_FC_NUM_SAS]            = {{0}};
static tsMAG_STATE UTLM_mag                             = {{{0}},{0},{0}};

/* frame tracking */
static tGDEF_UINT32 UTLM_aimRecvFrame[_FC_NUM_AIM]      = {0};
static tGDEF_UINT32 UTLM_aimExpFrame[_FC_NUM_AIM]       = {0};
static tGDEF_UINT32 UTLM_aimDuplicateFrame[_FC_NUM_AIM] = {0};

/* static tGDEF_UINT32 UTLM_sasExpMask = SAS_EXP_MASK; */

static tGDEF_UINT8  UTLM_sasGroupId     = UTLM_SAS_RAW_GROUP_ID;
static tGDEF_UINT16 HINT_AimSampleTime  = AIM_SAMPLE_TIME;
static tGDEF_UINT16 HINT_AimActuateTime = AIM_ACTUATE_TIME;

/*---------------------------------------------------------------------------
 * Global Data
 */
teGDEF_BOOLEAN HMGR_MtqActuateCmdSent = GDEF_FALSE;


/*********** UNIT CONFIGURATION FUNCTIONS *********************/
void HINT_UpdateAimPeriod(tGDEF_UINT8 frameSize)
{
   tGDEF_UINT8  aimNo  = 0;
   tGDEF_UINT16 cmdNo  = 0;
   tGDEF_UINT32 cmdVal = 0;

   /* set the current AIM frame size */
   HINT_aimFrameSize = frameSize;

   /*
    * want to set up frame size independent of current mode (unsol/sol)
    * as this command might be send as part of mode transition from sol->unsol or other way round
    */

   /*send the commands for unsolicited operation*/
   cmdNo  = AIMCMD_UNSOL_FRAME;
   cmdVal = HINT_aimFrameSize;

   /* Send config commands for each AIM */
   for(aimNo = 0; aimNo < _FC_NUM_AIM; aimNo++)
   {
      /* update AIM frame size*/
      HMGR_ConfigCanCmd(HINT_aimNode[aimNo], cmdNo, cmdVal);
   }

   /* calculate max pulse width (see bugzilla #5027) */
   HINT_maxMtqCmd = (tGDEF_UINT32) ((double)  ((HINT_aimFrameSize-1)*1000 - HINT_AimActuateTime)/20.0);

   /*send the commands for solicited operation*/
   cmdNo  = AIMCMD_SOL_FRAME;
   cmdVal = HINT_aimFrameSize * SECS_TO_20MS; /* converts the value from secs to 20ms */

   /* Send config commands for each AIM */
   for(aimNo = 0; aimNo < _FC_NUM_AIM; aimNo++)
   {
      /* update AIM frame size*/
      HMGR_ConfigCanCmd(HINT_aimNode[aimNo], cmdNo, cmdVal);
   }
}


/*
 * set the working aim sample time
 * value must be less than 1000
 */
void HINT_SetAimSampleTime(tGDEF_UINT16 sampleTime)
{
   tGDEF_UINT8 aimNo = 0;

   /* check if valid time (0-999) */
   if(sampleTime > MAX_AIM_SAMPLE_TIME)
   {
      return;
   }

   HINT_AimSampleTime = sampleTime;

   /* Send config commands for each AIM */
   for(aimNo = 0; aimNo < _FC_NUM_AIM; aimNo++)
   {
      /* update AIM frame size*/
      HMGR_ConfigCanCmd(HINT_aimNode[aimNo], AIMCMD_T_SAMPLE, sampleTime);
   }

}

/*
 * set the working aim actuate time
 * value must be less than 1000
 */

void HINT_SetAimActuateTime(tGDEF_UINT16 actTime)
{
   tGDEF_UINT8 aimNo = 0;

   /* check if valid time (0-999) */
   if(actTime > MAX_AIM_SAMPLE_TIME)
   {
      return;
   }

   HINT_AimActuateTime = actTime;

   /* Send config commands for each AIM */
   for(aimNo = 0; aimNo < _FC_NUM_AIM; aimNo++)
   {
      /* update AIM frame size*/
      HMGR_ConfigCanCmd(HINT_aimNode[aimNo], AIMCMD_T_ACTUATE, actTime);
   }

}





/*********** ACTUATION COMMAND FUNCTIONS*********************/


/*!
 * \brief      Set MtqCmd Value
 *
 *             Converts the alg cmd value for mtq into correct format for tcmd
 *
 * \param  cmdVal  commanding values from algs
 * \return telecommand value
 ******************************************************************************/
tGDEF_UINT32 SetMtqCmdVal(tGDEF_INT16 cmd[NUM_AXIS], tGDEF_UINT8 isValid[NUM_AXIS])
{
   tGDEF_UINT32 retVal        = 0;
   double       mtqTimeFactor = 0;
   double       cmdVal        = 0;
   tGDEF_UINT8  axisNo        = 0;

   /* initialise the return value to zero */
   retVal        = 0UL;
   mtqTimeFactor = ((double) HINT_aimFrameSize * MTQ_UNIT_CONV ) / (MAX_MAG_MOMENT * MTQ_SCALE_FACTOR);


   /* add contribution from each axis */
   for (axisNo = 0; axisNo < NUM_AXIS; axisNo++)
   {
      if(GDEF_TRUE == isValid[axisNo])
      {
         /* convert the mag moment command to pulse width */
         cmdVal = ((double) cmd[axisNo] * mtqTimeFactor );

         /* check on cmd value is within correct limits (see bugzilla #5027) */
         if(AMOH_isUnsolMode && (fabs(cmdVal) > (double) HINT_maxMtqCmd))
            cmdVal = (cmdVal > 0) ? (double) HINT_maxMtqCmd : (-1 * (double) HINT_maxMtqCmd);


         /* store the 10 bit value in the result (will retain the sign bit: see AIM ICD) */
         retVal |=  ((tGDEF_INT32) cmdVal & 0x3FFUL) << (10 * axisNo);
      }
   }

   return retVal;
}


/*!
 * \brief      ActuateMtq
 *
 *             Actuate Magnetorquers
 *
 * \param  cmdVal  commanding values from algs
 *
 ******************************************************************************/
void HINT_ActuateMtq(tGDEF_INT16 cmdVal[][NUM_AXIS], tGDEF_UINT8 isValid[], tGDEF_UINT8 isEnabled[])
{
   tGDEF_UINT32 mtqVal = 0;
   tGDEF_UINT8  aimNo  = 0;
   tGDEF_UINT8  isValidAxis[NUM_AXIS] = {GDEF_TRUE, GDEF_TRUE, GDEF_TRUE};

   HMGR_MtqActuateCmdSent = GDEF_FALSE; /* flag to indicate if command is sent or not (on 4 sec cycle onlt 1 out of 4 sec a command is sent) */

   /* put the mtq command value in the correct format */
   for(aimNo = 0; aimNo < _FC_NUM_AIM; aimNo++)
   {
	   if ( isValid[aimNo] == GDEF_TRUE)
	   {
		  mtqVal = SetMtqCmdVal(cmdVal[aimNo], isValidAxis);

         if(isEnabled[aimNo])
         {
            /* send the command */
            HMGR_ActuateCanCmd(HINT_aimNode[aimNo], AIMCMD_MTQ, mtqVal);
            HMGR_MtqActuateCmdSent = GDEF_TRUE;

         }


      } /* end: if the torquer command to that AIM has been set by the algs */
   }
}


void Generate_SAS_To_AIM_Mapping(void)
{

   tGDEF_UINT8 aimNoID    = 0;
   tGDEF_UINT8 aimSasNoID = 0;
   tGDEF_UINT8 sasNo      = 0;
   tGDEF_UINT8 aimNo      = 0;
   tGDEF_UINT8 aimSasNo   = 0;

   /* for each sun sensor find the aim it is attached to and the sasID on that AIM */
   for (sasNo = 0; sasNo < _FC_NUM_SAS; sasNo ++)
   {
      /* find out which aim this sun sensor is connected to */
      aimNoID    = _FC_NUM_AIM;
      aimSasNoID = AIM_NUM_SAS;

      for (aimNo=0; aimNo < _FC_NUM_AIM; aimNo++)
      {
         for (aimSasNo = 0; aimSasNo < AIM_NUM_SAS; aimSasNo++)
         {
            if (sasNo == HINT_aimSasMap[aimNo][aimSasNo])
            {
               aimNoID = aimNo;
               aimSasNoID = aimSasNo;
               break;
            }
         }
      }

      if (aimNoID == _FC_NUM_AIM || aimSasNoID == AIM_NUM_SAS)
      {
         strcpy(EWOD_sLogMsg, "Generate_SAS_To_AIM_Mapping - Error did not find Mapping for sas " );
         sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", sasNo);
         strcat(EWOD_sLogMsg, "\n" );
         EWOD_WriteMessage(EWOD_sLogMsg);
      }

      HINT_SasAimMap[sasNo][0] = aimNoID;
      HINT_SasAimMap[sasNo][1] = aimSasNoID;

   }

}


/***********SOLICITED TLM FUNCTIONS *********************/
tGDEF_UINT16 HINT_InitSampleMtm(tsCANS_TcTlmSingle tlmPacket[], tGDEF_UINT8 mtmMask)
{

   tGDEF_UINT16 frameNo = 0;
   tGDEF_UINT8 mtmNo;

   /* set up telemtry packets */
   for(mtmNo = 0; mtmNo < _FC_NUM_MTM; mtmNo++)
   {
      /* check if mtm tlm is required */
      if(( (mtmMask >> mtmNo) & 0x01) == 0x01)
      {
         /* set up tlm packet Note mtmNo = aimNo*/
         tlmPacket[frameNo].stTcTlm.u1Dest = HINT_aimNode[mtmNo];
         tlmPacket[frameNo].stTcTlm.u2Chan = HINT_mtmTlmChan;
         frameNo++;
      }
   }

   return frameNo;
}


tGDEF_UINT16 HINT_InitSampleSas(tsCANS_TcTlmSingle  tlmPacket[], tGDEF_UINT8 sasMask)
{
   tGDEF_UINT16 frameNo    = 0;
   tGDEF_UINT16 sasFrameNo = 0;
   tGDEF_UINT8  sasNo      = 0; /* sasID */
   tGDEF_UINT8  aimNoID    = 0 ;
   tGDEF_UINT8  aimSasNoID = 0 ;

   Generate_SAS_To_AIM_Mapping();

   for (sasNo = 0; sasNo < _FC_NUM_SAS; sasNo++)
   {
      /* check if sas tlm is required */
      if(( (sasMask >> sasNo) & 0x01) == 0x01)
      {
         aimNoID = HINT_SasAimMap[sasNo][0];
         aimSasNoID = HINT_SasAimMap[sasNo][1];

         if (aimNoID == _FC_NUM_AIM || aimSasNoID == AIM_NUM_SAS)
         {
            strcpy(EWOD_sLogMsg, "HINT_InitSampleSas - Error did not find Mapping for sas " );
            sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", sasNo);
            strcat(EWOD_sLogMsg, "\n" );
            EWOD_WriteMessage(EWOD_sLogMsg);
            continue;
         }

         /* setup telemetry frame request for all the relevant frames of that sun sensor */
         for(sasFrameNo = 0; sasFrameNo < NUM_SAS_FRAMES; sasFrameNo++)
         {
            tlmPacket[frameNo].stTcTlm.u1Dest     = HINT_aimNode[aimNoID];
            tlmPacket[frameNo].stTcTlm.u2Chan  = HINT_sasTlmChan[aimSasNoID][sasFrameNo];
            frameNo++;
         }

      } /* if no telemetry needed go to next one */
   } /* for all sun sensors */

   return frameNo;
}

teGDEF_FUNC_STATUS HINT_SetSasSampleTlm(tsCANS_TcTlmSingle recvTlmBuf[], tGDEF_UCHAR sasMask, tsSAS_STATE *pTtc)
{
   teGDEF_BOOLEAN isTlmValid            = GDEF_FALSE;
   tGDEF_UINT32 tlmData[NUM_SAS_FRAMES] = {0};
   tGDEF_UINT8 frameNo                  = 0; /* counts from zero to the number of frames for sensor type received */
   tGDEF_UINT8 i                        = 0;
   tGDEF_UINT8 sasNo                    = 0;

   /* set up telemtry packets */
   for(sasNo = 0; sasNo < _FC_NUM_SAS; sasNo++)
   {
      /* initially set flag to valid */
      pTtc->valid[sasNo] = GDEF_TRUE;

      /* check if sas tlm is required */
      if(( (sasMask >> sasNo) & 0x01) == 0x01)
      {
         /* get tlm for each of the sas frames */
         for(i = 0; i < NUM_SAS_FRAMES; i++)
         {
            /* get suns sensor data from can frame, size 2 bytes
            needs to match length in ATTC_EwodDefs.h which is 2 byte*/
            HMGR_GetTlmData(&(recvTlmBuf[frameNo]), 2, &tlmData[i], &isTlmValid);

             if(isTlmValid == GDEF_FALSE) // at least one frame is missing for that sun sensor
             {
                pTtc->valid[sasNo] = GDEF_FALSE;
             }
             frameNo++;
          }

         /* store azimuth data */
         pTtc->tlm[sasNo][AZ_DATA][CHAN_A] = *(tGDEF_INT32 *) &tlmData[0];
         pTtc->tlm[sasNo][AZ_DATA][CHAN_B] = *(tGDEF_INT32 *) &tlmData[1];

         /* store elevation data */
         pTtc->tlm[sasNo][EL_DATA][CHAN_A] = *(tGDEF_INT32 *) &tlmData[2];
         pTtc->tlm[sasNo][EL_DATA][CHAN_B] = *(tGDEF_INT32 *) &tlmData[3];
      }
   }

   return GDEF_SUCCESS;
}


teGDEF_FUNC_STATUS HINT_SetMtmSampleTlm(tsCANS_TcTlmSingle recvTlmBuf[], tGDEF_UCHAR mtmMask, tsMAG_STATE *pTtc)
{

   tGDEF_UINT32 tlmData = 0;
   tGDEF_UINT16 frameNo = 0;
   tGDEF_UINT8  mtmNo   = 0;

   /*set up telemtry packets */
   for(mtmNo = 0; mtmNo < _FC_NUM_MTM; mtmNo++)
   {
      /* check if sas tlm is required */
      if(( (mtmMask >> mtmNo) & 0x01) == 0x01)
      {
         /* get mtm data from can frame, size 4 bytes */
         HMGR_GetTlmData(&recvTlmBuf[frameNo], 4, &tlmData, (teGDEF_BOOLEAN *)&(pTtc->valid[mtmNo]));

         /* set the ttc data */
         ProcMtmData((long *)pTtc->tlm[mtmNo], &tlmData, (teGDEF_BOOLEAN *)&(pTtc->valid[mtmNo]));

         // reset the magnetometer frame counter if have valid tlm
          if(pTtc->valid[mtmNo])
		  {
            pTtc->MTMFrameCounter[mtmNo] = 0;
         }

         /* update the number of tlm frames */
         frameNo++;
      }
   }

   return GDEF_SUCCESS;
}



void ProcMtmData(long * pMagTlm, tGDEF_UINT32 * pData, teGDEF_BOOLEAN * pIsValid)
{
   tGDEF_UINT8 axisNo = 0;

   if (*pIsValid)
   {
      for (axisNo = GDEF_X_AXIS; axisNo < NUM_AXIS; axisNo++)
      {
         pMagTlm[axisNo] = *pData >> (10 * axisNo); /* shift down 10 bits for given axis */
         pMagTlm[axisNo] &= 0x000003FF;             /* mask off higher bits */
         pMagTlm[axisNo] <<= 2;                     /* shift up 2 bits to get correct value */
      }
   }
   else
   {
      for (axisNo = GDEF_X_AXIS; axisNo < NUM_AXIS; axisNo++)
      {
         pMagTlm[axisNo] = AINT_INVALID_16BIT_TLM; /* needs to match length in ATTC_EwodDefs.h which is 2 byte */
      }
   }

}


/***********UNSOLICITED TLM FUNCTIONS *********************/
#ifdef _DONT_USE
void UTLM_AimInit(tGDEF_UINT8 mode)
{

   tGDEF_UINT8  groupNo;

   //initialse exp frame to 0
   UTLM_aimExpFrame  = 0UL;

   /* get the seqNo's for each of the groups */
   for (groupNo = 0; groupNo <  NUM_AIM_UTLM_GROUPS; groupNo++)
   {
      /* check if group is valid, OR seq mask to exp value */
      if(ValidBit(aimGroupLookup[mode], groupNo) )
         UTLM_aimExpFrame |= aimSeqNoLookup[groupNo];
   }

}
#endif

void UTLM_AIMInit(void)
{
   tGDEF_UINT8 unitNo = 0;

   for(unitNo = 0; unitNo < _FC_NUM_AIM; unitNo++)
   {
      /* reset the expected frame */
      UTLM_aimExpFrame[unitNo] = 0UL;
   }
}

void UTLM_AIMInitDataGathering(void)
{
   memset(&UTLM_mag, 0, sizeof(UTLM_mag));
   memset(UTLM_sasData, 0, sizeof(UTLM_sasData));
   memset(UTLM_aimRecvFrame, 0, sizeof(UTLM_aimRecvFrame));
   memset(UTLM_aimDuplicateFrame,0, sizeof(UTLM_aimDuplicateFrame));

}

void UTLM_MtmInit(tGDEF_UINT8 mask)
{
   tGDEF_UINT8 unitNo = 0;

   /*NOTE: aim and mtm have the same unit number*/
   for(unitNo = 0; unitNo < _FC_NUM_MTM; unitNo++)
   {
      /*check if unit to be included */
      if(ValidBit(mask, unitNo))
      {
         UTLM_aimExpFrame[unitNo] |= 0x01 << UTLM_MTM_SEQNO;
      }
   }
}


void UTLM_SasInit(tGDEF_UINT8 mask)
{
   tGDEF_UINT8  unitNo     		= 0;
   tGDEF_UINT8  aimNoID    		= 0;
   tGDEF_UINT8  aimSasNoID 	    = 0;
   tGDEF_UINT32 UTLM_sasExpMask = 0;

   /* get the sasID and aimID for each sun sensor */
   Generate_SAS_To_AIM_Mapping();

   /*NOTE: aim and mtm have the same unit number*/
   for(unitNo = 0; unitNo < _FC_NUM_SAS; unitNo++)
   {
      /* check if unit to be included */
      if(ValidBit(mask, unitNo) )
      {
         /* find out which aim this sun sensor is connected to */
         aimNoID = HINT_SasAimMap[unitNo][0];
         aimSasNoID = HINT_SasAimMap[unitNo][1];

         if (aimNoID == _FC_NUM_AIM || aimSasNoID == AIM_NUM_SAS)
         {
            strcpy(EWOD_sLogMsg, "UTLM_SasInit - Error did not find Mapping for sas " );
            sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", unitNo);
            strcat(EWOD_sLogMsg, "\n" );
            EWOD_WriteMessage(EWOD_sLogMsg);
            continue;
         }

         UTLM_sasExpMask = sasSeqNoLookup[aimSasNoID][UTLM_sasGroupId];
         UTLM_aimExpFrame[aimNoID] |= UTLM_sasExpMask;
      }
   }

}

#if 0
void UTLM_SetSasTlmGroup(tGDEF_UINT8 groupId)
{
   if(groupId <= UTLM_MAX_SAS_GROUP_ID)
      UTLM_sasGroupId = groupId;
}
#endif

teGDEF_BOOLEAN UTLM_IsAllAimDataReceived(void)
{
   tGDEF_UINT8    unitNo = 0;
   teGDEF_BOOLEAN retVal = GDEF_TRUE;

   for(unitNo = 0; unitNo < _FC_NUM_AIM; unitNo++)
   {
      /*
       *  check expected and received frames
       *  any additional data received to the one expected is okay
       */
      if ( (UTLM_aimRecvFrame[unitNo] & UTLM_aimExpFrame[unitNo]) != UTLM_aimExpFrame[unitNo])
      {
#ifdef REALTIME_DEBUG
         (void)printf("AIM unxpected frame received for %d r:%d e:%d\n",unitNo,UTLM_aimRecvFrame[unitNo],UTLM_aimExpFrame[unitNo]);
#endif /* REALTIME_DEBUG */
         /* log missing telemetry */
         strcpy(EWOD_sLogMsg, "Missing AIM [");
         sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%u", unitNo);
         strcat(EWOD_sLogMsg, "]unsol tlm: exp  0x");
         sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%x", UTLM_aimExpFrame[unitNo]);
         strcat(EWOD_sLogMsg, " act  0x");
         sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%x", UTLM_aimRecvFrame[unitNo]);
         strcat(EWOD_sLogMsg, "\n");
         EWOD_WriteMessage(EWOD_sLogMsg);

         retVal =  GDEF_FALSE;
      }

      //log if duplicates received
      if (UTLM_aimDuplicateFrame[unitNo] != 0UL)
      {
#ifdef REALTIME_DEBUG
         (void)printf("AIM duplicate frame received %d for %d\n",UTLM_aimDuplicateFrame[unitNo],unitNo);
#endif /* REALTIME_DEBUG */
         strcpy(EWOD_sLogMsg, "Duplicate  unsol - AIM: ");
         strcat(EWOD_sLogMsg, " received twice  0x");
         sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", UTLM_aimDuplicateFrame[unitNo]);
         strcat(EWOD_sLogMsg, "\n");
         EWOD_WriteMessage(EWOD_sLogMsg);
      }
   }

   /* return status of data received */
   return retVal;

}



/*!
 * \brief     HINT_SetOpAimUnsolNode
 *
 *             Sets the unsolicted node for the operational AIM
 *
 *
 ******************************************************************************/
#if 0
void HINT_SetOpAimUnsolNode()
{
   if (HINT_opAimNode == CANADDR_AIM0)
      HMGR_ConfigCanCmd(HINT_opAimNode, AIMCMD_UNSOL_NODE, CANADDR_AIM0_TLM);
   else
      HMGR_ConfigCanCmd(HINT_opAimNode, AIMCMD_UNSOL_NODE, CANADDR_AIM1_TLM);
}
#endif


/*!
 * \brief     UTLM_GetAimGroupMask
 *
 *              returns the tlm groups for the AIM for a given mode
 *
 * \param mode mode number
 * \return group mask (uint8)
 *
 ******************************************************************************/
#if 0
tGDEF_UINT8 UTLM_GetAimGroupMask(tGDEF_UINT8 mode)
{
   return aimGroupLookup[mode];
}
#endif


/*!
 * \brief     UTLM_ProcAimTlm
 *
 *              Processes an incoming unsolicted AIM telemetry frame
 *
 * \param aimNo AIM number (0/1) data received from
 * \param *pData pointer to tlm data
 *
 ******************************************************************************/
void UTLM_ProcAimTlm(const tGDEF_UINT8 aimNo, tGDEF_UCHAR * pData)
{
   /* sas number  */
   /* note: aim0 sends sas 0,1 data, aim1 sends sas 2,3 */
   tGDEF_UINT8   sasNo = 0;
   tGDEF_UINT8   mtmNo = 0;
   teUTLM_SEQ_NO seqNo = 0;

   /* get the sequence number */
   seqNo = pData[0] & SEQNO_BYTE_MASK;

   switch (seqNo)
   {
   case SEQ_NO_0:
      /* set sas no: for SS-A on that AIM*/
      sasNo = HINT_aimSasMap[aimNo][SAS0];

      /* Sun Sensor A and B Frame 1, Sequence number 0 */
      /* decoding SAS 0 */
      UTLM_sasData[sasNo].azA = GetFirst12Bits(pData);
      UTLM_sasData[sasNo].azB = GetSecond12Bits(pData);
      UTLM_sasData[sasNo].elA = GetThird12Bits(pData);
      break;
   case SEQ_NO_1:
      /* set sas no: for SS-A on that AIM */
      sasNo = HINT_aimSasMap[aimNo][SAS0];

      /*
       *  Sun Sensor A and B Frame 2, Sequence number 1
       *  decoding +XElevationB
       */
      UTLM_sasData[sasNo].elB = GetFirst12Bits(pData);

      /*  set sas no: for SS-B on that AIM */
      sasNo = HINT_aimSasMap[aimNo][SAS1];

      /* extract data from packet */
      UTLM_sasData[sasNo].azA = GetSecond12Bits(pData);
      UTLM_sasData[sasNo].azB = GetThird12Bits(pData);
      break;
   case SEQ_NO_2:
      /*  set sas no: for SS-B on that AIM  */
      sasNo = HINT_aimSasMap[aimNo][SAS1];

      /* Sun Sensor A and B Frame 3, Sequence number 2 */
      UTLM_sasData[sasNo].elA = GetFirst12Bits(pData);
      UTLM_sasData[sasNo].elB = GetSecond12Bits(pData);
      break;
   case SEQ_NO_5:
      /* set mtm no for the aim */
      mtmNo = (aimNo == 0) ? 0 : 1;

      /* get mtm data */
      UTLM_mag.tlm[mtmNo][GDEF_X_AXIS] = GetFirst12Bits(pData);
      UTLM_mag.tlm[mtmNo][GDEF_Y_AXIS] = GetSecond12Bits(pData);
      UTLM_mag.tlm[mtmNo][GDEF_Z_AXIS] = GetThird12Bits(pData);
      UTLM_mag.MTMFrameCounter[mtmNo] = ((tGDEF_UINT16) pData[5] & 0x00F0)  >> 4;

      /* set mtm validity flag */
      UTLM_mag.valid[mtmNo] = GDEF_TRUE;
      break;
   default:
   {
      strcpy(EWOD_sLogMsg, "ERROR: UTLM_ProcAimTlm  - unknown seqNo" );
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", seqNo);
      strcat(EWOD_sLogMsg, "\n" );
      EWOD_WriteMessage(EWOD_sLogMsg);
      return;

   }
   }

#ifdef AIM_COLD_REDUNDANT
   if (ACON_opUnit.aim == aimNo)
#endif


      //check if this packet has already been received
      if(ValidBit(UTLM_aimRecvFrame[aimNo], seqNo))
      {
         //update the duplicated frames
         UTLM_aimDuplicateFrame[aimNo] |= (0x01UL << seqNo);

      }
      else
      {
         //update the received frames
         UTLM_aimRecvFrame[aimNo] |= (0x01UL << seqNo);
      }

}


/*!
 * \brief     SetSasData
 *
 *              Sets the Sun sensor data & validity based on the received tlm
 *
 ******************************************************************************/
teGDEF_FUNC_STATUS UTLM_SetSasData(tsSAS_STATE *pTTC, tGDEF_UCHAR isEnabled[])
{
   tGDEF_UINT8    sasNo 		  = 0;
   tGDEF_UINT8    aimNoID         = 0;
   tGDEF_UINT8    aimSasNoID      = 0; /* defines the aim and sas ID for each sun sensor */
   teGDEF_BOOLEAN errorFlag       = GDEF_FALSE;
   tGDEF_UINT32   UTLM_sasExpMask = 0;

   /* calculate combined az data and store */

   /* loop through each sunsensor */
   for(sasNo = 0; sasNo < _FC_NUM_SAS; sasNo++)
   {
      /* get AIM number for given sunsensor */
      aimNoID = HINT_SasAimMap[sasNo][0];
      aimSasNoID = HINT_SasAimMap[sasNo][1];

      /* check if received all sas data and no duplicate telemetry */
      /* get the expected frames mask */
      UTLM_sasExpMask = sasSeqNoLookup[aimSasNoID][UTLM_sasGroupId];

      if (CheckMasks(UTLM_sasExpMask, UTLM_aimRecvFrame[aimNoID])
            && !CheckMasks(UTLM_sasExpMask,UTLM_aimDuplicateFrame[aimNoID]))
      {
         /* store azimuth data */
         pTTC->tlm[sasNo][AZ_DATA][CHAN_A] = UTLM_sasData[sasNo].azA;
         pTTC->tlm[sasNo][AZ_DATA][CHAN_B] = UTLM_sasData[sasNo].azB;

         /* store elevation data */
         pTTC->tlm[sasNo][EL_DATA][CHAN_A] = UTLM_sasData[sasNo].elA;
         pTTC->tlm[sasNo][EL_DATA][CHAN_B] = UTLM_sasData[sasNo].elB;

         /* set the validity flag if sunsensor's are enabled */
         pTTC->valid[sasNo] = GDEF_TRUE;
      }
      else
      {

         /* store azimuth data */
         pTTC->tlm[sasNo][AZ_DATA][CHAN_A] = AINT_INVALID_16BIT_TLM; // needs to match length in ATTC_EwodDefs.h which is 2 byte
         pTTC->tlm[sasNo][AZ_DATA][CHAN_B] = AINT_INVALID_16BIT_TLM;

         /* store elevation data */
         pTTC->tlm[sasNo][EL_DATA][CHAN_A] = AINT_INVALID_16BIT_TLM;
         pTTC->tlm[sasNo][EL_DATA][CHAN_B] = AINT_INVALID_16BIT_TLM;

         errorFlag = GDEF_TRUE;
      }
   }

   for(sasNo = 0; sasNo < _FC_NUM_SAS; sasNo++)
   {
      if (GDEF_FALSE == isEnabled[sasNo])
         pTTC->valid[sasNo] = isEnabled[sasNo];
   }

   if(errorFlag)
      return GDEF_FAILURE;
   else
      return GDEF_SUCCESS;

}

teGDEF_FUNC_STATUS UTLM_SetMagData(tsMAG_STATE *pTTC, tGDEF_UCHAR isEnabled[])
{
   tGDEF_UINT8 mtmNo = 0;

   for (mtmNo = 0; mtmNo < _FC_NUM_MTM; mtmNo++)
   {
      if ( CheckMasks( (0x01 << UTLM_MTM_SEQNO), UTLM_aimDuplicateFrame[mtmNo]) ) // duplicate measurement arrived
      {
#ifdef REALTIME_DEBUG
         (void)printf("Duplicate frames received for MTM%d\n",mtmNo);
#endif /* REALTIME_DEBUG */
         /*set contents of mtm back to invalid tlm*/
         UTLM_mag.tlm[mtmNo][GDEF_X_AXIS] = AINT_INVALID_16BIT_TLM; // needs to match length in ATTC_EwodDefs.h which is 2 byte
         UTLM_mag.tlm[mtmNo][GDEF_Y_AXIS] = AINT_INVALID_16BIT_TLM;
         UTLM_mag.tlm[mtmNo][GDEF_Z_AXIS] = AINT_INVALID_16BIT_TLM;
         UTLM_mag.valid[mtmNo] = GDEF_FALSE;

      }

      /* if frame not received set to invalid tlm*/
      if (!CheckMasks((0x01 << UTLM_MTM_SEQNO), UTLM_aimRecvFrame[mtmNo]))
      {
#ifdef REALTIME_DEBUG
         (void)printf("Not enough frames received for MTM%d\n",mtmNo);
#endif /* REALTIME_DEBUG */
         /*set contents of mtm back to 0*/
         UTLM_mag.tlm[mtmNo][GDEF_X_AXIS] = AINT_INVALID_16BIT_TLM; // needs to match length in ATTC_EwodDefs.h which is 2 byte
         UTLM_mag.tlm[mtmNo][GDEF_Y_AXIS] = AINT_INVALID_16BIT_TLM;
         UTLM_mag.tlm[mtmNo][GDEF_Z_AXIS] = AINT_INVALID_16BIT_TLM;
         UTLM_mag.valid[mtmNo] = GDEF_FALSE;
      }
   }

   for (mtmNo= 0; mtmNo < _FC_NUM_MTM; mtmNo++)
   {
      if( GDEF_FALSE == isEnabled[mtmNo])
      {
         UTLM_mag.valid[mtmNo] = isEnabled[mtmNo];
      }
   }

   /* store mtm data */
   *pTTC = UTLM_mag;

   return GDEF_SUCCESS;
}




