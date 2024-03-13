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
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/source/HINT_MWheel.c,v $
 * Revision    : $Revision: 1.6 $
 *
 * History:
 *
 * $Log: HINT_MWheel.c,v $
 * Revision 1.6  2016/03/14 10:47:46  ytrichakis
 * Register with the SKED library
 *
 * Revision 1.3  2013/07/18 15:56:53  ytrichakis
 * fixed wheel bug(DR#14548) and applied the same solution in case of failed AIM
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
#include <string.h>
#include <math.h>

#include "GDEF_GlobDefs.h"
#include "Adcs_mission.h"
#include "mission.h"
#include "Adcs_IntDefs.h"

//#include "EWOD_EwodHandler_OBC750.h"
#include "EWOD_EwodHandler.h"

#ifdef _MWHL_10SP_
#include "HINT_Wheel_10SP.h"
#else

#ifdef _MWHL_100SP_
#include "HINT_Wheel_100SP.h"
#endif

#endif

#include "HMGR_Globals.h"
#include "HMGR_HardwareManager.h"

/*-----------------------------------------------------------------------
 * Defines and Macros
 */
/* defined maximum num of wheels that could exist on a spacecraft */
#define MAX_NUM_MWHLS (12)

/*---------------------------------------------------------------------------
 * Typedefs
 */

/*---------------------------------------------------------------------------
 * Private Function Prototypes (declared as static; used only within this module)
 */


/*---------------------------------------------------------------------------
 * Local Data (declared as static; used only within this module)
 */
static tGDEF_UINT32  UTLM_mwhlRecvFrame[_FC_NUM_MWHL]          = {0};
static tGDEF_UINT32  UTLM_mwhlExpFrame[_FC_NUM_MWHL]           = {0};
static tsWHEEL_STATE UTLM_wheel                                = {{0},{0}};
static tGDEF_UINT32  UTLM_mwhlDuplicateRecvFrame[_FC_NUM_MWHL] = {0};

/* TODO Not needed?
static tGDEF_UINT8   HINT_numMWhl                              = 0;
static tGDEF_UCHAR   HINT_mwhlEnabled[_FC_NUM_MWHL]            = {0};
static tGDEF_UINT8   HINT_mwhlFailed                           = 0;
 */

/*---------------------------------------------------------------------------
 * Global Data
 */

tGDEF_UINT16 HINT_InitSampleMWhl(tsCANS_TcTlmSingle tlmPacket[], tGDEF_UINT8 whlMask)
{
   tGDEF_UINT16 frameNo = 0;
   tGDEF_UINT8  whlNo   = 0;

   /* set up telemtry packets */
   for(whlNo = 0; whlNo < _FC_NUM_MWHL; whlNo++)
   {
      /* check if whl tlm is required */
      if(( (whlMask >> whlNo) & 0x01) == 0x01)
      {
         /* set up tlm packet */
         tlmPacket[frameNo].stTcTlm.u1Dest = HMGR_wheelNode[whlNo];
         tlmPacket[frameNo].stTcTlm.u2Chan = MWHLTLM_SPEED;
         frameNo++;
      }
   }

   return frameNo;
}



teGDEF_FUNC_STATUS HINT_SetMWhlSampleTlm(tsCANS_TcTlmSingle recvTlmBuf[], tGDEF_UCHAR whlMask, tsWHEEL_STATE *pTtc)
{
   tGDEF_UINT16 frameNo = 0;
   tGDEF_UINT8  whlNo   = 0;

   /* set up telemtry packets */
   for(whlNo = 0; whlNo < _FC_NUM_MWHL; whlNo++)
   {
      /* check if sas tlm is required */
      if( ( (whlMask >> whlNo) & 0x01) == 0x01)
      {
         /* get wheel data from can frame, size 4 bytes */
         HMGR_GetTlmData(&recvTlmBuf[frameNo], 4, (tGDEF_UINT32 *)&(pTtc->tlm[whlNo]), (teGDEF_BOOLEAN *)&(pTtc->valid[whlNo]));

         /** extra logging for wheels - AJ - check if necessary and/or return failure ***/

         /* if timeout or NAK then write a message into the log */
         if (eCANS_TRAN_TLM_NAK == recvTlmBuf[frameNo].stTcTlm.e1ReturnStatus)
         {
            strcpy(EWOD_sLogMsg, "CheckSampleWhls - MW" );
            sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", (int)whlNo);
            strcat(EWOD_sLogMsg, " TLM_NAK (val (0x01)) \n" );
            EWOD_WriteMessage(EWOD_sLogMsg);
         }

         if (eCANS_TRAN_TIMEOUT == recvTlmBuf[frameNo].stTcTlm.e1ReturnStatus)
         {
            strcpy(EWOD_sLogMsg, "CheckSampleWhls - MW" );
            sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", (int)whlNo);
            strcat(EWOD_sLogMsg, " TLM_TIMEOUT (val (0x00)) \n" );
            EWOD_WriteMessage(EWOD_sLogMsg);
         }

         /* update the number of tlm frames */
         frameNo++;
      }
   }

   return GDEF_SUCCESS;
}




/***********UNSOLICITED TLM FUNCTIONS *********************/

void UTLM_MWhlInit(tGDEF_UINT8 mask)
{
   tGDEF_UINT8 whlNo = 0;

   //first reset the buffer
   for(whlNo = 0; whlNo < _FC_NUM_MWHL; whlNo++)
   {
	   UTLM_mwhlExpFrame[whlNo] = 0x00000000; //WHEEL_FRAME_MASK;
   }

   //set up telemtry packets
   for(whlNo = 0; whlNo < _FC_NUM_MWHL; whlNo++)
   {
      /* check if whl tlm is required */
      if(ValidBit(mask, whlNo) )
      {
         UTLM_mwhlExpFrame[whlNo] = WHEEL_FRAME_MASK;
#ifdef REALTIME_DEBUG
         (void)printf("valid for wheel: %d mask: %d frame: %d\n",whlNo,mask,UTLM_mwhlExpFrame[whlNo]);
#endif /* REALTIME_DEBUG */
      }
   }

}

void UTLM_MWInitDataGathering(void)
{
   memset(&UTLM_wheel, 0, sizeof(UTLM_wheel));
   memset(UTLM_mwhlRecvFrame, 0, sizeof(UTLM_mwhlRecvFrame));
   memset(UTLM_mwhlDuplicateRecvFrame,0,sizeof(UTLM_mwhlDuplicateRecvFrame));
}

teGDEF_BOOLEAN UTLM_IsAllMWhlDataReceived(void)
{
   tGDEF_UINT8 whlNo = 0;

   for(whlNo = 0; whlNo < _FC_NUM_MWHL; whlNo++)
   {
      /* check expected and recieved frames */
      if((UTLM_mwhlExpFrame[whlNo] & UTLM_mwhlRecvFrame[whlNo]) != UTLM_mwhlExpFrame[whlNo]) /* It was UTLM_mwhlRecvFrame, but it should be UTLM_mwhlExpFrame, note when you integrate latest TDS code */
      {
#ifdef REALTIME_DEBUG
         (void)printf("MWHL unxpected frame received for %d r:%d e:%d\n",whlNo,UTLM_mwhlRecvFrame[whlNo],UTLM_mwhlExpFrame[whlNo]);
#endif /* REALTIME_DEBUG */
         /* log missing telemetry */
         strcpy(EWOD_sLogMsg, "Missing microsat wheel unsol tlm, whl no: ");
         sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", whlNo);
         strcat(EWOD_sLogMsg, " exp  0x");
         sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%x", UTLM_mwhlExpFrame[whlNo]);
         strcat(EWOD_sLogMsg, " act  0x");
         sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%x", UTLM_mwhlRecvFrame[whlNo]);
         strcat(EWOD_sLogMsg, "\n");
         EWOD_WriteMessage(EWOD_sLogMsg);

         return GDEF_FALSE;
      }

      if (UTLM_mwhlDuplicateRecvFrame[whlNo] != 0UL)
      {
#ifdef REALTIME_DEBUG
         (void)printf("MWHL duplicate frame received %d for %d\n",UTLM_mwhlDuplicateRecvFrame[whlNo],whlNo);
#endif /* REALTIME_DEBUG */
         strcpy(EWOD_sLogMsg, "Duplicate Frame for RW-");
         sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", whlNo);
         strcat(EWOD_sLogMsg, " is 0x");
         sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%x", UTLM_mwhlDuplicateRecvFrame[whlNo]);
         strcat(EWOD_sLogMsg, "\n");
         EWOD_WriteMessage(EWOD_sLogMsg);
      }
   }

   /* no discrepencies so return true */
   return GDEF_TRUE;

}


/*!
 * \brief     UTLM_ProcwheelTlm
 *
 *              Proceses an incoming unsolicted microsat wheel telemetry frame
 *
 * \param wheelNo wheel number (0-3)
 * \param *pData pointer to tlm data
 *
 ******************************************************************************/
void UTLM_ProcWheelTlm(const tGDEF_UINT8 wheelNo, tGDEF_UCHAR * pData)
{
   tGDEF_UINT16 seqNo = 0;

   /* get the sequence number */
   seqNo = pData[0] & SEQNO_BYTE_MASK;

   /* if have the correct seq no store telemetry */
   if(seqNo == SEQ_NO_0)
   {
      UTLM_wheel.tlm[wheelNo]  =  (tGDEF_INT32)pData[1]; /* was casted to long before*/
      UTLM_wheel.tlm[wheelNo] += ((tGDEF_INT32)pData[2] << 8);
      UTLM_wheel.tlm[wheelNo] += ((tGDEF_INT32)pData[3] << 16);
      UTLM_wheel.tlm[wheelNo] += ((tGDEF_INT32)pData[4] << 24);

      /* set the wheel validity flag */
      UTLM_wheel.valid[wheelNo] = GDEF_TRUE;
   }

   /* check if this packet has already been received */
   if(ValidBit(UTLM_mwhlRecvFrame[wheelNo], seqNo))
   {
      /* update the duplicated frames */
      UTLM_mwhlDuplicateRecvFrame[wheelNo] |= (0x01UL << seqNo);
   }
   else
   {
      /* update the received frames */
      UTLM_mwhlRecvFrame[wheelNo] |= (0x01UL << seqNo);
   }

}

/*!
 * \brief     SetWheelData
 *
 *              Sets the Microsat wheel data & validity based on the received tlm & if enabled
 *
 ******************************************************************************/
teGDEF_FUNC_STATUS UTLM_SetWheelData(tsWHEEL_STATE *pTtc, tGDEF_UCHAR isEnabled[])
{
   tGDEF_UINT8 wheelNo = 0;

   for (wheelNo = 0; wheelNo < _FC_NUM_MWHL; wheelNo++)
   {
      if ( UTLM_mwhlDuplicateRecvFrame[wheelNo] != 0UL ) /* duplicate measurement arrived */
      {
         /*set contents of wheel back to Invalid*/
         UTLM_wheel.valid[wheelNo] = GDEF_FALSE;
         UTLM_wheel.tlm[wheelNo]   = AINT_INVALID_32BIT_TLM; /* needs to match length in ATTC_EwodDefs.h which is 4 byte */
      }

      if ( UTLM_mwhlRecvFrame[wheelNo] == 0UL ) /* no measurement arrived */
      {
         /*set contents of wheel back to Invalid*/
         UTLM_wheel.valid[wheelNo] = GDEF_FALSE;
         UTLM_wheel.tlm[wheelNo] = AINT_INVALID_32BIT_TLM; /* needs to match length in ATTC_EwodDefs.h which is 4 byte */
      }
   }

   for (wheelNo = 0; wheelNo < _FC_NUM_MWHL; wheelNo++)
   {
      if (GDEF_FALSE == isEnabled[wheelNo])
         UTLM_wheel.valid[wheelNo] = isEnabled[wheelNo];
   }

   *pTtc = UTLM_wheel;

   return GDEF_SUCCESS;
}

/*!
 * \brief      ActuateWheels
 *
 *             Actuate Microsat wheels
 *
 * \param  cmdVal  commanding values from algs
 *
 ******************************************************************************/
void HINT_ActuateMWheels(tGDEF_INT32 cmdVal[_FC_NUM_MWHL], tGDEF_UINT8 isValid[_FC_NUM_MWHL], tGDEF_UINT8 isEnabled[_FC_NUM_MWHL] )
{
   tGDEF_UINT8 wheelNo = 0;

   /* loop through each wheel */
   for (wheelNo = 0; wheelNo < _FC_NUM_MWHL; wheelNo++)
   {
      if( isValid[wheelNo] && isEnabled[wheelNo])
      {
         /* send the telecommands to the msat wheel */
         HMGR_ActuateCanCmd(HMGR_wheelNode[wheelNo],MWHLCMD_SPEED, cmdVal[wheelNo]);
      }
   }
}



