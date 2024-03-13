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
 * Last Update : $Date: 2013/04/11 13:42:59 $
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/include/HMGR_HardwareManager.h,v $
 * Revision    : $Revision: 1.1 $
 *
 * History:
 *
 * $Log: HMGR_HardwareManager.h,v $
 * Revision 1.1  2013/04/11 13:42:59  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/

#ifndef __HMGR_H
#define __HMGR_H

/*---------------------------------------------------------------------------
 * Includes
 */
#include <mqueue.h>
#include <pthread.h>

#include "CANS_Interface.h"
#include "CANS_API.h"
#include "CANA_ServerApiLib.h"
#include "HMGR_Globals.h"

/*---------------------------------------------------------------------------
 * Defines and Macros (shared within this module)
 */
#define HMGR_HERITAGE_STATUS (0) /* units up to CPM: 2x MTM, 2x MTQ, 4x wheels, 4x sun sensors      */
#define HMGR_ADD_UNIT_STATUS (1) /* additional units i.e. 2nd wheel type, gyros, star trackers, gps */
#define CheckMasks(X,Y) ((teGDEF_BOOLEAN)((X & Y) == X))
/* seq corresponding to the group */
#define SEQNO_BYTE_MASK (0x1F)
/* wheel frame mask - only expect seqNo 0 */
#define WHEEL_FRAME_MASK (0x00000001)
/* Macro to calculate if a bit is Set froma given mask */
#define ValidBit(MASK, BIT) ( (0x01 & (MASK >> BIT) ) == 0x01)

/* Macros for extracting 12 its of data from CAN Stream frame - P is pointer to first byte (seq No) */
#define GetFirst12Bits(P)     (((tGDEF_UINT16) P[1] & 0x00FF)      ) | \
                              (((tGDEF_UINT16) P[2] & 0x000F)  << 8);


#define GetSecond12Bits(P)    (((tGDEF_UINT16) P[2] & 0x00F0)  >> 4) | \
                              (((tGDEF_UINT16) P[3] & 0x00FF)  << 4);

#define GetThird12Bits(P)    (((tGDEF_UINT16) P[4] & 0x00FF)      ) | \
                             (((tGDEF_UINT16) P[5] & 0x000F)  << 8);
#ifdef _INC_PROP_
#define PROP_ACTION_DISABLE (0)
#define PROP_ACTION_ATT   (1)
#define PROP_ACTION_CORR   (2)
#define PROP_ACTION_FIRE_PROP0 (3)
#define PROP_ACTION_FIRE_PROP1 (4)


teGDEF_FUNC_STATUS HMGR_PropHandler(tGDEF_UINT8 action, tGDEF_UINT32 value);

#endif

/*---------------------------------------------------------------------------
 * Typedefs (shared within this module)
 */
typedef enum
{
   SEQ_NO_0  =  0,
   SEQ_NO_1  =  1,
   SEQ_NO_2  =  2,
   SEQ_NO_3  =  3,
   SEQ_NO_4  =  4,
   SEQ_NO_5  =  5,
   SEQ_NO_6  =  6,
   SEQ_NO_7  =  7,
   SEQ_NO_8  =  8,
   SEQ_NO_9  =  9,
   SEQ_NO_10 = 10,
   SEQ_NO_11 = 11,
   SEQ_NO_12 = 12,
   SEQ_NO_13 = 13,
   SEQ_NO_14 = 14,
   SEQ_NO_15 = 15,
   SEQ_NO_16 = 16,
   SEQ_NO_17 = 17,
   SEQ_NO_18 = 18,
   SEQ_NO_19 = 19,
   SEQ_NO_20 = 20,
   SEQ_NO_21 = 21,
   SEQ_NO_22 = 22,
   SEQ_NO_23 = 23,
   SEQ_NO_24 = 24,
   SEQ_NO_25 = 25,
   SEQ_NO_26 = 26,
   SEQ_NO_27 = 27,
   SEQ_NO_28 = 28,
   SEQ_NO_29 = 29,
   SEQ_NO_30 = 30,
   SEQ_NO_31 = 31,
   SEQ_NO_32 = 32,
   SEQ_NO_33 = 33,
   SEQ_NO_34 = 34,
   SEQ_NO_35 = 35,
   SEQ_NO_36 = 36,
   SEQ_NO_37 = 37,
   SEQ_NO_38 = 38,
   SEQ_NO_39 = 39,
   SEQ_NO_40 = 40,
   SEQ_NO_41 = 41,
   SEQ_NO_42 = 42,
   SEQ_NO_43 = 43,
   SEQ_NO_44 = 44,
   SEQ_NO_45 = 45,
   SEQ_NO_46 = 46,
   SEQ_NO_47 = 47,
} teUTLM_SEQ_NO;

/*---------------------------------------------------------------------------
 * Data - (shared within this module)
 */
 
/*---------------------------------------------------------------------------
 * Module Function Prototypes
 */
void               HMGR_Actuate(void);
void               HMGR_InitMode(tGDEF_UINT8 mode);
tGDEF_UINT32       HMGR_GetStatus(tGDEF_UINT8 statusType);
teGDEF_BOOLEAN     HMGR_CheckNoAckErrors(void);
/* telemetry interface functions */
void               HMGR_InitSample(void);
/*util function for tlm gathering - AJ - move to ACOM_? */
void               HMGR_GetTlmData(tsCANS_TcTlmSingle *pTlm, tGDEF_UINT8 numBytes, tGDEF_UINT32 * pData, teGDEF_BOOLEAN * pIsValid);
teGDEF_FUNC_STATUS HMGR_SetSensorLogState(teHMGR_HW_SENSORS sensor, tGDEF_UINT8 snsNo, teGDEF_BOOLEAN state);
teGDEF_FUNC_STATUS HMGR_ProcUnsolTlm(tGDEF_UINT8 node, tGDEF_UINT16 chan, tGDEF_UINT8 TS, tGDEF_UCHAR * pData);
teGDEF_FUNC_STATUS HMGR_PrepareData(void);
void               HMGR_ConfigCanCmd(tGDEF_UINT8 node, tGDEF_UINT16 cmdNo, tGDEF_UINT32 val);
void               HMGR_ActuateCanCmd(tGDEF_UINT8 node, tGDEF_UINT16 cmdNo, tGDEF_UINT32 val);
void               UTLM_InitMode(tGDEF_UINT8 mode);
void               UTLM_InitDataGathering(void);
teGDEF_BOOLEAN     UTLM_IsAllDataReceived(void);

#ifdef _SOLICITED_
teGDEF_FUNC_STATUS HMGR_Sample(void);
void               HMGR_CheckSample(void);
void               HMGR_ReqInitSample(void);
#endif

#if _FC_NUM_DPU != 0
void               HMGR_Cmd_Update(void);
#endif

#endif /* __HMGR_H */
