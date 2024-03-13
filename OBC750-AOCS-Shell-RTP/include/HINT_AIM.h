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
 * Last Update : $Date: 2013/09/06 13:40:22 $
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/include/HINT_AIM.h,v $
 * Revision    : $Revision: 1.2 $
 *
 * History:
 *
 * $Log: HINT_AIM.h,v $
 * Revision 1.2  2013/09/06 13:40:22  ytrichakis
 * removed unguarded printfs
 *
 * Revision 1.1  2013/04/11 13:42:58  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/

#ifndef __HINT_AIM_H_
#define __HINT_AIM_H_

/*---------------------------------------------------------------------------
 * Includes
 */
#include <mqueue.h>
#include <pthread.h>
#include "GDEF_GlobDefs.h"
#include "Adcs_IntDefs.h"
#include "CANS_Interface.h"
#include "CANS_API.h"
#include "CANA_ServerApiLib.h"

/*---------------------------------------------------------------------------
 * Defines and Macros (shared within this module)
 */
/* AIM commands */
#define AIMCMD_MTQ         (1)
#define AIMCMD_SOL_FRAME   (2)
#define AIMCMD_SYNCH_MODE  (5)
#define AIMCMD_GROUP_MASK  (6)
#define AIMCMD_T_ACTUATE   (7)
#define AIMCMD_T_SAMPLE    (8)
#define AIMCMD_UNSOL_FRAME (9)
#define AIMCMD_UNSOL_NODE  (10)

/* AIM Telemetry */
#define AIMTLM_FRAME (37)
#define AIMTLM_MTM   (43)

#define AIMTLM_SAS0_AZ_A   (21)
#define AIMTLM_SAS0_AZ_B   (22)
#define AIMTLM_SAS0_EL_A   (23)
#define AIMTLM_SAS0_EL_B   (24)

//AJ check this
#define AIMTLM_SAS1_AZ_A   (25)
#define AIMTLM_SAS1_AZ_B   (26)
#define AIMTLM_SAS1_EL_A   (27)
#define AIMTLM_SAS1_EL_B   (28)

#define NUM_SAS_FRAMES     (4)

#define SAS_TLM_PACKETS \
{\
   {AIMTLM_SAS0_AZ_A,\
   AIMTLM_SAS0_AZ_B,\
   AIMTLM_SAS0_EL_A,\
   AIMTLM_SAS0_EL_B},\
\
   {AIMTLM_SAS1_AZ_A,\
   AIMTLM_SAS1_AZ_B,\
   AIMTLM_SAS1_EL_A,\
   AIMTLM_SAS1_EL_B},\
}

#define MTM_TLM_PACKETS        AIMTLM_MTM
#define AIM_DEFAULT_FRAME_SIZE (10)  /* default frame size of AIM on power up */

/*---------------------------------------------------------------------------
 * Typedefs (shared within this module)
 */

/*---------------------------------------------------------------------------
 * Data - (shared within this module)
 */
 extern teGDEF_BOOLEAN HMGR_MtqActuateCmdSent;
 
/*---------------------------------------------------------------------------
 * Module Function Prototypes
 */
void               UTLM_AIMInit(void);
tGDEF_UINT16       HINT_InitSampleMtm(tsCANS_TcTlmSingle  tlmPacket[], tGDEF_UINT8 mtmMask);
tGDEF_UINT16       HINT_InitSampleSas(tsCANS_TcTlmSingle  tlmPacket[], tGDEF_UINT8 sasMask);
teGDEF_FUNC_STATUS HINT_SetSasSampleTlm(tsCANS_TcTlmSingle recvTlmBuf[], tGDEF_UCHAR sasMask, tsSAS_STATE *pTtc);
teGDEF_FUNC_STATUS HINT_SetMtmSampleTlm(tsCANS_TcTlmSingle recvTlmBuf[], tGDEF_UCHAR mtmMask, tsMAG_STATE *pTtc);
void               UTLM_ProcAimTlm(const tGDEF_UINT8 aimNo, tGDEF_UCHAR * pData);
void               HINT_ActuateMtq(tGDEF_INT16 cmdVal[][NUM_AXIS], tGDEF_UINT8 isValid[], tGDEF_UINT8 isEnabled[]);
void               UTLM_MtmInit(tGDEF_UINT8 mask);
void               UTLM_SasInit(tGDEF_UINT8 mask);
teGDEF_BOOLEAN     UTLM_IsAllAimDataReceived(void);
teGDEF_FUNC_STATUS UTLM_SetMagData(tsMAG_STATE *pTTC, tGDEF_UCHAR isEnabled[]);
teGDEF_FUNC_STATUS UTLM_SetSasData(tsSAS_STATE *pTTC, tGDEF_UCHAR isEnabled[]);
void               HINT_UpdateAimPeriod(tGDEF_UINT8 frameSize);
void               HINT_SetAimSampleTime(tGDEF_UINT16 sampleTime);
void               HINT_SetAimActuateTime(tGDEF_UINT16 actTime);
void               UTLM_AIMInitDataGathering(void);

#endif /* __HINT_AIM_H */
