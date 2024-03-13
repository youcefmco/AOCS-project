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
 * Last Update : $Date: 2013/04/11 13:42:58 $
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/include/HINT_MWheel.h,v $
 * Revision    : $Revision: 1.1 $
 *
 * History:
 *
 * $Log: HINT_MWheel.h,v $
 * Revision 1.1  2013/04/11 13:42:58  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/

#ifndef __HINT_MWHEEL_H
#define __HINT_MWHEEL_H

/*---------------------------------------------------------------------------
 * Includes
 */

/*---------------------------------------------------------------------------
 * Defines and Macros (shared within this module)
 */

/*---------------------------------------------------------------------------
 * Typedefs (shared within this module)
 */

/*---------------------------------------------------------------------------
 * Data - (shared within this module)
 */
 
/*---------------------------------------------------------------------------
 * Module Function Prototypes
 */
tGDEF_UINT16       HINT_InitSampleMWhl(tsCANS_TcTlmSingle tlmPacket[], tGDEF_UINT8 whlMask);
teGDEF_FUNC_STATUS HINT_SetMWhlSampleTlm(tsCANS_TcTlmSingle recvTlmBuf[], tGDEF_UCHAR whlMask, tsWHEEL_STATE *pTtc);
void               UTLM_ProcWheelTlm(const tGDEF_UINT8  wheelNo, tGDEF_UCHAR * pData);
void               HINT_ActuateMWheels(tGDEF_INT32 cmdVal[_FC_NUM_MWHL], tGDEF_UINT8 isValid[_FC_NUM_MWHL], tGDEF_UINT8 isEnabled[_FC_NUM_MWHL] );
void               UTLM_MWhlInit(tGDEF_UINT8 mask);
teGDEF_BOOLEAN     UTLM_IsAllMWhlDataReceived(void);
void               UTLM_MWInitDataGathering(void);
teGDEF_FUNC_STATUS UTLM_SetWheelData(tsWHEEL_STATE *pTtc, tGDEF_UCHAR isEnabled[]);

#endif /* __HINT_10SP_H */
