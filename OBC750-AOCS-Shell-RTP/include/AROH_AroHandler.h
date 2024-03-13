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
 * Last Update : $Date: 2013/07/18 15:56:52 $
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/include/AROH_AroHandler.h,v $
 * Revision    : $Revision: 1.2 $
 *
 * History:
 *
 * $Log: AROH_AroHandler.h,v $
 * Revision 1.2  2013/07/18 15:56:52  ytrichakis
 * fixed wheel bug(DR#14548) and applied the same solution in case of failed AIM
 *
 * Revision 1.1  2013/04/11 13:42:58  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/

#ifndef __AROH_H
#define __AROH_H

/*---------------------------------------------------------------------------
 * Includes
 */

/*---------------------------------------------------------------------------
 * Defines and Macros (shared within this module)
 */

/*---------------------------------------------------------------------------
 * Typedefs (shared within this module)
 */
typedef enum
{
   AROH_ALGFDIR_ERR   = 0,
   AROH_CMD_ACK_ERR   = 1,
   AROH_UNSOL_TLM_ERR = 2,   
   AROH_ACTUATE_ERR   = 3,  
   AROH_ALG_ERR       = 4,
} teAROH_ERRORS;

/*---------------------------------------------------------------------------
 * Data - (shared within this module)
 */
extern teGDEF_BOOLEAN AROH_isReconfigEnabled;
extern tGDEF_UINT32   AROH_errors; 
extern tGDEF_UINT8    AROH_reconfigState; 

/*---------------------------------------------------------------------------
 * Module Function Prototypes
 */
teGDEF_FUNC_STATUS AROH_SetCounterP(tGDEF_UINT16 value);
teGDEF_FUNC_STATUS AROH_SetCounterQ(tGDEF_UINT16 value);
teGDEF_FUNC_STATUS AROH_SetCounterMax(tGDEF_UINT8 mode, tGDEF_UINT16 value);
tGDEF_UINT32       AROH_GetStatus(void);
void               AROH_SetDecFlag(tGDEF_UINT8 errNo);
void               AROH_Update(void);
void               AROH_Reset(void);
void               AROH_SetHoldOff(tGDEF_UINT8 mode);
void               AROH_SetHoldoffVal(tGDEF_UINT8 mode, tGDEF_UINT32 period);
tGDEF_UINT32       AROH_GetHoldoffVal(void);
void               AROH_Enable(teGDEF_BOOLEAN isEnabled);
void               AROH_MathError(void);

#endif /* __AROH_H */
