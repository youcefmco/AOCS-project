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
 * Last Update : $Date: 2015/11/17 14:22:06 $
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/include/EWOD_EwodHandler.h,v $
 * Revision    : $Revision: 1.4 $
 *
 * History:
 *
 * $Log: EWOD_EwodHandler.h,v $
 * Revision 1.4  2015/11/17 14:22:06  ytrichakis
 * Updated for new messgae handling + sked
 *
 * Revision 1.2  2013/07/18 15:56:52  ytrichakis
 * fixed wheel bug(DR#14548) and applied the same solution in case of failed AIM
 *
 * Revision 1.1  2013/04/11 13:42:58  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/

#ifndef __EWOD_H
#define __EWOD_H

/*---------------------------------------------------------------------------
 * Includes
 */
#include <string.h>
#include "GDEF_GlobDefs.h"

/*---------------------------------------------------------------------------
 * Defines and Macros (shared within this module)
 */
#define EWOD_MAX_LOGMSG_SIZE    (200)
#define MAX_NUMBER_OF_CHANNELS  (0x00C8)            /*!< Maximum Number Of Extended Whole Orbit Data Channels, 200 for now             */

/*---------------------------------------------------------------------------
 * Typedefs (shared within this module)
 */
typedef struct
{
   tGDEF_UCHAR  len;
   tGDEF_UCHAR  formatId;
}tsEWOD_INFO;

typedef struct
{
   tGDEF_UINT16  chan;
   tsEWOD_INFO   ewod;
   tGDEF_UINT32  *pData;
}tsEWOD_TLM;

typedef struct
{
   tGDEF_UINT32 type;
   tGDEF_UINT16 chan;
   tsEWOD_INFO  ewod;
}tsEWOD_CHAN_LOOKUP;

typedef struct
{
   tGDEF_UINT8  node;
   tGDEF_UINT16 cmd;
   tGDEF_UINT32 value;
   tGDEF_UINT32 result;
   tGDEF_UCHAR  state;
}tsEWOD_COMMAND_MSG;

/*---------------------------------------------------------------------------
 * Data - (shared within this module)
 */
extern tGDEF_CHAR EWOD_sLogMsg[EWOD_MAX_LOGMSG_SIZE];
 
/*---------------------------------------------------------------------------
 * Module Function Prototypes
 */
void EWOD_Update(void);
void EWOD_WriteMessage(tGDEF_CHAR * pMsg);
void EWOD_CloseLog(void);
void EWOD_ZipFile(teGDEF_BOOLEAN zipFlag);
void EWOD_SetPeriod(tGDEF_UINT8 mode);
#ifdef _DONT_USE_
void EWOD_SetWodPeriod(tGDEF_UINT8 period, tGDEF_UINT8 type);
void EWOD_SetWodRateMultiplier(tGDEF_UINT8 multiple, tGDEF_UINT8 type);
#endif
void EWOD_SetWodPeriodArray(tGDEF_UINT8 mode,  tGDEF_UINT8 value);
void EWOD_InitMode(tGDEF_UINT8 mode);
/* tlm mask functions */
void EWOD_AddTlmType(tGDEF_UINT8 mode, tGDEF_UINT8 typeNo);
void EWOD_RemoveTlmType(tGDEF_UINT8 mode, tGDEF_UINT8 typeNo);
void EWOD_SetDefaultTlmType(tGDEF_UINT8 mode);
tGDEF_UINT32 EWOD_GetModeMask(void);

#endif /* __EWOD_H */
