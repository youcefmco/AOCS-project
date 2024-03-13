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
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/include/AMOH_AocsModeHandler.h,v $
 * Revision    : $Revision: 1.3 $
 *
 * History:
 *
 * $Log: AMOH_AocsModeHandler.h,v $
 * Revision 1.3  2013/07/18 15:56:52  ytrichakis
 * fixed wheel bug(DR#14548) and applied the same solution in case of failed AIM
 *
 * Revision 1.2  2013/06/24 13:13:24  ytrichakis
 * Removed TODO's as agreed between YT and AJ (07/06/13)
 *
 * Revision 1.1  2013/04/11 13:42:58  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/

#ifndef __AMOH_H
#define __AMOH_H

/*---------------------------------------------------------------------------
 * Includes
 */
#include "GDEF_GlobDefs.h"

/*---------------------------------------------------------------------------
 * Defines and Macros (shared within this module)
 */

/*---------------------------------------------------------------------------
 * Typedefs (shared within this module)
 */
/*mode transition state */
typedef enum
{
   AMOH_TRANS_IDLE   = 0,  //no transition occurring
   AMOH_TRANS_REQ    = 1,  //transition requested 
   AMOH_TRANS_CONFIG = 2, // hardware configuration
   AMOH_TRANS_SHELL  = 3, // set shell parameters
   AMOH_TRANS_ALGS   = 4, //set algorithm parameters
} teAMOH_TRANS_STATE;

/*---------------------------------------------------------------------------
 * Data - (shared within this module)
 */
extern teGDEF_BOOLEAN AMOH_isUnsolMode; 

/*---------------------------------------------------------------------------
 * Module Function Prototypes
 */
tGDEF_UINT8        AMOH_GetMode();
teGDEF_BOOLEAN     AMOH_GetUnsolModeFlag();
tGDEF_UINT32       AMOH_GetUnsolModeArray();
void               AMOH_SetUnsolMode( tGDEF_UINT8 mode, teGDEF_BOOLEAN value);
void               AMOH_UpdateMode();
teGDEF_FUNC_STATUS AMOH_SafeModeTransition(tGDEF_UINT8 mode);
teGDEF_FUNC_STATUS AMOH_SetSafeMode( teGDEF_BOOLEAN value, tGDEF_UINT8 mode);
teGDEF_BOOLEAN     AMOH_GetSafeMode();
teGDEF_FUNC_STATUS AMOH_RequestMode(unsigned char newMode);
void               AMOH_ForceMode(unsigned char newMode);
teGDEF_BOOLEAN     AMOH_isAllSafeSkedsRegistered(void);
tGDEF_UINT8        AMOH_GetReqMode();
#ifdef _DONT_USE_
teGDEF_BOOLEAN AMOH_GetResetFlag();
void AMOH_ClearResetFlag();
#endif 

#endif /* __AMOH_H */



