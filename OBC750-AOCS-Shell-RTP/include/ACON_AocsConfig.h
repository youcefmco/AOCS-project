/******************************************************************************
 * Project        OBC 386 Flight Code
 * Subsystem      AOCS Task - AOCS Shell Config
 * Author         Allon Jameson
 * Date           05/09/2008
 ******************************************************************************/
/*!****************************************************************************
 * \file ACON_AOCSConfig.h
 *   <detailed description of the module functionality>
 *
 ******************************************************************************/
/******************************************************************************
 * Copyright (c) 2008 Surrey Satellite Technology, Ltd.
 * All rights reserved.
 *
 ******************************************************************************
 * Documentation : N/A
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
 * Last Update : $Date: 2013/04/11 13:42:57 $
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/include/ACON_AocsConfig.h,v $
 * Revision    : $Revision: 1.1 $
 *
 * History:
 *
 * $Log: ACON_AocsConfig.h,v $
 * Revision 1.1  2013/04/11 13:42:57  ytrichakis
 * Initial Commit
 *
 * Revision 1.3  2012/10/24 15:50:56  SFingerloos
 * added function to clear specific timer - used for unsolicited mode on 4 sec cycle to clear MTQ timer for cycles where no cmd is sent
 *
 * Revision 1.2  2012/04/20 11:16:13  SFingerloos
 * added function to check if configskeds are loaded for all safe modes, added safe mode offset for sked
 *
 * Revision 1.1  2012/02/21 15:22:26  ajameson
 * initial check in
 *
 * Revision 1.1  2012/02/09 17:49:41  ajameson
 * code updated following integration of shell lib and stubbed code
 *
 * Revision 1.1  2012/02/09 14:33:14  ajameson
 * initial check in of AOCS shell library functions
 *
 *
 ******************************************************************************/
#ifndef _ACON_H_
#define _ACON_H_

/*---------------------------------------------------------------------------
 * Includes
 */
#include <stdio.h>
#include "GDEF_GlobDefs.h"

/*---------------------------------------------------------------------------
 * Defines and Macros (shared within this module)
 */
#define TIMER_START_ALGS 0
#define TIMER_END_ALGS   1
#define TIMER_MTQ_ACK    2
#define TIMER_WHEEL_ACK  3

#define TIMER_FIRST_UNSOL 4
#define TIMER_LAST_UNSOL  5

#define ACON_NO_DRIVE_FILE (0UL)

#define TIMERS_A (0)
#define TIMERS_B (1)
#define NUM_TIMERS_TLM (2)

#define ACON_SAFE_SKED_OFFSET (10)

/*---------------------------------------------------------------------------
 * Typedefs (shared within this module)
 */
typedef enum
{
   ACON_RATE_LOW = 0,
   ACON_RATE_HIGH = 1,
} teACON_RATE_TYPE;

/*---------------------------------------------------------------------------
 * Data - (shared within this module)
 */
extern tGDEF_UINT32 ACON_TimerTlm[NUM_TIMERS_TLM]; 

/*---------------------------------------------------------------------------
 * Module Function Prototypes
 */
tGDEF_UINT32 ACON_GetShellStatus();
void ACON_SetTimer(tGDEF_UINT8 timerNo);
void ACON_ClearTimer(tGDEF_UINT8 timerNo);
void ACON_SetRate(tGDEF_UINT8 mode);
teGDEF_FUNC_STATUS ACON_StartSked(tGDEF_UINT32 id);
teGDEF_BOOLEAN ACON_GetSkedStatus(void);
teGDEF_BOOLEAN ACON_isSkedRegistered(tGDEF_UINT32 id);

/* TODO Remove?*/
teGDEF_FUNC_STATUS ACON_AbortSked(void);

#endif
