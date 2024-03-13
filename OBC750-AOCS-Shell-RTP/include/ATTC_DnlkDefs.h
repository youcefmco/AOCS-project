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
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/include/ATTC_DnlkDefs.h,v $
 * Revision    : $Revision: 1.1 $
 *
 * History:
 *
 * $Log: ATTC_DnlkDefs.h,v $
 * Revision 1.1  2013/04/11 13:42:58  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/

#ifndef __ATTC_DNLK_DEFS_H_
#define __ATTC_DNLK_DEFS_H_

/*---------------------------------------------------------------------------
 * Includes
 */
#include "Adcs_ttcDefs.h"

/*---------------------------------------------------------------------------
 * Defines and Macros (shared within this module)
 */
/*SF looks like we can only support up to 33 channels?*/

#define ADLK_TLM_TABLE \
{\
   /* chan                      len */\
   {TLM_DRIVE_FNUM,       2},\
   /*TLM chans +1, gap between chans + 1 ==> number chans = 1, gaps = 1*/\
   {TLM_ARO,              4},\
   /*TLM chans +1, gap between chans + 1 ==> number chans = 2, gaps = 2*/\
   {TLM_SHELL_STATUS,     4},\
   {TLM_HW_STATUS,        4},\
   {TLM_TIMERS_A,         4},\
   {TLM_TIMERS_B,         2},\
   {TLM_HW_STATUS_EXT,    4},\
   /* TLM chans +5, gap between chans + 1 ==> number chans = 7, gaps = 3*/\
   {TLM_PROCTIME,         4},\
   {TLM_POS_X,            4},\
   {TLM_POS_Y,            4},\
   {TLM_POS_Z,            4},\
   {TLM_VEL_X,            4},\
   {TLM_VEL_Y,            4},\
   {TLM_VEL_Z,            4},\
   /* TLM chans +7, gap between chans + 1 ==> number chans = 14, gaps = 4*/\
   {TLM_EST_RP,           4},\
   {TLM_EST_WXY,          4},\
   {TLM_EST_YAW_WZ,       4},\
   {TLM_BACKGROUND_ATT,   4},\
   /* TLM chans +4, gap between chans + 1 ==> number chans = 18, gaps = 5*/\
   {TLM_GUIDE_YAW,        4},\
   {TLM_GUIDE_RP,         4},\
   /* TLM chans +2, gap between chans + 1 ==> number chans = 20, gaps = 6*/\
   {TLM_MAG_OCXY,         4},\
   {TLM_MAG_OCZ,          2},\
   /* TLM chans +2, gap between chans + 1 ==> number chans = 22, gaps = 7*/\
   {TLM_DIPOLEZ,          2},\
   {TLM_DIPOLE_XY,        4},\
   /* TLM chans +2, gap between chans + 1 ==> number chans = 24, gaps = 8*/\
   {TLM_SAS_AZEL,         4},\
   /* TLM chans +1, gap between chans + 1 ==> number chans = 25, gaps = 9*/\
   {TLM_ALG_STATUS,       4},\
   {TLM_FDIR_STATUS,      4},\
   {TLM_EXT_FDIR_STATUS,  4},\
   /* TLM chans +3, gap between chans + 1 ==> number chans = 28, gaps = 10*/\
   {TLM_MTM0,             4},\
   {TLM_MTM1,             4},\
   /* TLM chans +2, gap between chans + 1 ==> number chans = 30, gaps = 11*/\
   {TLM_WHL01_SPEED,      4},\
   {TLM_WHL23_SPEED,      4},\
   {TLM_WHL4_SPEED,       4},\
   /* TLM chans +3, gap between chans + 1 ==> number chans = 33, gaps = 12*/\
   {TLM_MTQX,             2},\
   {TLM_MTQY,             2},\
   {TLM_MTQZ,             2},\
   /* TLM chans +3, gap between chans + 1 ==> number chans = 36, gaps = 13*/\
   {TLM_GYR0_RATE_XY,     4},\
   {TLM_GYR0_RATE_ZT,     4},\
   /* TLM chans +2, gap between chans + 1 ==> number chans = 38, gaps = 14*/\
}

#define ADLK_MAX_NUM_CHANS 40

/* #define set such that with that number of channels including gaps we have 
less than 50 channels on first page */
#define ADLK_MAX_NUM_CHANS_PER_DNLK 36 

/*---------------------------------------------------------------------------
 * Typedefs (shared within this module)
 */

/*---------------------------------------------------------------------------
 * Data - (shared within this module)
 */
 
/*---------------------------------------------------------------------------
 * Module Function Prototypes
 */

#endif /* __ATTC_DNLK_DEFS_H_*/

