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
 * Last Update : $Date: 2013/04/23 15:49:21 $
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/include/HMGR_Globals.h,v $
 * Revision    : $Revision: 1.2 $
 *
 * History:
 *
 * $Log: HMGR_Globals.h,v $
 * Revision 1.2  2013/04/23 15:49:21  ytrichakis
 * Added block wait till SKED msgQ is ready to open
 *
 * Revision 1.1  2013/04/11 13:42:59  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/
#ifndef _HMGR_GLOBALS_H_
#define _HMGR_GLOBALS_H_

/*---------------------------------------------------------------------------
 * Includes
 */
#include "Adcs_mission.h"

/*---------------------------------------------------------------------------
 * Defines and Macros (shared within this module)
 */
#define WHEEL_CAN_NODES \
{\
   CANADDR_WHEEL0,\
   CANADDR_WHEEL1,\
   CANADDR_WHEEL2,\
   CANADDR_WHEEL3,\
}

#define SWHEEL_CAN_NODES \
{\
   CANADDR_SMALL_SAT_WHEEL,\
}


#define WHEEL_TLM_CAN_NODES {\
      CANADDR_WHEEL0_TLM,\
      CANADDR_WHEEL1_TLM,\
      CANADDR_WHEEL2_TLM,\
      CANADDR_WHEEL3_TLM,\
   }

#if _FC_NUM_SWHL != 0
//AJ temp!
#define SWHEEL_TLM_CAN_NODES {\
      CANADDR_GPS_TLM,\
   }


#endif

#define DPU_CAN_NODES \
{\
   CANADDR_DPU0,\
   CANADDR_DPU1,\
}

/*---------------------------------------------------------------------------
 * Typedefs (shared within this module)
 */
typedef struct
{
   tGDEF_UINT16 numChans;
   tGDEF_UINT16 index;
} tsHMGR_CAN_TLM;

typedef void (*tpfUTLM_HANDLER)(const tGDEF_UINT8 , tGDEF_UCHAR * );
/*---------------------------------------------------------------------------
 * Data - (shared within this module)
 */
extern const tGDEF_UINT8 HMGR_wheelNode[_FC_NUM_MWHL];
extern       tGDEF_UINT8 HMGR_opAimNode;
extern       tGDEF_UINT8 HMGR_mode;
extern       tGDEF_UINT8 HMGR_numAcksPending ;
extern       tGDEF_UINT8 HMGR_numNaks;

/* log parameters */
extern teGDEF_BOOLEAN HMGR_logMwhlFlag[_FC_NUM_MWHL];
extern teGDEF_BOOLEAN HMGR_logSasFlag[_FC_NUM_SAS];
extern teGDEF_BOOLEAN HMGR_logMtmFlag[_FC_NUM_MTM];

#if _FC_NUM_GPS != 0
	extern teGDEF_BOOLEAN HMGR_logGpsFlag[_FC_NUM_GPS] ;
#endif
#if _FC_NUM_SCAM != 0
	extern teGDEF_BOOLEAN HMGR_logScamFlag[_FC_NUM_SCAM] ;
#endif
#if _FC_NUM_FSS != 0
extern teGDEF_BOOLEAN HMGR_logFssFlag[_FC_NUM_FSS];
#endif
#if _FC_NUM_GYR != 0
extern teGDEF_BOOLEAN HMGR_logGyrFlag[_FC_NUM_GYR];
#endif
#if _FC_NUM_SWHL != 0
extern teGDEF_BOOLEAN HMGR_logSwhlFlag[_FC_NUM_SWHL];
#endif 

/*---------------------------------------------------------------------------
 * Module Function Prototypes
 */
/*tGDEF_UINT8 HMGR_GetNoOfACKs(void);
void HMGR_IncrementNoOfACKs(void);
void HMGR_DecrementNoOfACKs(void);*/

#endif
