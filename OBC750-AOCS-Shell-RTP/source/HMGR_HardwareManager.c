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
 * Last Update : $Date: 2013/04/23 15:49:22 $
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/source/HMGR_HardwareManager.c,v $
 * Revision    : $Revision: 1.3 $
 *
 * History:
 *
 * $Log: HMGR_HardwareManager.c,v $
 * Revision 1.3  2013/04/23 15:49:22  ytrichakis
 * Added block wait till SKED msgQ is ready to open
 *
 * Revision 1.2  2013/04/17 14:17:41  ytrichakis
 * Check in with fixing the identation only
 *
 * Revision 1.1  2013/04/11 13:42:22  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <limits.h>

#include "GDEF_GlobDefs.h"
#include "Adcs_mission.h"
#include "mission.h"

#include "AMOH_AocsModeHandler.h"
#include "AINT_AdcsInterface.h"
#include "ATTC_AocsTTC.h"
#include "HMGR_HardwareManager.h"

/*-----------------------------------------------------------------------
 * Defines and Macros
 */

/*---------------------------------------------------------------------------
 * Typedefs
 */

/*---------------------------------------------------------------------------
 * Private Function Prototypes (declared as static; used only within this module)
 */


/*---------------------------------------------------------------------------
 * Local Data (declared as static; used only within this module)
 */
#if _FC_NUM_SWHL != 0
static const tGDEF_UINT8 HMGR_swheelNode[_FC_NUM_SWHL]   = SWHEEL_CAN_NODES;
#endif

/*---------------------------------------------------------------------------
 * Global Data
 */
tGDEF_UINT8       HMGR_mode                       = MODE_SBM;
const tGDEF_UINT8 HMGR_wheelNode[_FC_NUM_MWHL]    = WHEEL_CAN_NODES;
tGDEF_UINT8       HMGR_numAcksPending             = 0;
tGDEF_UINT8       HMGR_numNaks                    = 0;
/* log parameters */
teGDEF_BOOLEAN    HMGR_logMwhlFlag[_FC_NUM_MWHL]  = {0};
teGDEF_BOOLEAN    HMGR_logSasFlag[_FC_NUM_SAS]    = {0};
teGDEF_BOOLEAN    HMGR_logMtmFlag[_FC_NUM_MTM]    = {0};
#if _FC_NUM_GPS != 0
teGDEF_BOOLEAN HMGR_logGpsFlag[_FC_NUM_GPS]   = {0};
#endif
#if _FC_NUM_SCAM != 0
teGDEF_BOOLEAN HMGR_logScamFlag[_FC_NUM_SCAM] = {0};
#endif
#if _FC_NUM_FSS != 0
teGDEF_BOOLEAN HMGR_logFssFlag[_FC_NUM_FSS]   = {0};
#endif
#if _FC_NUM_GYR != 0
teGDEF_BOOLEAN HMGR_logGyrFlag[_FC_NUM_GYR]       = {0};
#endif
#if _FC_NUM_SWHL != 0
teGDEF_BOOLEAN HMGR_logSwhlFlag[_FC_NUM_SWHL]     = {0};
#endif

#if _FC_NUM_SWHL != 0

const tGDEF_UINT16 HMGR_swheelUnsolNode[_FC_NUM_SWHL] = SWHEEL_TLM_CAN_NODES;
#endif


/*
tGDEF_UINT8 HMGR_GetNoOfACKs(void)
{
	return HMGR_numAcksPending;
}

void HMGR_IncrementNoOfACKs(void)
{
	HMGR_numAcksPending++;
}

void HMGR_DecrementNoOfACKs(void)
{
	HMGR_numAcksPending--;
}
*/
/*!
 * \brief      <function brief description>
 *
 *             <function detailed description>
 *
 * \param  <param name>  <param description>
 * \return <return value description>
 *
 ******************************************************************************/
void HMGR_InitMode(tGDEF_UINT8 mode)
{
   /* updatet the mode in hw */
   HMGR_mode  = mode;
}


tGDEF_UINT32 HMGR_GetStatus(tGDEF_UINT8 statusType)
{
   tGDEF_UINT32 status = 0UL;
   tGDEF_UINT8  unitNo = 0;
   tGDEF_UINT8  aimNo  = 0;
   tGDEF_UINT8  bitNo  = 0;

   tsALG_ENABLE_SENSOR   *pSnsEnableFlag = AINT_GetSensorEnableFlags();
   tsALG_ENABLE_ACTUATOR *pActEnableFlag = AINT_GetActuatorEnableFlags();

   switch (statusType)
   {
   case HMGR_HERITAGE_STATUS:
      /*logging */
      for (unitNo = 0; unitNo< _FC_NUM_MTM; unitNo++)
      {
         status |= (0x01UL & HMGR_logMtmFlag[unitNo]) << bitNo;
         bitNo++;
      }

      for (unitNo = 0; unitNo< _FC_NUM_SAS; unitNo++)
      {
         status |= (0x01UL & HMGR_logSasFlag[unitNo]) << bitNo;
         bitNo++;
      }

      for (unitNo = 0; unitNo< _FC_NUM_MWHL; unitNo++)
      {
         status |= (0x01UL & HMGR_logMwhlFlag[unitNo]) << bitNo;
         bitNo++;
      }

      /*sns enable*/

      for (unitNo = 0; unitNo< _FC_NUM_MTM; unitNo++)
      {
         status |= (0x01UL & pSnsEnableFlag->mtm[unitNo]) << bitNo;
         bitNo++;
      }
      for (unitNo = 0; unitNo< _FC_NUM_SAS; unitNo++)
      {
         status |= (0x01UL & pSnsEnableFlag->sas[unitNo]) << bitNo;
         bitNo++;
      }
      for (unitNo = 0; unitNo< _FC_NUM_MWHL; unitNo++)
      {
         status |= (0x01UL & pSnsEnableFlag->whl[unitNo]) << bitNo;
         bitNo++;
      }

      /*actuators*/
      for (aimNo = 0; aimNo < _FC_NUM_AIM; aimNo++)
      {
         status |= (0x01UL & pActEnableFlag->mtq[aimNo]) << bitNo;
         bitNo++;
      }
      for (unitNo = 0; unitNo< _FC_NUM_MWHL; unitNo++)
      {
         status |= (0x01UL & pActEnableFlag->whl[unitNo]) << bitNo;
         bitNo++;
      }


      break;


   case HMGR_ADD_UNIT_STATUS:

#if _FC_NUM_SWHL != 0
      for (unitNo = 0; unitNo< _FC_NUM_SWHL; unitNo++)
      {
         status |= (0x01UL & HMGR_logSwhlFlag[unitNo]) << bitNo;
         bitNo++;
      }
      for (unitNo = 0; unitNo< _FC_NUM_SWHL; unitNo++)
      {
         status |= (0x01UL & pSnsEnableFlag->swhl[unitNo]) << bitNo;
         bitNo++;
      }
#endif

#if _FC_NUM_FSS != 0
      for (unitNo = 0; unitNo< _FC_NUM_FSS; unitNo++)
      {
         status |= (0x01UL & HMGR_logFssFlag[unitNo]) << bitNo;
         bitNo++;
      }
      for (unitNo = 0; unitNo< _FC_NUM_FSS; unitNo++)
      {
         status |= (0x01UL & pSnsEnableFlag->fss[unitNo]) << bitNo;
         bitNo++;
      }
#endif


#if _FC_NUM_SCAM != 0
      for (unitNo = 0; unitNo< _FC_NUM_SCAM; unitNo++)
      {
         status |= (0x01UL & HMGR_logScamFlag[unitNo]) << bitNo;
         bitNo++;
      }
      for (unitNo = 0; unitNo< _FC_NUM_SCAM; unitNo++)
      {
         status |= (0x01UL & pSnsEnableFlag->scam[unitNo]) << bitNo;
         bitNo++;
      }
#endif

#if _FC_NUM_GPS != 0
      for (unitNo = 0; unitNo< _FC_NUM_GPS; unitNo++)
      {
         status |= (0x01UL & HMGR_logGpsFlag[unitNo]) << bitNo;
         bitNo++;
      }
      for (unitNo = 0; unitNo< _FC_NUM_GPS; unitNo++)
      {
         status |= (0x01UL & pSnsEnableFlag->gps[unitNo]) << bitNo;
         bitNo++;
      }
#endif

#if _FC_NUM_DPU != 0
      for (unitNo = 0; unitNo< _FC_NUM_DPU; unitNo++)
      {
         status |= (0x01UL & pSnsEnableFlag->dpu[unitNo]) << bitNo;
         bitNo++;
      }
#endif

#if _FC_NUM_SWHL != 0
      bitNo = 20;

      for (unitNo = 0; unitNo< _FC_NUM_SWHL; unitNo++)
      {
         status |= (0x01UL & pActEnableFlag->swhl[unitNo]) << bitNo;
         bitNo++;
      }
#endif


#if _FC_NUM_GYR != 0
      for (unitNo = 0; unitNo< _FC_NUM_GYR; unitNo++)
      {
         status |= (0x01UL & HMGR_logGyrFlag[unitNo]) << bitNo;
         bitNo++;
      }
      for (unitNo = 0; unitNo< _FC_NUM_GYR; unitNo++)
      {
         status |= (0x01UL & pSnsEnableFlag->gyr[unitNo]) << bitNo;
         bitNo++;
      }
#endif

      break;
   }

   return status;
}





