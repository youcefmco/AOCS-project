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
 * Last Update : $Date: 2013/06/24 13:13:25 $
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/source/RCON_Reconfigure.c,v $
 * Revision    : $Revision: 1.4 $
 *
 * History:
 *
 * $Log: RCON_Reconfigure.c,v $
 * Revision 1.4  2013/06/24 13:13:25  ytrichakis
 * Removed TODO's as agreed between YT and AJ (07/06/13)
 *
 * Revision 1.3  2013/06/04 10:09:03  ytrichakis
 * AOCS Delivery 4th June for flight rehearsal
 *
 * Revision 1.2  2013/04/17 14:17:41  ytrichakis
 * Check in with fixing the identation only
 *
 * Revision 1.1  2013/04/11 13:42:22  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "GDEF_GlobDefs.h"
#include "Adcs_mission.h"

#include "AINT_AdcsInterface.h"
#include "ACON_AocsConfig.h"
#include "RCON_Reconfigure.h"
#include "AMOH_AocsModeHandler.h"
#include "HMGR_HardwareManager.h"
#include "td_adcs.h"


/*!
 * \brief     Reconfigure
 *
 *              reconfigures the AOCS units depending on the state and mode
 *              this is in accordance with the satellite's FDIR strategy
 *
 * \param mode mode number
 * \return new mode number
 *
 ******************************************************************************/
tGDEF_UINT8 RCON_Reconfigure(tGDEF_UINT8 mode)
{
   tGDEF_UINT8 newMode;

   /* 
    * set reconfig state 
    */
   AROH_reconfigState = RCON_RECONFIG1;

   /*
    *  set safe mode to standby 
    */
   newMode = MODE_SBM;

   /*  
    * kick off a safe mode transition
    */
   AMOH_SafeModeTransition(newMode);

   /* 
    * return new mode
    */
   return newMode;
}




/*!
 * \brief     AROH_ResetReconfigState
 *
 *              resets the reconfig state
 *
 ******************************************************************************/
void RCON_SetState(tGDEF_UINT8 value)
{

   if(AROH_reconfigState > RCON_MAX_RECONFIG_STATE)
   {
      /* log invalid reconfig state */
      /*strcpy(EWOD_sLogMsg, "RCON_SetState -Error: Invalid reconfig state [" );
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d",value);
      strcat(EWOD_sLogMsg, "]\n" );
      EWOD_WriteMessage(EWOD_sLogMsg);*/
      return;
   }

   /* set the reconfig state */
   AROH_reconfigState = value;

   /* Log Update */
   /*strcpy(EWOD_sLogMsg, "RCON_SetState -Updated reconfig state to  [" );
   itoa(value, &EWOD_sLogMsg[strlen(EWOD_sLogMsg)], 10);
   strcat(EWOD_sLogMsg, "]\n" );
   EWOD_WriteMessage(EWOD_sLogMsg);*/
}


/*!
 * \brief     AROH_GetReconfigState
 *
 *              Gets the current reconfiguration state
 *
 * \return reconfig state value
 *
 ******************************************************************************/
tGDEF_UINT8 RCON_GetState()
{
   return AROH_reconfigState;
}



/*!
 * \brief     AROH_MathError
 *
 *              Sets correct reconfiguration for a math error
 *
 *
 ******************************************************************************/
void RCON_MathError()
{
   //set the state to reconfig state 2
   AROH_reconfigState = RCON_RECONFIG2;

   //trigger a safe mode transition to standby
   AMOH_SafeModeTransition(MODE_SBM);
}




