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
 * Last Update : $Date: 2016/03/17 15:57:36 $
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/source/ACON_sked.c,v $
 * Revision    : $Revision: 1.12 $
 *
 * History:
 *
 * $Log: ACON_sked.c,v $
 * Revision 1.12  2016/03/17 15:57:36  ytrichakis
 * Create FSM thread before CAN and check for EEROR in dispatcher queue
 *
 * Revision 1.19  2015/10/21 08:20:44  ytrichakis
 * uccesful AOCS baseline that transitioned to CPM in July 2015
 *
 * Revision 1.8  2014/09/19 13:34:55  ytrichakis
 * Removed Filenumbers of SKEDs so that the appear as not registered if no SKED has been uploaded for each mode
 *
 * Revision 1.7  2013/09/03 13:35:21  ytrichakis
 * Removed calls to logging library as now all logs are done in the internel ADYYMMDD file
 *
 * Revision 1.6  2013/07/18 15:56:52  ytrichakis
 * fixed wheel bug(DR#14548) and applied the same solution in case of failed AIM
 *
 * Revision 1.5  2013/06/24 13:13:24  ytrichakis
 * Removed TODO's as agreed between YT and AJ (07/06/13)
 *
 * Revision 1.4  2013/05/24 13:03:27  ytrichakis
 * Fixed AIM0 problem and added TC return handling from SKED RTP
 *
 * Revision 1.3  2013/04/23 15:49:21  ytrichakis
 * Added block wait till SKED msgQ is ready to open
 *
 * Revision 1.2  2013/04/17 14:17:40  ytrichakis
 * Check in with fixing the identation only
 *
 * Revision 1.1  2013/04/11 13:42:21  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/

/*---------------------------------------------------------------------------
 * Includes
 */
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <mqueue.h>
#include <timers.h>

#include "GDEF_GlobDefs.h"
#include "mission.h"

#include "pfh.h"

#include "HMGR_Globals.h"
#include "AOCS_AocsShell.h"
#include "ACON_Sked.h"
#include "EWOD_EwodHandler.h"
#include "AFIH_Filehandler.h"
#include "CANS_Interface.h"
#include "CANS_API.h"
#include "CANA_ServerApiLib.h"
#include "spacecraft_log.h"
#include "SXAP_SkedExecAPI.h"

/*-----------------------------------------------------------------------
 * Defines and Macros
 */

#define ACON_NO_SKED_ID (0)
#define MAX_NO_OF_SKEDS (20)

/*---------------------------------------------------------------------------
 * Typedefs
 */
typedef struct
{
   FILE            *fp;
   tsAFIH_REG_ENTRY info;
} tsACON_SKED_PARAMS;

/*---------------------------------------------------------------------------
 * Local Data (declared as static; used only within this module)
 */
#if 0
static tGDEF_UINT8        *filename                = (tGDEF_UINT8*)("ACON_Sked.c");
static tGDEF_UINT8        *revision                = (tGDEF_UINT8*)("$Revision: 1.12 $ : " __DATE__ " @ " __TIME__);
#endif
static tGDEF_UINT32   active_fileNumber    = 0;
static teGDEF_BOOLEAN ACON_pollSkedFlag    = GDEF_FALSE;

/*---------------------------------------------------------------------------
 * Global Data
 */
tGDEF_UINT16 ACON_SKEDFilenumber[MAX_NO_OF_SKEDS] = {0};

/*---------------------------------------------------------------------------
 * External Function Prototypes
 */
//extern tsCANS_RegRtn *getCANReg(void);

/*---------------------------------------------------------------------------
 * Public Functions
 */


/*!
 *  \brief Rountine to determine the current ACON Configuration Sked status.
 *  \param  void           - None
 *  \return teGDEF_BOOLEAN - None
 */
teGDEF_BOOLEAN ACON_GetSkedStatus(void)
{
   /* RETURN */
   return ACON_pollSkedFlag;
}

/*!
 *  \brief Rountine to set the current ACON Configuration Sked status.
 *  \param  teGDEF_BOOLEAN - None
 *  \return void           - None
 */
void ACON_SetSkedStatus(teGDEF_BOOLEAN setFlag)
{
   ACON_pollSkedFlag = setFlag;
}

/*!
 * \brief  Start config sked
 *
 *         Starts the configuration sked
 *
 * \return Function status
 *
 ******************************************************************************/
teGDEF_FUNC_STATUS ACON_StartSked(tGDEF_UINT32 id)
{
   teGDEF_BOOLEAN            sked_started  = GDEF_FALSE;

   /* if id not valid return failure */
   if(id == ACON_NO_SKED_ID)
      return GDEF_FAILURE;

   /* Check if current sked not already open */
   if (ACON_GetSkedStatus() == GDEF_TRUE)
   {
      /* Config sked currently active so log error & return */
      EWOD_WriteMessage("ACON_StartConfigSked - Config sked currently active\n");
      return GDEF_FAILURE;
   }

   /*
    * Choose the right SKED to run
    */   
	active_fileNumber = ACON_SKEDFilenumber[id];

	sked_started = SXAP_ExecuteSkedFileNumber(active_fileNumber, SXAP_BIG_ENDIAN);
	if (sked_started == GDEF_FALSE)
	{
      //(void)lfprintf(RECORD_ERROR, "ACON_StartSked: error in starting SKED for mode: %d \n", id);
	}

   
   ACON_SetSkedStatus(GDEF_TRUE);
   /* RETURN */
   return GDEF_SUCCESS;
}

/*!
 *  \brief Rountine to abort the Configuration Sked.
 *  \param  void               - None
 *  \return teGDEF_FUNC_STATUS - Success / Failure Indication
 */
teGDEF_FUNC_STATUS ACON_AbortSked(void)
{

   EWOD_WriteMessage("Abort Config Sked\n");

   /* set the polling flag to false */
   ACON_pollSkedFlag = GDEF_FALSE;


   /* RETURN */
   return GDEF_SUCCESS;
}



#ifdef _CHECK_REQUIRED_
/*  \brief Rountine to supply the caller with the Configuration Sked File Number.
*  \param  void         - None
*  \return tGDEF_UINT32 - Configuration Sked File Number
*/
tGDEF_UINT32 ACON_GetSkedFnum(void)
{
   /** RETURN */
   return ACON_skedParams.info.fnumr;
}

/*!
 *  \brief Rountine to supply the caller with the Configuration Sked ID.
 *  \param  void         - None
 *  \return tGDEF_UINT32 - Configuration Sked ID
 */
tGDEF_UINT32 ACON_GetSkedId(void)
{
   /* RETURN */
   return ACON_skedParams.info.id;
}
#endif /* CHECK_REQUIRED*/

/*!
 *  \brief Routine to determine is a sked with this ID is registered
 *  \param  void           - None
 *  \return teGDEF_BOOLEAN - None
 */
teGDEF_BOOLEAN ACON_isSkedRegistered(tGDEF_UINT32 id)
{
   tGDEF_UINT32 fileNumber = 0;

   /* Find file number in the registry for the given id */
   if(AFIH_GetFnum(id, &fileNumber) != GDEF_SUCCESS)
   {
      return GDEF_FALSE;
   }

   return GDEF_TRUE;
}


/* End of File  */
