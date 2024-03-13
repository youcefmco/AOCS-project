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
 * Last Update : $Date: 2016/03/14 10:47:46 $
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/source/AROH_AroHandler.c,v $
 * Revision    : $Revision: 1.7 $
 *
 * History:
 *
 * $Log: AROH_AroHandler.c,v $
 * Revision 1.7  2016/03/14 10:47:46  ytrichakis
 * Register with the SKED library
 *
 * Revision 1.4  2013/07/18 15:56:53  ytrichakis
 * fixed wheel bug(DR#14548) and applied the same solution in case of failed AIM
 *
 * Revision 1.3  2013/06/24 13:13:25  ytrichakis
 * Removed TODO's as agreed between YT and AJ (07/06/13)
 *
 * Revision 1.2  2013/04/17 14:17:40  ytrichakis
 * Check in with fixing the identation only
 *
 * Revision 1.1  2013/04/11 13:42:21  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/
#include <time.h>
#include <stdio.h>

#include "GDEF_GlobDefs.h"

#include "Adcs_mission.h"

#include "RCON_Reconfigure.h"
#include "ACON_AocsConfig.h"
#include "AROH_AroHandler.h"
#include "HMGR_HardwareManager.h"
//#include "EWOD_EwodHandler_OBC750.h"
#include "EWOD_EwodHandler.h"
#include "AMOH_AocsModeHandler.h"


/*-----------------------------------------------------------------------
 * Defines and Macros
 */
#define ARO_DEFAULT_P  (1)
#define ARO_DEFAULT_Q  (2)
#define ARO_INIT_VALUE (100)

/*---------------------------------------------------------------------------
 * Typedefs
 */

/*---------------------------------------------------------------------------
 * Private Function Prototypes (declared as static; used only within this module)
 */
static void         AROH_ClearDecFlag(void);

/*---------------------------------------------------------------------------
 * Local Data (declared as static; used only within this module)
 */
static teGDEF_BOOLEAN AROH_DecFlag                  = GDEF_FALSE;
static tGDEF_INT16    AROH_counter                  = ARO_INIT_VALUE;
static tGDEF_UINT16   AROH_counterP                 = ARO_DEFAULT_P;
static tGDEF_UINT16   AROH_counterQ                 = ARO_DEFAULT_Q;

static tGDEF_UINT16   AROH_counterMax[NUM_MODES]    = ARO_DEFAULT_MAX_VALS;
static time_t         AROH_holdOffPeriod[NUM_MODES] = ARO_HOLDOFF_INTERVALS;
static time_t         AROH_HoldOffTime              = 0;

static tGDEF_UINT32   AROH_newErrors                = 0x0UL;

/*---------------------------------------------------------------------------
 * Global Data
 */
teGDEF_BOOLEAN AROH_isReconfigEnabled = GDEF_TRUE;
tGDEF_UINT8    AROH_reconfigState     = 0;
tGDEF_UINT32   AROH_errors            = 0x0UL;


/*!
 * \brief     AROH_SetDecFlag
 *
 *              Sets a flag to indicate ARO is to decrement
 *
 *
 ******************************************************************************/
void AROH_SetDecFlag(tGDEF_UINT8 errNo)
{
   AROH_DecFlag   = GDEF_TRUE;
   AROH_newErrors |= 0x01UL << errNo;
}

/*!
 * \brief     AROH_ClearDecFlag
 *
 *              Clears the decrement flag
 *
 ******************************************************************************/
void AROH_ClearDecFlag()
{
   AROH_DecFlag = GDEF_FALSE;

   /* store error codes */
   AROH_errors    = AROH_newErrors;
   AROH_newErrors = 0x0UL;
}

/*!
 * \brief     AROH_Update
 *
 *              Updates the ARO counter and takes action on it reaching 0
 *
 *
 ******************************************************************************/
void AROH_Update(void)
{
   time_t      tNow = 0;
   tGDEF_UINT8 mode = 0;
   /* get current time */
   tNow = time(NULL);

   /* check if ARO hold off is active */
   if (tNow < AROH_HoldOffTime)
   {
      AROH_ClearDecFlag();
      return;
   }

   /* get current mode */
   mode = AMOH_GetMode();

   /* no action in standby mode */
   if(mode == MODE_SBM )
   {
      /* ensure dec flag is cleared */
      AROH_ClearDecFlag();
      return;
   }

   // Do we need to decrement the ARO?
   if (GDEF_TRUE == AROH_DecFlag)
   {
      if (AROH_counter > 0)
      {
         AROH_counter -= AROH_counterQ;
      }
   }
   else
   {
      /* Increment the ARO */
      AROH_counter += AROH_counterP;

      /* Limit the ARO counter */
      if (AROH_counter > (tGDEF_INT16) AROH_counterMax[mode])
      {
         AROH_counter = AROH_counterMax[mode];
      }
   }

   /* Reset the decrement flag */
   AROH_ClearDecFlag();

   /* reconfigure if  the ARO counter has hit the floor */
   if (AROH_counter <= 0)
   {
      /* ensure the min value is 0 */
      AROH_counter = 0;

      if (AROH_isReconfigEnabled)
      {
         EWOD_WriteMessage("ARO Counter < 0, Reconfigure Mode \n");
         mode = RCON_Reconfigure(mode);

         /* reset the counter */
         AROH_counter = AROH_counterMax[mode];
      }
   }

}

/*!
 * \brief     AROH_SetHoldoff
 *
 *              Updates  the ARO hold off time value
 *
 * \param mode mode number
 *
 ******************************************************************************/
void AROH_SetHoldOff(tGDEF_UINT8 mode)
{
   time_t tNow = 0;

   /* get currrent time */
   tNow = time(NULL);

   /* set new ARO hold off time */
   AROH_HoldOffTime = tNow + AROH_holdOffPeriod[mode];
}


/*!
 * \brief     AROH_SetHoldoffVal
 *
 *              Sets the ARO hold off value following a reconfiguration for a given mode
 *
 * \param mode mode number
 * \param period hold off period
 *
 ******************************************************************************/
void AROH_SetHoldoffVal(tGDEF_UINT8 mode, tGDEF_UINT32 period)
{
   AROH_holdOffPeriod[mode] = period;
}


/*!
 * \brief     AROH_GetHoldoffVal
 *
 *              Sets Gets the current ARO holdoff time
 *
 * \return holdoff time
 *
 ******************************************************************************/
tGDEF_UINT32 AROH_GetHoldoffVal(void)
{
   tGDEF_UINT8 mode = 0;

   mode = AMOH_GetMode();

   return AROH_holdOffPeriod[mode];
}



/*!
 * \brief     AROH_SetCounterP
 *
 *              Sets the current ARO P value for a given mode
 *
 * \param mode mode number
 * \param value of ARO P value
 * \return function status
 *
 ******************************************************************************/
teGDEF_FUNC_STATUS AROH_SetCounterP(tGDEF_UINT16 value)
{
   /* set counter */
   AROH_counterP = value;

   /* return success */
   return GDEF_SUCCESS;
}


/*!
 * \brief     AROH_SetCounterQ
 *
 *              Sets the current ARO Q value for a given mode
 *
 * \param mode mode number
 * \param value of ARO Q value
 * \return function status
 *
 ******************************************************************************/
teGDEF_FUNC_STATUS AROH_SetCounterQ(tGDEF_UINT16   value)
{
   /* set counter */
   AROH_counterQ = value;

   /* return success */
   return GDEF_SUCCESS;
}



/*!
 * \brief     AROH_SetCounterMax
 *
 *              Sets the current ARO max value for a given mode
 *
 * \param mode mode number
 * \param value of max ARO value
 * \return function status
 *
 ******************************************************************************/
teGDEF_FUNC_STATUS AROH_SetCounterMax(tGDEF_UINT8 mode, tGDEF_UINT16 value)
{
   /* set max value */
   AROH_counterMax[mode] = value;

   /* return success */
   return GDEF_SUCCESS;
}



/*!
 * \brief     AROH_GetStatus
 *
 *              Gets the current ARO, P, Q &  max values
 *
 * \return max value
 ******************************************************************************/
tGDEF_UINT32 AROH_GetStatus(void)
{
   tGDEF_UINT8  mode   = 0;
   tGDEF_UINT32 status = 0UL;

   mode = AMOH_GetMode();


   status = ((tGDEF_UINT32) (AROH_counter          &0x0FFF)      )   |
            ((tGDEF_UINT32) (AROH_counterMax[mode] &0x0FFF) << 12)   |
            ((tGDEF_UINT32) (AROH_counterP         &0x000F) << 24)   |
            ((tGDEF_UINT32) (AROH_counterQ         &0x000F) << 28);

   return status;
}


/*!
 * \brief     AROH_Reset
 *
 *              Resets the ARO counter for the current mode
 *
 *
 ******************************************************************************/
void AROH_Reset(void)
{
   tGDEF_UINT8 mode = 0;

   mode = AMOH_GetMode();

   AROH_counter = AROH_counterMax[mode];
}


/*!
 * \brief     AROH_Enable
 *
 *              Enables/disables reconfiguration on ARO counter reaching 0
 *
 * \param isEnabled boolean flag
 *
 ******************************************************************************/
void AROH_Enable(teGDEF_BOOLEAN isEnabled)
{
   AROH_isReconfigEnabled = (teGDEF_BOOLEAN) isEnabled;
}


/*!
 * \brief     AROH_MathError
 *
 *              Sets correct reconfiguration for a math error
 *
 *
 ******************************************************************************/
void AROH_MathError(void)
{
   RCON_MathError();

}


