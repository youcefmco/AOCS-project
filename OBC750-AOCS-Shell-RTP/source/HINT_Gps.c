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
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/source/HINT_Gps.c,v $
 * Revision    : $Revision: 1.5 $
 *
 * History:
 *
 * $Log: HINT_Gps.c,v $
 * Revision 1.5  2016/03/14 10:47:46  ytrichakis
 * Register with the SKED library
 *
 * Revision 1.2  2013/04/17 14:17:41  ytrichakis
 * Check in with fixing the identation only
 *
 * Revision 1.1  2013/04/11 13:42:21  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

//#include "EWOD_Ewodhandler_OBC750.h"
#include "EWOD_EwodHandler.h"
#include "GDEF_GlobDefs.h"

#include "Adcs_mission.h"

#include "HINT_Gps.h"

/*-----------------------------------------------------------------------
 * Defines and Macros
 */
// GPS data mask.
// These bits are used to record if all GPS data for a
// complete data set has been received.
#define  GPSFLAG_POSX      (0x001)
#define  GPSFLAG_POSY      (0x002)
#define  GPSFLAG_POSZ      (0x004)
#define  GPSFLAG_VELX      (0x008)
#define  GPSFLAG_VELY      (0x010)
#define  GPSFLAG_VELZ      (0x020)
#define  GPSFLAG_WEEK      (0x040)
#define  GPSFLAG_SECS      (0x080)
#define  GPSFLAG_NANOSECS  (0x100)
#define  GPSFLAG_STATUS    (0x200)
#define  GPSFLAG_MODE      (0x400)

#define GPS_DATA_COMPLETE  (0x3ff)

#define  GPSTLM_IN_POSX       (16)
#define  GPSTLM_IN_POSY       (17)
#define  GPSTLM_IN_POSZ       (18)
#define  GPSTLM_IN_VELX       (19)
#define  GPSTLM_IN_VELY       (20)
#define  GPSTLM_IN_VELZ       (21)
#define  GPSTLM_IN_WEEK       (22)
#define  GPSTLM_IN_SECS       (23)
#define  GPSTLM_IN_NANOSECS   (24)
#define  GPSTLM_IN_STATUS_GPS (27)

#define GPS_STATUS_3DFIX      (2)



/*---------------------------------------------------------------------------
 * Typedefs
 */

/*---------------------------------------------------------------------------
 * Private Function Prototypes (declared as static; used only within this module)
 */


/*---------------------------------------------------------------------------
 * Local Data (declared as static; used only within this module)
 */
static tGDEF_UINT16      UTLM_gpsExpFrame   = GPS_DATA_COMPLETE;
static tGDEF_UINT16      UTLM_gpsRecvFrame  = 0;
static tsGPS_STATE       UTLM_gpsTlm        = {{0},{0},0,0,0,0,0,0};
static const tsGPS_STATE UTLM_invalidGpsTlm = AINT_INVALID_GPS_STATE;
static tGDEF_UINT32      UTLM_gpsStatus     = 0;

/*---------------------------------------------------------------------------
 * Global Data
 */

/*!
 * \brief     UTLM_ProcGpsTlm
 *
 *              Proceses an incoming unsolicted GPS telemetry frame
 * \param chan tlm channel number
 * \param *pData pointer to tlm data
 * \param isnewFrame boolean to flag if the data is for a new frame
 *
 ******************************************************************************/
void UTLM_ProcGpsTlm(tGDEF_UINT16 chan, tGDEF_UCHAR * pData, teGDEF_BOOLEAN isNewFrame)
{
   // clear recv flag if sent a new frame
   if (isNewFrame)
   {
      UTLM_gpsRecvFrame = 0;
   }

   /* *** Simplified checks *** */
   switch (chan)
   {
   case GPSTLM_IN_POSX:
      UTLM_gpsTlm.pos[GDEF_X_AXIS] = *(tGDEF_INT32 *)pData;
      UTLM_gpsRecvFrame           |= GPSFLAG_POSX;
      break;
   case GPSTLM_IN_POSY:
      UTLM_gpsTlm.pos[GDEF_Y_AXIS] = *(tGDEF_INT32 *)pData;
      UTLM_gpsRecvFrame           |= GPSFLAG_POSY;
      break;
   case GPSTLM_IN_POSZ:
      UTLM_gpsTlm.pos[GDEF_Z_AXIS] = *(tGDEF_INT32 *)pData;
      UTLM_gpsRecvFrame           |= GPSFLAG_POSZ;
      break;
   case GPSTLM_IN_VELX:
      UTLM_gpsTlm.vel[GDEF_X_AXIS] = *(tGDEF_INT32 *)pData;
      UTLM_gpsRecvFrame           |= GPSFLAG_VELX;
      break;
   case GPSTLM_IN_VELY:
      UTLM_gpsTlm.vel[GDEF_Y_AXIS] = *(tGDEF_INT32 *)pData;
      UTLM_gpsRecvFrame           |= GPSFLAG_VELY;
      break;
   case GPSTLM_IN_VELZ:
      UTLM_gpsTlm.vel[GDEF_Z_AXIS] = *(tGDEF_INT32 *)pData;
      UTLM_gpsRecvFrame           |= GPSFLAG_VELZ;
      break;
   case GPSTLM_IN_WEEK:
      UTLM_gpsTlm.week   = *(tGDEF_INT32 *) pData;
      UTLM_gpsRecvFrame |= GPSFLAG_WEEK;
      break;
   case GPSTLM_IN_SECS:
      UTLM_gpsTlm.second = *(tGDEF_INT32 *) pData;
      UTLM_gpsRecvFrame |= GPSFLAG_SECS;
      break;
   case GPSTLM_IN_NANOSECS:
      UTLM_gpsTlm.fraction = (*(tGDEF_INT32 *) pData);
      UTLM_gpsRecvFrame   |= GPSFLAG_NANOSECS;
      break;
   case GPSTLM_IN_STATUS_GPS:
   {
      /* store gps Status data */
      UTLM_gpsStatus = *(tGDEF_INT32 *) pData;

      /* check if have a 3-d fix (byte 3) */
      if (((tGDEF_UCHAR) (UTLM_gpsStatus >> 16 & 0xFFUL)) == GPS_STATUS_3DFIX)
      {
         UTLM_gpsTlm.status = GDEF_TRUE;
      }
      else
      {
         UTLM_gpsTlm.status = GDEF_FALSE;
      }

      UTLM_gpsRecvFrame |= GPSFLAG_STATUS;
   }
   break;
   default:
      break;
   }
}



/*!
 * \brief     SetGpsData
 *
 *              Sets the GPS data & validity based on the received tlm
 *
 ******************************************************************************/
teGDEF_FUNC_STATUS UTLM_SetGpsData(tsGPS_STATE *pTtc, tGDEF_UCHAR isEnabled[])
{
   tGDEF_UINT8    gpsNo     = 0;
   tGDEF_UINT8    gpsState  = 0;
   teGDEF_BOOLEAN errorFlag = GDEF_FALSE;

   /* check if all gps data received */
   if (UTLM_gpsRecvFrame == UTLM_gpsExpFrame)
   {
      /* reset received frame */
      UTLM_gpsRecvFrame = 0UL;

      if (UTLM_gpsTlm.status == GDEF_TRUE)
      {
         /* 3d fix has been checked in ProcGpsData */
         UTLM_gpsTlm.valid = GDEF_TRUE;
      }
      else
      {
         UTLM_gpsTlm.valid = GDEF_FALSE;
      }
   }
   else
   {
      UTLM_gpsTlm.valid = GDEF_FALSE;
   }

   /*check if GPS was enabled or not*/
   gpsState = GDEF_FALSE;
   for(gpsNo = 0; gpsNo <= _FC_NUM_GPS; gpsNo++)
   {
      if( GDEF_TRUE == isEnabled[gpsNo])
      {
         gpsState= GDEF_TRUE; // at least one of the GPS flags is set to high
      }
   }

   if (GDEF_FALSE == gpsState)
   {
      UTLM_gpsTlm.valid = GDEF_FALSE;
   }

   /* copy rest of data if valid */
   if (GDEF_TRUE == UTLM_gpsTlm.valid)
   {
      /* copy data */
      *pTtc = UTLM_gpsTlm;
   }
   else
   {
      /* copy invalid data state */
      *pTtc = UTLM_invalidGpsTlm;
      /* flag a sampling error */
      errorFlag = GDEF_TRUE;
   }

   if(errorFlag)
   {
      return GDEF_FAILURE;
   }
   else
   {
      return GDEF_SUCCESS;
   }

}

void UTLM_GpsInitDataGathering(void)
{
   memset(&UTLM_gpsTlm, 0, sizeof(UTLM_gpsTlm));
   memset(&UTLM_gpsRecvFrame, 0 , sizeof(UTLM_gpsRecvFrame));
}
