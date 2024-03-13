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
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/source/AINT_AdcsInterface.c,v $
 * Revision    : $Revision: 1.13 $
 *
 * History:
 *
 * $Log: AINT_AdcsInterface.c,v $
 * Revision 1.13  2016/03/17 15:57:36  ytrichakis
 * Create FSM thread before CAN and check for EEROR in dispatcher queue
 *
 * Revision 1.9  2014/09/15 12:56:50  ytrichakis
 * Changes for TDS-1 Flight to include accepting drive file from automation process and removing sending sync cmds to AIM when in SBM
 *
 * Revision 1.8  2014/09/15 12:29:25  ytrichakis
 * Update for TDS-1 flight to inlcude accepting drive file from automation process
 *
 * Revision 1.7  2013/09/06 13:40:22  ytrichakis
 * removed unguarded printfs
 *
 * Revision 1.6  2013/09/02 09:20:34  ytrichakis
 * AOCS Version before completion review 4/9/13
 *
 * Revision 1.5  2013/07/18 15:56:53  ytrichakis
 * fixed wheel bug(DR#14548) and applied the same solution in case of failed AIM
 *
 * Revision 1.4  2013/06/24 13:13:25  ytrichakis
 * Removed TODO's as agreed between YT and AJ (07/06/13)
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
//#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>

#include "GDEF_GlobDefs.h"
#include "Adcs_mission.h"
#include "Adcs_IntDefs.h"

#include "Adcs_ttcDefs.h"

//#include "lunisol.h"
#include "adcslib.h"
#include "td_adcs.h"

#include "pfh.h"
#include "file_number.h"

#include "AMOH_AocsModeHandler.h"
#include "AOCS_aocsShell.h"
#include "AROH_AroHandler.h"
//#include "EWOD_Ewodhandler_OBC750.h"
#include "EWOD_EwodHandler.h"
#include "AFIH_FileHandler.h"
#include "AINT_AdcsInterface.h"
#include "AINT_Tlm.h"
#include "HMGR_HardwareManager.h"


/*-----------------------------------------------------------------------
 * Defines and Macros
 */
#define NUM_ALG_TLM_CHANS (10)

#define AOCS_Telemetry(X) _fc_AOCS_Telemetry(X)
#define AOCS_Go(X, Y)  _fc_AOCS_Go(X, (char) Y)

#define AINT_TEST_STATE_ON (0x65AB)
#define SATID 			   (2)

/*---------------------------------------------------------------------------
 * Typedefs
 */

/*---------------------------------------------------------------------------
 * Private Function Prototypes (declared as static; used only within this module)
 */
static teGDEF_FUNC_STATUS ReadDriveHdr(FILE *fp, tGDEF_UINT16 * pDataLen);
static tGDEF_UINT32       GetFileLenToEnd(FILE *fp);
static void               AINT_PackTlm();
static ts_UNIX_TIME       AINT_GetUnixTime(void);
static void               AlgDelay(tGDEF_UINT16 delay);

/*---------------------------------------------------------------------------
 * Local Data (declared as static; used only within this module)
 */

static tGDEF_INT16 AINT_ticksDelay[NUM_MODES] = AOCS_ALG_DELAY;
static teGDEF_BOOLEAN AINT_isModeReqAllowed   = GDEF_FALSE;
static tGDEF_UINT8 AINT_LogMode               = MODE_SBM; /* for solicited telemetry requests on enable flags for certain mode */
static teGDEF_BOOLEAN AINT_LogSafeMode        = GDEF_FALSE;

/* packed data channels */
static tGDEF_UINT32 AINT_packedTlm[NUM_PACKED_TYPES]       = {0};

static const tsAINT_TLM_DATA AINT_algTlmLookup[]           = AINT_ALG_TLM_LOOKUP;
static const tsAINT_TLM_DATA AINT_algCmdTlmLookup[]        = AINT_ALG_CMD_LOOKUP;
static const tsAINT_TLM_DATA AINT_sensorTlmLookup[]        = AINT_SENSOR_TLM_LOOKUP;
static const tsAINT_FDIR_CHECK_DATA AINT_FdirCheckLookup[] = AINT_FDIR_ENABLE_TABLE;

static const tGDEF_UINT16 AINT_numAlgTlmChans    = sizeof(AINT_algTlmLookup)    / sizeof(tsAINT_TLM_DATA);
static const tGDEF_UINT16 AINT_numAlgCmdTlmChans = sizeof(AINT_algCmdTlmLookup) / sizeof(tsAINT_TLM_DATA);
static const tGDEF_UINT16 AINT_numSensorTlmChans = sizeof(AINT_sensorTlmLookup) / sizeof(tsAINT_TLM_DATA);
static const tGDEF_UINT16 AINT_numFDIRChecks     = sizeof(AINT_FdirCheckLookup) / sizeof(tsAINT_FDIR_CHECK_DATA);

/*---------------------------------------------------------------------------
 * Global Data
 */

/* The AOCS structure used to pass data between the shell and the AOCS algorithms*/
tsAOCS_TTC            ttcState                   = {{{0},0},{0},{0},{{{0},{0},{0}}},{{{0},{0}}},{{{0},{0},{0}}},{{{0},{0}}},{{{{0}},{0},{0}}},{{0}},{{0}},{{0},{0},{{0}},{0}},0,0,0,{0},0};

tsAOCS_TTC            * const AINT_pTtc          = &ttcState;
tsALG_CONFIG          * const AINT_pAlgConfig    = &(ttcState.algConfig);
tsALG_ATTITUDE_DEMAND * const AINT_pAttitude     = &(ttcState.attitude);
tsALG_SENSOR_TLM      * const AINT_pSensorTlm    = &(ttcState.tlm);

tsALG_TCMD            * const AINT_pAlgCmd       = &(ttcState.algCmd);
tsALG_TLM             * const AINT_pAlgTlm       = &(ttcState.algTlm);
tsALG_FDIR_STATE      * const AINT_pAlgFdir      = &(ttcState.algFdir);

tGDEF_UINT8           * const  AINT_pWheelFailNo = &(ttcState.algConfig.wheel.failedWheel);



/* TTC functions */
tGDEF_UINT32 * AINT_GetAlgTlmData(tGDEF_UINT16 chan)
{
   tGDEF_UINT16 index = 0;

   for (index = 0; index < AINT_numAlgTlmChans; index++)
   {
      if (AINT_algTlmLookup[index].chan == chan)
      {
         return (tGDEF_UINT32 *) AINT_algTlmLookup[index].pData;
      }
   }

   /* if not found then return NULL */
   return NULL;
}

tGDEF_UINT32 * AINT_GetAlgCmdTlmData(tGDEF_UINT16 chan)
{
   tGDEF_UINT16 index = 0;

   for (index = 0; index < AINT_numAlgCmdTlmChans; index++)
   {
      if (AINT_algCmdTlmLookup[index].chan == chan)
      {
         return (tGDEF_UINT32 *) AINT_algCmdTlmLookup[index].pData;
      }
   }

   /* if not found then return NULL */
   return NULL;
}


tGDEF_UINT32 * AINT_GetSensorTlmData(tGDEF_UINT16 chan)
{
   tGDEF_UINT16 index = 0;

   for (index = 0; index < AINT_numSensorTlmChans; index++)
   {
      if (AINT_sensorTlmLookup[index].chan == chan)
      {
         return (tGDEF_UINT32 *) AINT_sensorTlmLookup[index].pData;
      }
   }

   /* if not found then return NULL */
   return NULL;
}

void AINT_PackTlm()
{
   tGDEF_UINT8 unitNo = 0;
#if _FC_NUM_SCAM != 0
   tGDEF_UINT16 measNo;
#endif

   AINT_packedTlm[PACKED_EST_RP] = (( (tGDEF_UINT32) (AINT_pAlgTlm->euler_smooth[GDEF_X_AXIS]/100 & 0x0000FFFF)) |
                                    (( (tGDEF_UINT32) (AINT_pAlgTlm->euler_smooth[GDEF_Y_AXIS]/100 << 16 &0xFFFF0000)))) ;

   AINT_packedTlm[PACKED_GUIDE_RP] = (( (tGDEF_UINT32) (AINT_pAlgTlm->rpy_guide[GDEF_X_AXIS]/100 & 0x0000FFFF)) |
                                      (( (tGDEF_UINT32) (AINT_pAlgTlm->rpy_guide[GDEF_Y_AXIS]/100 << 16 &0xFFFF0000)))) ;


   AINT_packedTlm[PACKED_EST_WXY] = (( (tGDEF_UINT32) AINT_pAlgTlm->omega_smooth[GDEF_X_AXIS] & 0x0000FFFF) |
                                     (( (tGDEF_UINT32) AINT_pAlgTlm->omega_smooth[GDEF_Y_AXIS] << 16) &0xFFFF0000)) ;

   AINT_packedTlm[PACKED_EST_YAW_WZ] = (( (tGDEF_UINT32) (AINT_pAlgTlm->euler_smooth[GDEF_Z_AXIS]/100 & 0x0000FFFF)) |
                                        (( (tGDEF_UINT32) AINT_pAlgTlm->omega_smooth[GDEF_Z_AXIS] << 16) &0xFFFF0000)) ;


   AINT_packedTlm[PACKED_DIPOLE_XY] = (( (tGDEF_UINT32) AINT_pAlgTlm->dipole[GDEF_X_AXIS] )  & 0x0000FFFF) |
                                      (( (tGDEF_UINT32) ttcState.algTlm.dipole[GDEF_Y_AXIS] << 16) &0xFFFF0000) ;
   
   
   
   
   AINT_packedTlm[PACKED_MTQX] = (( (tGDEF_UINT32)(ttcState.algTlm.mtqCmd[GDEF_X_AXIS]) )  & 0x0000FFFF);
                                      
   AINT_packedTlm[PACKED_MTQY] = (( (tGDEF_UINT32)(ttcState.algTlm.mtqCmd[GDEF_Y_AXIS]))  & 0x0000FFFF);
                                      
   AINT_packedTlm[PACKED_MTQZ] = (( (tGDEF_UINT32)(ttcState.algTlm.mtqCmd[GDEF_Z_AXIS]) )  & 0x0000FFFF);
   
   AINT_packedTlm[PACKED_ATTYAW] = (( (tGDEF_UINT32)(ttcState.attitude.BackgroundAttitudeYaw) )  & 0x0000FFFF);
   
   AINT_packedTlm[PACKED_OBS_Z] = (( (tGDEF_UINT32)(ttcState.algTlm.mag_o_c[GDEF_Z_AXIS]) )  & 0x0000FFFF);
   
   AINT_packedTlm[PACKED_MTQ0_AIM0] = (( (tGDEF_UINT32)(ttcState.algCmd.torquer[AIM0][GDEF_X_AXIS]) )  & 0x0000FFFF);
   AINT_packedTlm[PACKED_MTQ1_AIM0] = (( (tGDEF_UINT32)(ttcState.algCmd.torquer[AIM0][GDEF_Y_AXIS]) )  & 0x0000FFFF);
   AINT_packedTlm[PACKED_MTQ2_AIM0] = (( (tGDEF_UINT32)(ttcState.algCmd.torquer[AIM0][GDEF_Z_AXIS]) )  & 0x0000FFFF);
   
   AINT_packedTlm[PACKED_MTQ0_AIM1] = (( (tGDEF_UINT32)(ttcState.algCmd.torquer[AIM1][GDEF_X_AXIS]) )  & 0x0000FFFF);
   AINT_packedTlm[PACKED_MTQ1_AIM1] = (( (tGDEF_UINT32)(ttcState.algCmd.torquer[AIM1][GDEF_Y_AXIS]) )  & 0x0000FFFF);
   AINT_packedTlm[PACKED_MTQ2_AIM1] = (( (tGDEF_UINT32)(ttcState.algCmd.torquer[AIM1][GDEF_Z_AXIS]) )  & 0x0000FFFF);
                                      

   
   AINT_packedTlm[PACKED_OBS_RES] =  (( (tGDEF_UINT32) AINT_pAlgTlm->mag_o_c[GDEF_X_AXIS] )  & 0x0000FFFF) |
                                     (( (tGDEF_UINT32) ttcState.algTlm.mag_o_c[GDEF_Y_AXIS] << 16) &0xFFFF0000) ;

   AINT_packedTlm[PACKED_SAS]    =  (((tGDEF_UINT32) AINT_pAlgTlm->sunazel[AZ_DATA]      ) & 0x0000FFFF) |
                                    (((tGDEF_UINT32) AINT_pAlgTlm->sunazel[EL_DATA] << 16) & 0xFFFF0000) ;

   AINT_packedTlm[PACKED_STATUS] =
      ((tGDEF_UINT32)(AINT_pAlgTlm->AdcsHealth                             & 0x0003)      ) |
      ((tGDEF_UINT32)(AINT_pTtc->test.configuration                        & 0x0001) << 2 ) |
      ((tGDEF_UINT32)(AINT_pAlgConfig->cpmest.Enable_MagSun_WhenYTM        & 0x0001) << 4 ) |
      ((tGDEF_UINT32)(AINT_pAlgTlm->SASClosedLoop                          & 0x0001) << 6 ) |
      ((tGDEF_UINT32)(AINT_pAlgTlm->eclipse                                & 0x0001) << 7 ) |
      ((tGDEF_UINT32)(AINT_pAlgConfig->cpmest.Enable_Dipole                & 0x0001) << 9 ) |
      ((tGDEF_UINT32)(AINT_pAlgConfig->mag.Disable_MTR                     & 0x0001) << 10) |
      ((tGDEF_UINT32)(AINT_pAlgConfig->prop.firingAttitude                 & 0x0007) << 11) |
      ((tGDEF_UINT32)(AINT_pAlgConfig->mag.failedAIM                       & 0x0001) << 13) |
      ((tGDEF_UINT32)(AINT_pAlgConfig->wheel.failedWheel                   & 0x0007) << 14) |
      ((tGDEF_UINT32)(AINT_pAlgConfig->cpmest.Enable_MagSunInit_by_MagOnly & 0x0001) << 19) |
      ((tGDEF_UINT32)(AINT_pAlgConfig->cpmest.Enable_MagSun_Colinear_Check & 0x0001) << 20) |
      ((tGDEF_UINT32)(AINT_pAlgTlm->MTMFrameCntAlgs                        & 0x0003) << 21) |
      ((tGDEF_UINT32)(AINT_pAlgConfig->mag.mtmSelected                     & 0x0001) << 23) |
      ((tGDEF_UINT32)(AINT_pAlgConfig->mode.EnableBackgroundAttitude       & 0x0001) << 24) |
      ((tGDEF_UINT32)(AINT_pAlgConfig->prop.usePropCorr                    & 0x0007) << 25);


   AINT_packedTlm[PACKED_WHL_SPD_01] = ((AINT_pSensorTlm->wheel.tlm[WHEEL0] >> 5 ) & 0x0000FFFF) |
                                       ((AINT_pSensorTlm->wheel.tlm[WHEEL1] << 11) & 0xFFFF0000);

   AINT_packedTlm[PACKED_WHL_SPD_23] = ((AINT_pSensorTlm->wheel.tlm[WHEEL2] >> 5 ) & 0x0000FFFF) |
                                       ((AINT_pSensorTlm->wheel.tlm[WHEEL3] << 11) & 0xFFFF0000);

#if _FC_NUM_SWHL != 0
   AINT_packedTlm[PACKED_SWHL_SPD] = ((AINT_pSensorTlm->swheel[WHEEL0].data >> 5 ) & 0x0000FFFF) |
                                     ((AINT_pAlgConfig->wheel.pdSpeed << 16)& 0xFFFF0000);
#endif



   AINT_packedTlm[PACKED_MTM0] =  ((AINT_pSensorTlm->mag.tlm[0][GDEF_X_AXIS] >> 2) & 0x000003FF)        |
                                  (((AINT_pSensorTlm->mag.tlm[0][GDEF_Y_AXIS] >> 2) & 0x000003FF) << 10) |
                                  (((AINT_pSensorTlm->mag.tlm[0][GDEF_Z_AXIS] >> 2) & 0x000003FF) << 20);

   AINT_packedTlm[PACKED_MTM1] =  ((AINT_pSensorTlm->mag.tlm[1][GDEF_X_AXIS] >> 2) & 0x000003FF)        |
                                  (((AINT_pSensorTlm->mag.tlm[1][GDEF_Y_AXIS] >> 2) & 0x000003FF) << 10) |
                                  (((AINT_pSensorTlm->mag.tlm[1][GDEF_Z_AXIS] >> 2) & 0x000003FF) << 20);


   /*** FDIR STATUS Byte ****/
   AINT_packedTlm[PACKED_FDIR_STATUS] = 0UL;


   AINT_packedTlm[PACKED_FDIR_STATUS] =  ((tGDEF_UINT32)(AINT_pAlgFdir->MTMTmValid          & 0x01)     ) |
                                         ((tGDEF_UINT32)(AINT_pAlgFdir->MTMMeasurementValid & 0x01) << 1) |
                                         ((tGDEF_UINT32)(AINT_pAlgFdir->SASTmValid          & 0x01) << 2) |
                                         ((tGDEF_UINT32)(AINT_pAlgFdir->SASMeasurementValid & 0x01) << 3);


   /* pack 3 valid flags for each wheel */
   for(unitNo = 0; unitNo < _FC_NUM_MWHL; unitNo++)
   {
      AINT_packedTlm[PACKED_FDIR_STATUS] |= ((tGDEF_UINT32)(AINT_pAlgFdir->MRWTmValid[unitNo]           & 0x01) << (7+unitNo*3) ) |
                                            ((tGDEF_UINT32)(AINT_pAlgFdir->MRWMeasurementValid[unitNo]  & 0x01) << (8+unitNo*3) ) |
                                            ((tGDEF_UINT32)(AINT_pAlgFdir->MRWSpeedCmdValid[unitNo]        & 0x01) << (9+unitNo*3) );
   }

   AINT_packedTlm[PACKED_FDIR_STATUS]  |= ((tGDEF_UINT32)(AINT_pAlgFdir->MagOCValid      & 0x01) << 19 ) |
                                          ((tGDEF_UINT32)(AINT_pAlgFdir->SunOCValid      & 0x01) << 20 ) |
                                          ((tGDEF_UINT32)(AINT_pAlgFdir->ControllerValid & 0x01) << 21 );
#if _FC_NUM_SCAM != 0
   AINT_packedTlm[PACKED_FDIR_STATUS]  |= ((tGDEF_UINT32)(AINT_pAlgFdir->STRTmValid[0]      & 0x01) << 19 ) |
                                          ((tGDEF_UINT32)(AINT_pAlgFdir->STRQuaternionValid[0]  & 0x01) << 20 ) |
                                          ((tGDEF_UINT32)(AINT_pAlgFdir->STRTimeStampValid[0] & 0x01) << 21 );
#endif

   /*SECOND FDIR TLM for remaining SCAM flags*/
   AINT_packedTlm[PACKED_FDIR_STATUS_1] = 0L;

#if _FC_NUM_SCAM != 0
   AINT_packedTlm[PACKED_FDIR_STATUS_1] |= ((tGDEF_UINT32)(ttcState.algFdir.STRTmValid[0][3] & 0x01) << 0 )|
                                           ((tGDEF_UINT32)(ttcState.algFdir.STRQuaternionValid[0][3] & 0x01) << 1 ) |
                                           ((tGDEF_UINT32)(ttcState.algFdir.STRTimeStampValid[0][3]  & 0x01) << 2 ); ;


   /* pack bits for each star camera */
   for(measNo = 0; measNo < _FC_NUM_SCAM_MEAS ; measNo++)
   {

      AINT_packedTlm[PACKED_FDIR_STATUS_1] |= ((tGDEF_UINT32)(ttcState.algFdir.STRTmValid[1][measNo] & 0x01) << (3 + 3*measNo) )|
                                              ((tGDEF_UINT32)(ttcState.algFdir.STRQuaternionValid[1][measNo] & 0x01) << (4 + 3*measNo) ) |
                                              ((tGDEF_UINT32)(ttcState.algFdir.STRTimeStampValid[1][measNo]  & 0x01) << (5 + 3*measNo) ); ;
   }


#endif

   for(unitNo = 0; unitNo < _FC_NUM_MWHL; unitNo++)
   {
      AINT_packedTlm[PACKED_FDIR_STATUS_1] |= ((tGDEF_UINT32)(ttcState.algFdir.MRWSpeedTrackValid[unitNo] & 0x01) << (15 + unitNo) );

   }

   for(unitNo = 0; unitNo < _FC_NUM_MWHL; unitNo++)
   {
      AINT_packedTlm[PACKED_FDIR_STATUS_1] |= ((tGDEF_UINT32)(ttcState.algFdir.MRWTorqueCmdValid[unitNo] & 0x01) << (19 + unitNo) );

   }

#if _FC_NUM_SWHL != 0
   for(unitNo = 0; unitNo < _FC_NUM_SWHL; unitNo++)
   {
      AINT_packedTlm[PACKED_FDIR_STATUS_1] |= ((tGDEF_UINT32)(ttcState.algFdir.SP100MeasurementValid[unitNo] & 0x01) << (23 + unitNo) );

   }

   for(unitNo = 0; unitNo < _FC_NUM_SWHL; unitNo++)
   {
      AINT_packedTlm[PACKED_FDIR_STATUS_1] |= ((tGDEF_UINT32)(ttcState.algFdir.SP100SpeedTrackValid[unitNo] & 0x01) << (24 + unitNo) );
   }

   for(unitNo = 0; unitNo < _FC_NUM_SWHL; unitNo++)
   {
      AINT_packedTlm[PACKED_FDIR_STATUS_1] |= ((tGDEF_UINT32)(ttcState.algFdir.SRWTmValid[unitNo] & 0x01) << (25 + unitNo) );
   }




#endif


#if _FC_NUM_GYR != 0

   AINT_packedTlm[PACKED_GYR1_XY] = (AINT_pSensorTlm->gyr[GYR1].data[GDEF_X]    & 0x0000FFFF)        |
                                    ((AINT_pSensorTlm->gyr[GYR1].data[GDEF_Y]  & 0x0000FFFF) <<16);

   AINT_packedTlm[PACKED_GYR1_ZT] = (AINT_pSensorTlm->gyr[GYR1].data[GDEF_Z]    & 0x0000FFFF)        |
                                    ((AINT_pSensorTlm->gyr[GYR1].temp[GDEF_X]  & 0x0000FFFF) <<16);

   AINT_packedTlm[PACKED_GYR2_XY] = (AINT_pSensorTlm->gyr[GYR2].data[GDEF_X] & 0x0000FFFF)        |
                                    ((AINT_pSensorTlm->gyr[GYR2].data[GDEF_Y]  & 0x0000FFFF) <<16);

   AINT_packedTlm[PACKED_GYR2_ZT] = (AINT_pSensorTlm->gyr[GYR2].data[GDEF_Z] & 0x0000FFFF)        |
                                    ((AINT_pSensorTlm->gyr[GYR2].temp[GDEF_X]  & 0x0000FFFF) <<16);

   AINT_packedTlm[PACKED_GYR3_XY] = (AINT_pSensorTlm->gyr[GYR3].data[GDEF_X] & 0x0000FFFF)        |
                                    ((AINT_pSensorTlm->gyr[GYR3].data[GDEF_Y]  & 0x0000FFFF) <<16);

   AINT_packedTlm[PACKED_GYR3_ZT] = (AINT_pSensorTlm->gyr[GYR3].data[GDEF_Z] & 0x0000FFFF)        |
                                    ((AINT_pSensorTlm->gyr[GYR3].temp[GDEF_X]  & 0x0000FFFF) <<16);

   AINT_packedTlm[PACKED_GYR0_XY] =  ((AINT_pSensorTlm->gyr[GYR0].data[GDEF_X] & 0x00FFFF00) >> 8)     |
                                     ((AINT_pSensorTlm->gyr[GYR0].data[GDEF_Y] & 0x00FFFF00) << 8)  ;

   AINT_packedTlm[PACKED_GYR0_ZT] =  ((AINT_pSensorTlm->gyr[GYR0].data[GDEF_Z] & 0x00FFFF00) >> 8)     |
                                     ((AINT_pSensorTlm->gyr[GYR0].temp[GDEF_X] & 0x0000FFFF) << 16)  ;

#endif

}


/* Function to return the length of a file from its current position */
tGDEF_UINT32 GetFileLenToEnd(FILE *fp)
{
   tGDEF_INT32  fileStart = 0;
   tGDEF_UINT32 fileLen   = 0;

   fileStart = ftell(fp);                              /* Store he start position              */
   fseek(fp, 0L, SEEK_END);                            /* move file pointer to end             */
   fileLen = (tGDEF_UINT32) (ftell(fp) - fileStart);   /* get file length from the start value */
   fseek(fp, fileStart, SEEK_SET);                     /* rewind back to the start position    */

   return fileLen;
}

void AlgDelay(tGDEF_UINT16 delay)
{
   time_t unixStart = 0;
   struct timespec tm;

   tGDEF_INT16 ticksEnd = 0;

   time_t unix = 0;
   tGDEF_INT16 ticks = 0;

   clock_gettime(CLOCK_REALTIME,&tm);
   ticks = tm.tv_sec;
   // qcf_today(&unixStart, &ticks);

   ticksEnd = ticks + delay; //AINT_ticksDelay[mode];

   while(ticks < ticksEnd)
   {
      /*   get current time to tick resolution  */
      //qcf_today(&unix, &ticks);

      /* add on any extra seconds */
      ticks += (tGDEF_INT16) ((unix-unixStart)*100 );
   }

}

void SetAlgDelay(tGDEF_UINT8 mode,  tGDEF_INT16 ticks)
{
   AINT_ticksDelay[mode] = ticks;
}



void AINT_Init(teGDEF_BOOLEAN ResetAllFlag)
{
   tGDEF_UINT8 modeNo = 0;

   if (GDEF_TRUE == ResetAllFlag)
   {
      EWOD_WriteMessage("AINT_Init: ttcState set to 0 \n");

      /* ensure the ttcstate is ste to 0 */
      memset(&ttcState, 0, sizeof(ttcState)); /* ! this also clears the enable flags ! */
   }

   ttcState.errorFlag                 = 0;
   ttcState.attitude.cmd[GDEF_X_AXIS] = 0;     /* roll/pitch/yaw command */
   ttcState.attitude.cmd[GDEF_Y_AXIS] = 0;     /* roll/pitch/yaw command */
   ttcState.attitude.cmd[GDEF_Z_AXIS] = 0;     /* roll/pitch/yaw command */
   ttcState.attitude.initFlag         = 0;

#ifdef _NOT_NEEDED_
   ttcState.algConfig.YbiasMometum = 0;   /* small y-bias operation with 4 wheels */

   ttcState.algConfig.WheelSpeedOffset = 50; //*  y wheel speed command in rpm tc needed (for LEOP) */
#endif

   /* intialise test state off */
   ttcState.test.isEnabled     = GDEF_FALSE;
   ttcState.test.configuration = 0;			 /* test state 1 so run LCM */

   /* set initial delay values for test state */
   for(modeNo = 0; modeNo < NUM_MODES; modeNo++)
   {
      ttcState.test.delay[modeNo] = AINT_ticksDelay[modeNo];
   }

   ttcState.mode.current    = MODE_SBM;
   ttcState.mode.requested  = MODE_SBM;
   ttcState.mode.isReset    = GDEF_FALSE;
   ttcState.mode.isSafeMode = GDEF_FALSE;
#ifdef _NOT_NEEDED_
   ttcState.mode.enableSnsPtr = AINT_GetSensorEnableFlags();
   ttcState.mode.enableActPtr = AINT_GetActuatorEnableFlags();
#endif

   ttcState.dt_nom = AOCS_DEFAULT_CYCLE_TIME;

   /* ALG configuration for mtm & mtq */
   ttcState.algConfig.mag.Disable_MTR   				       = GDEF_FALSE;
   ttcState.algConfig.cpmest.Enable_MagSunInit_by_MagOnly = GDEF_TRUE;

   ttcState.algConfig.MappingComplete                     = OFF; /* mapping would also be reset on a mode reset (mode.isReset = 1) */

   /* Initialise all FDIR enable flag to HIGH */

   /* MTM tm and measurement check */
   ttcState.algConfig.fdir.Enable_NavMagTmValidCheck          = GDEF_TRUE;
   ttcState.algConfig.fdir.Enable_NavMagMeasurementValidCheck = GDEF_TRUE;

   /* SunSensor tm and measurement check */
   ttcState.algConfig.fdir.Enable_SunSensorTmValidCheck          = GDEF_TRUE;
   ttcState.algConfig.fdir.Enable_SunSensorMeasurementValidCheck = GDEF_TRUE;

   /* Microwheel tm and measurement check */
   ttcState.algConfig.fdir.Enable_MRWMeasurementValidCheck = GDEF_TRUE;
   ttcState.algConfig.fdir.Enable_MRWTmValidCheck          = GDEF_TRUE;
   ttcState.algConfig.fdir.Enable_MRWSpeedTrackValidCheck  = GDEF_TRUE;
   ttcState.algConfig.fdir.Enable_MRWTorqueCmdValidCheck   = GDEF_TRUE;
   ttcState.algConfig.fdir.Enable_MRWSpeedCmdValidCheck    = GDEF_TRUE;

#if _FC_NUM_SWHL != 0
   ttcState.algConfig.fdir.Enable_SP100MeasurementValidCheck  = GDEF_TRUE;
   ttcState.algConfig.fdir.Enable_SP100SpeedTrackValidCheck   = GDEF_TRUE;

#endif

#if _FC_NUM_SCAM != 0
   /* Star Tracker tm and measurement check */
   ttcState.algConfig.fdir.Enable_STRQuaternionValidCheck = GDEF_TRUE;
   ttcState.algConfig.fdir.Enable_STRTimestampValidCheck  = GDEF_TRUE;
   ttcState.algConfig.fdir.Enable_STRTmValidCheck         = GDEF_TRUE;
   ttcState.algConfig.fdir.Enable_STRBlindingValidCheck   = GDEF_TRUE;
   ttcState.algConfig.fdir.Enable_STRDPUValidFlagCheck    = GDEF_TRUE;
#endif

#if _FC_NUM_GPS != 0
   /* GPS tm and measurement check */
   ttcState.algConfig.fdir.Enable_GPSTmValidCheck          = GDEF_TRUE;
   ttcState.algConfig.fdir.Enable_GPSMeasurementRangeCheck = GDEF_TRUE;
   ttcState.algConfig.fdir.Enable_GPSMeasurementValidCheck = GDEF_TRUE;
#endif

   ttcState.algConfig.fdir.Enable_MagOCValidCheck      = GDEF_TRUE;      /* Mag o-c 	  */
   ttcState.algConfig.fdir.Enable_SunOCValidCheck      = GDEF_TRUE;      /* SunSensor o-c */
   ttcState.algConfig.fdir.Enable_ControllerValidCheck = GDEF_TRUE;

   /* set tlm data for algorithms invalid */
   AINT_SetDataInvalid();

   /* reset flags in algs */
   _fc_AOCS_ReInit();

   /* update telemetry */
   AINT_PackTlm();
}


teGDEF_FUNC_STATUS AINT_SetFDIREnableFlags(tGDEF_UINT8 CheckID, tGDEF_UCHAR CheckState)
{
   tGDEF_UINT16 index = 0;

   for (index = 0; index < AINT_numFDIRChecks; index++)
   {
      if (AINT_FdirCheckLookup[index].CheckID == CheckID)
      {
         *AINT_FdirCheckLookup[index].CheckState = CheckState;
         return GDEF_SUCCESS;
      }
   }

   /* if not found then return failed */
   return GDEF_FAILURE;

}





teGDEF_FUNC_STATUS AINT_SetSensorEnableState(teHMGR_HW_SENSORS sensor,tGDEF_UINT16 modeNo, tGDEF_UINT8 snsNo, teGDEF_BOOLEAN state)
{
   tsALG_ENABLE_SENSOR*    enableSns;
   enableSns = &ttcState.enableSns[modeNo];

   switch(sensor)
   {
   case HMGR_SNS_MTM:
      if (snsNo >= _FC_NUM_MTM)
         return GDEF_FAILURE;
      enableSns->mtm[snsNo] = state;
      strcpy(EWOD_sLogMsg, "AINT_SetSensorEnableState: Set Enable SNS-MTM");
      break;

   case HMGR_SNS_SAS:
      if (snsNo >= _FC_NUM_SAS)
         return GDEF_FAILURE;
      enableSns->sas[snsNo] = state;
      strcpy(EWOD_sLogMsg, "AINT_SetSensorEnableState: Set Enable SNS-SAS");
      break;

   case HMGR_SNS_MWHL:
      if (snsNo >= _FC_NUM_MWHL)
         return GDEF_FAILURE;
      enableSns->whl[snsNo] = state;
      strcpy(EWOD_sLogMsg, "AINT_SetSensorEnableState: Set Enable SNS-MW");
      break;

#if _FC_NUM_SWHL != 0
   case HMGR_SNS_SWHL:
      if (snsNo >= _FC_NUM_SWHL)
         return GDEF_FAILURE;
      enableSns->swhl[snsNo] = state;
      strcpy(EWOD_sLogMsg, "AINT_SetSensorEnableState: Set Enable SNS-SW");
      break;
#endif

#if _FC_NUM_FSS != 0
   case HMGR_SNS_FSS:
      if (snsNo >= _FC_NUM_FSS)
         return GDEF_FAILURE;

      enableSns->fss[snsNo] = state;
      strcpy(EWOD_sLogMsg, "AINT_SetSensorEnableState: Set Enable SNS -FSS");
      break;
#endif

#if _FC_NUM_GYR != 0
   case HMGR_SNS_GYR:
      if (snsNo >= _FC_NUM_GYR)
         return GDEF_FAILURE;

      enableSns->gyr[snsNo] = state;
      strcpy(EWOD_sLogMsg, "AINT_SetSensorEnableState: Set Enable SNS -GYR");
      break;
#endif

   default:
      /* invalid sensor type */
      return GDEF_FAILURE;
   }

   sprintf( &EWOD_sLogMsg[strlen(EWOD_sLogMsg)],"%d",(tGDEF_UINT16) snsNo);
   strcat(EWOD_sLogMsg, " for mode ");
   sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", modeNo);
   strcat(EWOD_sLogMsg, " to ");
   sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", (tGDEF_UINT16) state);
   strcat(EWOD_sLogMsg, "\n");
   EWOD_WriteMessage(EWOD_sLogMsg);

   /* if we changed enable states for current mode the re-initialise mapping */
   if (modeNo == ttcState.mode.current)
   {
      ttcState.algConfig.MappingComplete = OFF;
#ifdef _SOLICITED_
      HMGR_ReqInitSample(); /* ensure sampling is updated to reflect the changes */
#endif
   }
   return GDEF_SUCCESS;
}


teGDEF_FUNC_STATUS AINT_SetActuatorEnableState(teHMGR_HW_ACTUATORS actuator,tGDEF_UINT16 modeNo, tGDEF_UINT8 actNo, teGDEF_BOOLEAN state)
{
   tsALG_ENABLE_ACTUATOR*    enableAct;
   enableAct = &ttcState.enableAct[modeNo];

   switch(actuator)
   {
   case HMGR_ACT_MTQ:
      if (actNo >= _FC_NUM_AIM)
         return GDEF_FAILURE;
      enableAct->mtq[actNo] = state;
      strcpy(EWOD_sLogMsg, "AINT_SetActuatorEnableState: Set Enable ACT-MTQ");
      break;

   case HMGR_ACT_MWHL:
      if (actNo >= _FC_NUM_MWHL)
         return GDEF_FAILURE;
      enableAct->whl[actNo] = state;
      strcpy(EWOD_sLogMsg, "AINT_SetActuatorEnableState: Set Enable ACT-MW");
      break;

#if _FC_NUM_SWHL != 0
   case HMGR_ACT_SWHL:
      if (actNo >= _FC_NUM_SWHL)
         return GDEF_FAILURE;

      enableAct->swhl[actNo] = state;
      strcpy(EWOD_sLogMsg, "AINT_SetActuatorEnableState: Set Enable ACT-SW");
      break;
#endif

   default:
      /* invalid sensor type */
      return GDEF_FAILURE;
   }

   sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)],"%d", (tGDEF_UINT16) actNo);
   strcat(EWOD_sLogMsg, " for mode ");
   sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)],"%d", modeNo);
   strcat(EWOD_sLogMsg, " to ");
   sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)],"%d",(tGDEF_UINT16) state);
   strcat(EWOD_sLogMsg, "\n");
   EWOD_WriteMessage(EWOD_sLogMsg);

   /* if we changed enable states for current mode the re-initialise mapping */
   if (modeNo == ttcState.mode.current)
      ttcState.algConfig.MappingComplete = OFF;

   return GDEF_SUCCESS;
}


teGDEF_FUNC_STATUS AINT_SetEnableStateFromDatabase( tGDEF_UCHAR ModeType )
{
   /* call the function in the algs to set the values from the database and re-initialise the mapping */
   EnableFlags_SetDatabaseValues(AINT_pTtc, ModeType);

   ttcState.algConfig.MappingComplete = OFF;

   return GDEF_SUCCESS;
}

tsAOCS_TTC  * AINT_GetTtcState(void)
{
   return &ttcState;
}

tsALG_CONFIG * AINT_GetAlgConfig(void)
{
   return &(ttcState.algConfig);
}

tsALG_TCMD * AINT_GetAlgCmds(void)
{
   return &(ttcState.algCmd);
}

tsALG_TLM * AINT_GetAlgTlm(void)
{
   return &(ttcState.algTlm);
}

tsALG_ATTITUDE_DEMAND * AINT_GetAttitudeDemand(void)
{
   return &(ttcState.attitude);
}

tsALG_SENSOR_TLM * AINT_GetSensorTlm(void)
{
   return &(ttcState.tlm);
}

tsALG_MODE * AINT_GetMode(void)
{
   return &(ttcState.mode);
}


void AINT_Set_LogMode( tGDEF_UINT8 mode, teGDEF_BOOLEAN safeMode)
{
   if (GDEF_TRUE == safeMode && mode >= NUM_SAFE_MODES)
      return;
   if (mode >= NUM_MODES)
      return;

   AINT_LogMode     = mode;
   AINT_LogSafeMode = safeMode;

}
tGDEF_UINT32 AINT_GetPackedSensorEnableFlagsArray(void)
{
   tGDEF_UINT32 		 retVal = 0;
   tGDEF_UCHAR          bitNo  = 0;
   tGDEF_UCHAR          unitNo = 0;
   tsALG_ENABLE_SENSOR* enableSns;

   if (GDEF_FALSE == AINT_LogSafeMode)
   {
      enableSns = &AINT_pTtc->enableSns[AINT_LogMode];
   }
#if NUM_SAFE_MODES != 0
   else
   {
      enableSns = &AINT_pTtc->SAFEenableSns[AINT_LogMode];
   }
#endif

   retVal = (tGDEF_UINT32)(0xFF & AINT_LogMode) ;
   retVal|= (tGDEF_UINT32)((0x01 & AINT_LogSafeMode) << 8);
   bitNo = 9;


   for ( unitNo = 0; unitNo < _FC_NUM_MTM; unitNo ++)
   {
      retVal |=  ((tGDEF_UINT32)(enableSns->mtm[unitNo] & 0x01)<< bitNo);
      bitNo++;
   }
   for ( unitNo = 0; unitNo < _FC_NUM_SAS; unitNo ++)
   {
      retVal |=  ((tGDEF_UINT32)(enableSns->sas[unitNo] & 0x01) << bitNo) ;
      bitNo++;
   }
   for ( unitNo = 0; unitNo < _FC_NUM_MWHL; unitNo ++)
   {
      retVal |=  ((tGDEF_UINT32)(enableSns->whl[unitNo] & 0x01) << bitNo) ;
      bitNo++;
   }

   return retVal;
}

tGDEF_UINT32 AINT_GetPackedActuatorEnableFlagsArray(void)
{
   tGDEF_UINT32           retVal = 0;
   tGDEF_UCHAR            bitNo  = 0;
   tGDEF_UCHAR            unitNo = 0;
   tsALG_ENABLE_ACTUATOR* enableAct;

   if ( GDEF_FALSE == AINT_LogSafeMode)
   {
      enableAct = &AINT_pTtc->enableAct[AINT_LogMode];
   }
#if NUM_SAFE_MODES != 0
   else
   {
      enableAct = &AINT_pTtc->SAFEenableAct[AINT_LogMode];
   }
#endif

   retVal = (tGDEF_UINT32)(0xFF & AINT_LogMode) ;
   retVal|= (tGDEF_UINT32)((0x01 & AINT_LogSafeMode) << 8);
   bitNo = 9;


   for ( unitNo = 0; unitNo < _FC_NUM_AIM; unitNo ++)
   {
      retVal |=  ((tGDEF_UINT32)(enableAct->mtq[unitNo] & 0x01)<< bitNo);
      bitNo++;
   }
   for ( unitNo = 0; unitNo < _FC_NUM_MWHL; unitNo ++)
   {
      retVal |=  ((tGDEF_UINT32)(enableAct->whl[unitNo] & 0x01) << bitNo) ;
      bitNo++;
   }

   return retVal;
}

/***************************************************************************
    function: NG_AOCS_LoadDriveFile
    actions :
    author  : yoshi
    date    : 27-07-2007
****************************************************************************/
teGDEF_FUNC_STATUS  AINT_LoadDrive(tGDEF_UINT32 fnum)
{
   teGDEF_FUNC_STATUS retVal                     = GDEF_FAILURE;
   teGDEF_BOOLEAN     fileFound                  = GDEF_FALSE;
   tGDEF_INT8         filename[MAX_HEADER_CHARS] = {0};

   FILE *fp = NULL;   /* drive file pointer, */

   tGDEF_UINT32   fileLen              = 0;
   tGDEF_UINT16   dataLen              = 0;
   teGDEF_BOOLEAN initialConfiguration = GDEF_FALSE;

   /* check if to deconfigure AJ - move to a separate function??*/
   if (fnum == 0UL)
   {
      /* go to standby mode to stop algs from executing */
      AMOH_ForceMode(MODE_SBM);

      /* AJ - reset database values to 0?? */

      /* log message */
      strcpy(EWOD_sLogMsg, "ERROR -AINT_LoadDrive: Deconfigured successfully\n");
#ifdef REALTIME_DEBUG
      (void)printf("Error: Deconfigured succesfully\n");
#endif
      EWOD_WriteMessage(EWOD_sLogMsg);
      /* store drive file number */
      AFIH_driveFileNum = fnum;

      /* complete so return success */
      return GDEF_SUCCESS;
   }

   fileFound = file_name_obtain(fnum, &filename[0], FILENAME_SIZE);
   if (fileFound == GDEF_FALSE)
   {
      /* log message */
      strcpy(EWOD_sLogMsg, "ERROR -AINT_LoadDrive: File not found\n");
#ifdef REALTIME_DEBUG
      (void)printf("Error: File not found\n");
#endif
      EWOD_WriteMessage(EWOD_sLogMsg);
   }

   /* Try and open the file */
#ifndef TEST_ON_OBC
   sprintf(filename,"\\TEMP\\%d",fnum); /* fopen points to C:\TEMP in Yannis PC */
#endif

   fp = fopen(filename, "rb");

   /* check if file opened successfully */
   if (fp == NULL)
   {
      /* Log error and return failure */
      EWOD_WriteMessage("ERROR -AINT_LoadDrive: Unable to open file\n");
#ifdef REALTIME_DEBUG 
      (void)printf("Error unable to open file\n");
#endif
      return GDEF_FAILURE;

   }

   /* Read & check the drive file header */
   if(GDEF_SUCCESS != ReadDriveHdr(fp, &dataLen))
   {
      /* log error*/
      strcpy(EWOD_sLogMsg, "ERROR -AINT_LoadDrive: Load failed  -Invalid header\n");
#ifdef REALTIME_DEBUG 
      (void)printf("Error invalid header\n");
#endif
      EWOD_WriteMessage(EWOD_sLogMsg);

      /* close the drive file and ensure file pointer is null */
      fclose(fp);
      fp = NULL;

      /* return failure */
      return GDEF_FAILURE;
   }

   /* get the length to the end of the file */
   fileLen = GetFileLenToEnd(fp);

   /* check data length is consistent with the value given in the header */
   if (fileLen != (tGDEF_UINT32) dataLen)
   {
      /* log error & return */
      strcpy(EWOD_sLogMsg, "ERROR -AINT_LoadDrive: Inconsistent drive file length: exp  [");
      sprintf( &EWOD_sLogMsg[strlen(EWOD_sLogMsg)],"%d", (tGDEF_UINT32) dataLen);
      strcat(EWOD_sLogMsg, "] act [");
      sprintf( &EWOD_sLogMsg[strlen(EWOD_sLogMsg)],"%d",fileLen);
      strcat(EWOD_sLogMsg, "]\n");
      EWOD_WriteMessage(EWOD_sLogMsg);
      return GDEF_FAILURE;
   }

   /* load the file data into the adcs database */
   if ( _fc_AOCS_LoadDriveFile(fp, dataLen) == 0)
   {
      /* signal success */
      /* log message */
      strcpy(EWOD_sLogMsg, "AINT_LoadDrive: Load successful\n");
      EWOD_WriteMessage(EWOD_sLogMsg);
#ifdef REALTIME_DEBUG 
      (void)printf("Load success\n");
#endif
      if (AFIH_NO_FILE == AFIH_driveFileNum)
      {
         initialConfiguration = GDEF_TRUE;
         strcpy(EWOD_sLogMsg, "AINT_LoadDrive: Initial Configuration\n");
         EWOD_WriteMessage(EWOD_sLogMsg);
#ifdef REALTIME_DEBUG
         (void)printf("Initial Config\n");
#endif /* REALTIME_DEBUG */
      }
      /* store drive file number */
      AFIH_driveFileNum = fnum;

      retVal = GDEF_SUCCESS;
   }
   else
   {
      /* log data len error */
      strcpy(EWOD_sLogMsg, "ERROR -AINT_LoadDrive: err [" );
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)],"%d", dataLen);
      strcat(EWOD_sLogMsg, "], data len exp [");
      sprintf( &EWOD_sLogMsg[strlen(EWOD_sLogMsg)],"%d",sizeof(ts_DataBase));
      strcat(EWOD_sLogMsg, "] act [");
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)],"%d",dataLen);
      strcat(EWOD_sLogMsg, "] \n");
      EWOD_WriteMessage (EWOD_sLogMsg);
#ifdef REALTIME_DEBUG
      (void)printf("Data length error %d expected: %d\n",dataLen,sizeof(ts_DataBase));
#endif /* REALTIME_DEBUG */
   }

   if (GDEF_TRUE == initialConfiguration)
   {
      /* initialise the enable flags from the drive file! */
      AINT_SetEnableStateFromDatabase(_FC_RESET_BOTH);
   }

   /* close the drive file and ensure file pointer is null */
   fclose(fp);
   fp = NULL;

   /* return function status */
   return retVal;

}

teGDEF_FUNC_STATUS ReadDriveHdr(FILE *fp, tGDEF_UINT16 * pDataLen)
{

   tGDEF_CHAR   sId[AOCS_DRIVE_FILE_ID_SIZE+1] = {0};
   tGDEF_CHAR   sSatId[SATID]                  = {0};
   tGDEF_UINT16 version                        = 0;

#ifndef TEST_ON_OBC
   tGDEF_UINT16 flipped_version;
   tGDEF_UINT16 non_flipped_DataLen;
#endif

   if(fp == NULL)
      return GDEF_FAILURE;

   /* Skip the PacSat File Header. */
   pfh_skip(fp);

   /* read in header parameters from drive file */
   fread (sId, 1, AOCS_DRIVE_FILE_ID_SIZE, fp);   /* aocs id */
   fread (sSatId, 1, AOCS_DRIVE_SAT_ID_SIZE, fp); /* aocs id */
   fread (&version, 2, 1, fp);   				  /* version */

#ifndef TEST_ON_OBC
   fread (&non_flipped_DataLen, 2, 1, fp);   //  data size

   flipped_version = ((version >> 8) | (version << 8)) & 0xFFFF;
   *pDataLen = ((non_flipped_DataLen >> 8) |  (non_flipped_DataLen << 8)) & 0xFFFF;
#else
   fread (pDataLen, 2, 1, fp);   /* data size */
#endif
   /* integrity checks */
   if(strncmp(sId, AOCS_DRIVE_FILE_ID, AOCS_DRIVE_FILE_ID_SIZE) != 0)
   {
      /* log error & return */
#ifdef REALTIME_DEBUG
      (void)printf("Invalid drive file id\n");
#endif /* REALTIME_DEBUG */
      EWOD_WriteMessage("Invalid drive file id\n");
      return GDEF_FAILURE;
   }

   if(strncmp(sSatId, AOCS_DRIVE_SAT_ID, AOCS_DRIVE_SAT_ID_SIZE) != 0)
   {
      /* log error & return */
#ifdef REALTIME_DEBUG
      (void)printf("Invalid drive file sat id\n");
#endif /* REALTIME_DEBUG */
      strcpy(EWOD_sLogMsg, "Invalid drive file sat id\n");
      EWOD_WriteMessage(EWOD_sLogMsg);
      return GDEF_FAILURE;
   }
#ifndef TEST_ON_OBC
   if(flipped_version != AOCS_DRIVE_FILE_VERSION)
#else
   if(version != AOCS_DRIVE_FILE_VERSION)
#endif
   {
      /* log error & return */
#ifdef REALTIME_DEBUG
      (void)printf("Invalid drive file version\n");
#endif /* REALTIME_DEBUG */
      strcpy(EWOD_sLogMsg, "Invalid drive file version\n");
      EWOD_WriteMessage(EWOD_sLogMsg);
      return GDEF_FAILURE;
   }

   /* data len */
   strcpy(EWOD_sLogMsg, "ReadDriveHdr: data len ");
   sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", *pDataLen);
#ifdef REALTIME_DEBUG
   (void)printf("ReadDriveHdr: data len %d\n",*pDataLen);
#endif /* REALTIME_DEBUG */
   EWOD_WriteMessage (EWOD_sLogMsg);

   return GDEF_SUCCESS;

}

teGDEF_FUNC_STATUS  AINT_AlgManager(void)
{
   teGDEF_FUNC_STATUS retVal = GDEF_FAILURE;
   tGDEF_INT16 callVal       = 0;
   ts_UNIX_TIME ut           = {0};

   /* set the current unix time */
   ut = AINT_GetUnixTime();
   ttcState.unixSecs  = ut.sec;
   ttcState.unixTicks = ut.msec/10;

   /* reset the requested mode */
   ttcState.mode.requested = ttcState.mode.current;

   /* nothing to do if drive file has not been loaded or in standby*/
   /* want to call the aocs_go function also in standby to ensure the buffer lastRunTime is initialised correctly
   after longer period of standby should always go via DTM to ensure timing buffers are sensible again
   also for Kaz Standby is safe mode but with logging of telemetry so need to call the sensor processing in aocs_go*/

   if(AFIH_driveFileNum == 0)
      return GDEF_SUCCESS;

   /* run the algorithms AJ - change prototype */
   callVal = AOCS_Go (&ttcState, ttcState.mode.isReset);

   /*SF: need to check if there are alg errors that shall not trigger mode trans = AROH_ALG_ERR*/
   if(ttcState.errorFlag)
   {
      AROH_SetDecFlag(AROH_ALGFDIR_ERR);
      ttcState.errorFlag = GDEF_FALSE;
      /* too many messages! */
      strcpy(EWOD_sLogMsg, "Algorithm ARO error no: ");
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", callVal);
      strcat(EWOD_sLogMsg, "\n" );
      //EWOD_WriteMessage(EWOD_sLogMsg);
   }

   if(ttcState.test.isEnabled)
   {
      AlgDelay(ttcState.test.delay[ttcState.mode.current]);
   }

   /*SF: needs review - what if callVal was not success then should we still set thr is Reset flag back ?*/
   if (ttcState.mode.isReset)
   {
      ttcState.mode.isReset = GDEF_FALSE;
   }
   else
   {
      /* check if mode has completed and requests a change (if allowed) */
      if (GDEF_TRUE == AINT_isModeReqAllowed)
      {
         AMOH_RequestMode((tGDEF_UINT8) ttcState.mode.requested);
      }
   }

   /* update the alg telemetry */
   AOCS_Telemetry(&ttcState);

   /* pack the telemetry */
   AINT_PackTlm();

   /* check if algorithms ran successfully */
   if (callVal == 0)
   {
      /* if algs ran successfully - update last run time */
      retVal = GDEF_SUCCESS;
   }
   else
   {
      strcpy(EWOD_sLogMsg, "Algorithm Error: ");
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", callVal);
      strcat(EWOD_sLogMsg, "\n");
      EWOD_WriteMessage(EWOD_sLogMsg);
   }

   return retVal;
}


/*tGDEF_UINT8  * AINT_GetWheelFailNo()
{
   return (tGDEF_UINT8 *) &ttcState.algConfig.wheel.failedWheel;
}
*/

tsALG_ENABLE_SENSOR * AINT_GetSensorEnableFlags()
{
   tGDEF_UINT8 mode = AMOH_GetMode();

#if NUM_SAFE_MODES != 0
   if(GDEF_FALSE == AMOH_GetSafeMode() )
      return &(ttcState.enableSns[mode]);

   else
      return &(ttcState.SAFEenableSns[mode]);
#else
   return &(ttcState.enableSns[mode]);
#endif
}


tsALG_ENABLE_ACTUATOR * AINT_GetActuatorEnableFlags()
{
   tGDEF_UINT8 mode = AMOH_GetMode();
#if NUM_SAFE_MODES != 0
   if(GDEF_FALSE == AMOH_GetSafeMode() )
      return &(ttcState.enableAct[mode]);
   else
      return &(ttcState.SAFEenableAct[mode]);
#else
   return &(ttcState.enableAct[mode]);
#endif
}



/*Function used in SOLICITED and UNSOLICITED mode to reset validity flags
in SOLICITED operation this will also set the telemetry to invalid values
in UNSOLICITED operation this is handled in the HINT_UNIT functions*/
void AINT_SetDataInvalid(void)
{
   tGDEF_UINT8 unitNo = 0;
   tGDEF_UINT8 i      = 0; 
   tGDEF_UINT8 j      = 0;
   tsALG_ENABLE_SENSOR * pSnsEnable = AINT_GetSensorEnableFlags();
   tsALG_SENSOR_TLM *pSnsTlm = &(ttcState.tlm);

#if _FC_NUM_GPS != 0
   const tsGPS_STATE   invalidGpsTlm    = AINT_INVALID_GPS_STATE;
   tGDEF_UINT8 ActiveGps;
#endif

#if _FC_NUM_SCAM != 0
   tGDEF_UINT8 measNo;
#endif

   /* reset sas flags */
   for (unitNo = 0; unitNo < _FC_NUM_SAS; unitNo++)
   {
      pSnsTlm->sas.valid[unitNo] = 0;
      if(GDEF_FALSE == pSnsEnable->sas[unitNo] && !ttcState.mode.isUnsolicited)
      {
         for (i = 0; i < _FC_NUM_SAS_AXIS; i++)
         {
            for (j = 0; j < _FC_NUM_SAS_CHANS; j++)
            {  // needs to match length in ATTC_EwodDefs.h which is 2 byte
               pSnsTlm->sas.tlm[unitNo][i][j] = AINT_INVALID_16BIT_TLM;
            }
         }
      }
   }


#if _FC_NUM_FSS != 0
   /* reset fss flags */
   for (unitNo = 0; unitNo < _FC_NUM_FSS; unitNo++)
   {
      pSnsTlm->fss[unitNo].valid = 0;
      if(GDEF_FALSE == pSnsEnable->fss[unitNo])
      {  // needs to match length in ATTC_EwodDefs.h which is 2 byte
         pSnsTlm->fss[unitNo].data[AZ_DATA] = AINT_INVALID_32BIT_TLM;
         pSnsTlm->fss[unitNo].data[EL_DATA] = AINT_INVALID_32BIT_TLM;
      }
   }


#endif

   /* reset mag flag */
   for (unitNo = 0; unitNo < _FC_NUM_MTM; unitNo++)
   {
      pSnsTlm->mag.valid[unitNo] = 0;
      if(GDEF_FALSE == pSnsEnable->mtm[unitNo] && !ttcState.mode.isUnsolicited)
      {
         for (i = 0; i < NUM_AXIS; i++)
         {  // needs to match length in ATTC_EwodDefs.h which is 2 byte
            pSnsTlm->mag.tlm[unitNo][i] = AINT_INVALID_16BIT_TLM;
         }
      }
   }

#if _FC_NUM_SCAM != 0
   /* reset star camera flag */
   for (unitNo = 0; unitNo < _FC_NUM_SCAM; unitNo++)
   {
      for (measNo = 0; measNo < _FC_NUM_SCAM_MEAS; measNo++)
      {
         pSnsTlm->starcam.valid[unitNo][measNo] = 0;
         if(GDEF_FALSE == pSnsEnable->scam[unitNo] && !ttcState.mode.isUnsolicited )
         {
            pSnsTlm->starcam.tSecs[unitNo][measNo] = AINT_INVALID_32BIT_TLM;
            pSnsTlm->starcam.tFrac[unitNo][measNo] = AINT_INVALID_16BIT_TLM;
            for (i = 0; i < _FC_NUM_Q_VALS; i++)
            {
               pSnsTlm->starcam.qVal[unitNo][measNo][i] = AINT_INVALID_32BIT_TLM;
            }
            pSnsTlm->starcam.chu_status[unitNo][measNo] = 0;
            pSnsTlm->starcam.msmt_flags[unitNo][measNo] = 0;
         }
      }
   }
#endif

   /* reset microsat wheel flags */
   for (unitNo = 0; unitNo < _FC_NUM_MWHL; unitNo++)
   {
      pSnsTlm->wheel.valid[unitNo] = 0;
      if(GDEF_FALSE == pSnsEnable->whl[unitNo] && !ttcState.mode.isUnsolicited)
      {  // needs to match length in ATTC_EwodDefs.h which is 4 byte
         pSnsTlm->wheel.tlm[unitNo] = AINT_INVALID_32BIT_TLM;
      }
   }

#if _FC_NUM_SWHL != 0
   /* reset microsat wheel flags */
   for (unitNo = 0; unitNo < _FC_NUM_SWHL; unitNo++)
   {
      pSnsTlm->swheel[unitNo].valid = 0;
      if(GDEF_FALSE == pSnsEnable->swhl[unitNo] && !ttcState.mode.isUnsolicited)
      {  // needs to match length in ATTC_EwodDefs.h which is 4 byte
         pSnsTlm->swheel[unitNo].data  = AINT_INVALID_32BIT_TLM;
      }
   }
#endif



#if _FC_NUM_GPS != 0

   /* reset gps flag */
   pSnsTlm->gps.valid = 0;
   ActiveGps = 0;
   for (unitNo = 0; unitNo < _FC_NUM_GPS; unitNo++)
   {
      if(GDEF_TRUE == pSnsEnable->gps[unitNo] )
      {
         ActiveGps = 1; // at least one gps is on
      }
   }

   if (ActiveGps == 0 && !ttcState.mode.isUnsolicited)
   {
      pSnsTlm->gps = invalidGpsTlm;
      pSnsTlm->gps.valid = 0;
   }
#endif


}




void AINT_SetModeReqState(teGDEF_BOOLEAN isAllowed)
{
   AINT_isModeReqAllowed = isAllowed;
}

teGDEF_BOOLEAN AINT_GetModeReqState(void)
{
   return AINT_isModeReqAllowed;
}


void AINT_SetTestState(tGDEF_UINT16 value)
{
   if (value != 0)
   {
      /* turn on only if have the correct on value */
      if (value == AINT_TEST_STATE_ON)
         ttcState.test.isEnabled = GDEF_TRUE;
   }
   else
      ttcState.test.isEnabled = GDEF_FALSE;
}


ts_UNIX_TIME AINT_GetUnixTime(void)
{
   struct timespec tm = {0};
   ts_UNIX_TIME    ut = {0};

   clock_gettime(CLOCK_REALTIME,&tm);

   ut.sec  = tm.tv_sec;
   ut.msec = tm.tv_nsec / 1000000;
   return (ut);
}

void AINT_SetSampleRate(tGDEF_UINT8 rate)
{
   ttcState.dt_nom = rate;
}


/* Set Failed wheel - id = wheel no +1 (0 = no failed wheel) */
void AINT_SetFailedWheel(tGDEF_UINT8 whlId)
{
   if (whlId < (_FC_NUM_MWHL +1))      
      ttcState.algConfig.wheel.failedWheel = whlId;
   
   /*
    *  re-initialise mapping for the current mode
    */
   ttcState.algConfig.MappingComplete = OFF;
#ifdef _SOLICITED_
   HMGR_ReqInitSample(); // ensure sampling is updated to reflect the changes
#endif

}

/* Set Failed aim - id = aim no +1 (0 = no failed aim) */
void AINT_SetFailedAIM(tGDEF_UINT8 aimId)
{
   if (aimId < (_FC_NUM_AIM +1))
      ttcState.algConfig.mag.failedAIM = aimId;
   
   if (aimId == 1)
      ttcState.algConfig.mag.mtmSelected = 1;
   else
      ttcState.algConfig.mag.mtmSelected = 0;
   
   /*
    *  re-initialise mapping for the current mode
    */
   ttcState.algConfig.MappingComplete = OFF;
#ifdef _SOLICITED_
   HMGR_ReqInitSample(); // ensure sampling is updated to reflect the changes
#endif

}
