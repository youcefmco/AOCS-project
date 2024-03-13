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
 * Last Update : $Date: 2013/09/02 09:20:33 $
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/include/AINT_Tlm.h,v $
 * Revision    : $Revision: 1.2 $
 *
 * History:
 *
 * $Log: AINT_Tlm.h,v $
 * Revision 1.2  2013/09/02 09:20:33  ytrichakis
 * AOCS Version before completion review 4/9/13
 *
 * Revision 1.1  2013/04/11 13:42:58  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/
#ifndef _AINT_TLM_H_
#define _AINT_TLM_H_

/*---------------------------------------------------------------------------
 * Includes
 */

/*---------------------------------------------------------------------------
 * Typedefs (shared within this module)
 */
typedef struct
{
   tGDEF_UINT16 chan;
   void *       pData;
} tsAINT_TLM_DATA;

typedef struct
{
   tGDEF_UINT8 CheckID;
   tGDEF_UCHAR *CheckState;
} tsAINT_FDIR_CHECK_DATA;

typedef enum
{
   PACKED_OBS_RES        = 0,
   PACKED_SAS            = 1,
   PACKED_STATUS         = 2,
   PACKED_MTM0           = 3,
   PACKED_MTM1           = 4,
   PACKED_WHL_SPD_01     = 5,
   PACKED_WHL_SPD_23     = 6,
   PACKED_DIPOLE_XY      = 7,
   PACKED_GUIDE_RP       = 8,
   PACKED_STK_RP         = 9,
   PACKED_STKY_GYROW_Z   = 10,
   PACKED_FDIR_STATUS    = 11,  
   PACKED_FDIR_STATUS_1  = 12,
   PACKED_SWHL_SPD       = 13,
   PACKED_EST_RP         = 14,
   PACKED_EST_WXY        = 15,
   PACKED_EST_YAW_WZ     = 16,
   PACKED_GYR1_XY        = 17,
   PACKED_GYR1_ZT        = 18,
   PACKED_GYR2_XY        = 19,
   PACKED_GYR2_ZT        = 20,
   PACKED_GYR3_XY        = 21,
   PACKED_GYR3_ZT        = 22,
   PACKED_GYR0_XY        = 23,
   PACKED_GYR0_ZT        = 24,
   PACKED_MTQX           = 25,
   PACKED_MTQY           = 26,
   PACKED_MTQZ           = 27,
   PACKED_ATTYAW         = 28,
   PACKED_OBS_Z          = 29,
   PACKED_MTQ0_AIM0      = 30,
   PACKED_MTQ1_AIM0      = 31,
   PACKED_MTQ2_AIM0      = 32,
   PACKED_MTQ0_AIM1      = 33,
   PACKED_MTQ1_AIM1      = 34,
   PACKED_MTQ2_AIM1      = 35,


} teAINT_PACKED_TYPES;
/*---------------------------------------------------------------------------
 * Data - (shared within this module)
 */
 
/*---------------------------------------------------------------------------
 * Module Function Prototypes
 */
extern tsAOCS_TTC ttcState;

/*---------------------------------------------------------------------------
 * Defines and Macros (shared within this module)
 */
#define NUM_PACKED_TYPES (36)
/* look up table for algorithm tlm (outputs) */
#define AINT_ALG_TLM_LOOKUP \
{\
/* 0  */ {TLM_PROCTIME, &(ttcState.unixSecs)},   \
/* 1  */ {TLM_POS_X,  &(ttcState.algTlm.rv[0])}, \
/* 2  */ {TLM_POS_Y,   &(ttcState.algTlm.rv[1])},\
/* 3  */ {TLM_POS_Z,   &(ttcState.algTlm.rv[2])},\
/* 4  */ {TLM_VEL_X,  &(ttcState.algTlm.rv[3])}, \
/* 5  */ {TLM_VEL_Y,   &(ttcState.algTlm.rv[4])},\
/* 6  */ {TLM_VEL_Z,   &(ttcState.algTlm.rv[5])},\
\
/* 7  */ {TLM_ROLL,    &(ttcState.algTlm.euler_smooth[GDEF_X_AXIS])},\
/* 8  */ {TLM_PITCH,   &(ttcState.algTlm.euler_smooth[GDEF_Y_AXIS])},\
/* 9  */ {TLM_YAW,     &(ttcState.algTlm.euler_smooth[GDEF_Z_AXIS])},\
\
/* 10  */ {TLM_WX,    &(ttcState.algTlm.omega_smooth[GDEF_X_AXIS])},\
/* 11  */ {TLM_WY,    &(ttcState.algTlm.omega_smooth[GDEF_Y_AXIS])},\
/* 12  */ {TLM_WZ,    &(ttcState.algTlm.omega_smooth[GDEF_Z_AXIS])},\
\
   /* 7  */ {TLM_EST_RP,    &AINT_packedTlm[PACKED_EST_RP]},\
   /* 8  */ {TLM_EST_WXY,   &AINT_packedTlm[PACKED_EST_WXY]},\
   /* 9  */ {TLM_EST_YAW_WZ,     &AINT_packedTlm[PACKED_EST_YAW_WZ]},\
\
/* 13 */ {TLM_GUIDE_ROLL,   &(ttcState.algTlm.rpy_guide[GDEF_X_AXIS])},\
/* 14 */ {TLM_GUIDE_PITCH,  &(ttcState.algTlm.rpy_guide[GDEF_Y_AXIS])},\
/* 15 */ {TLM_GUIDE_YAW,    &(ttcState.algTlm.rpy_guide[GDEF_Z_AXIS])},\
/* 16 */ {TLM_GUIDE_RP,   &AINT_packedTlm[PACKED_GUIDE_RP]},\
/* 17 */ {TLM_BACKGROUND_ATT, &AINT_packedTlm[PACKED_ATTYAW]},\
\
/* 16 */ {TLM_DIPOLEX,    &(ttcState.algTlm.dipole[GDEF_X_AXIS])},\
/* 17 */ {TLM_DIPOLEY,    &(ttcState.algTlm.dipole[GDEF_Y_AXIS])},\
/* 18 */ {TLM_DIPOLEZ,    &(ttcState.algTlm.dipole[GDEF_Z_AXIS])},\
/* 19 */ {TLM_DIPOLE_XY,    &AINT_packedTlm[PACKED_DIPOLE_XY]},\
\
/* 20 */ {TLM_MAG_OCXY,  &AINT_packedTlm[PACKED_OBS_RES]},\
/* 21 */ {TLM_MAG_OCZ,  &AINT_packedTlm[PACKED_OBS_Z]},\
\
/* 22 */ {TLM_ALG_STATUS, &AINT_packedTlm[PACKED_STATUS]},\
/* 23 */ {TLM_FDIR_STATUS, &AINT_packedTlm[PACKED_FDIR_STATUS]},\
/* 24 */ {TLM_SAS_AZEL,   &AINT_packedTlm[PACKED_SAS]},\
\
/* 27 */ {TLM_MTM_X, &ttcState.algTlm.magObs[GDEF_X_AXIS]},\
/* 28 */ {TLM_MTM_Y, &ttcState.algTlm.magObs[GDEF_Y_AXIS]},\
/* 29 */ {TLM_MTM_Z, &ttcState.algTlm.magObs[GDEF_Z_AXIS]},\
\
/* 30 */ {TLM_EXT_FDIR_STATUS, &AINT_packedTlm[PACKED_FDIR_STATUS_1]},\
} 

#define AINT_ALG_CMD_LOOKUP \
{\
/* 0 */ {TLM_MTQX,               &AINT_packedTlm[PACKED_MTQX]},\
/* 1 */ {TLM_MTQY,               &AINT_packedTlm[PACKED_MTQY]},\
/* 2 */ {TLM_MTQZ,               &AINT_packedTlm[PACKED_MTQZ]},\
\
/* 3  */ {TLM_MTQ0_AIM0,         &AINT_packedTlm[PACKED_MTQ0_AIM0]},\
/* 4  */ {TLM_MTQ1_AIM0,         &AINT_packedTlm[PACKED_MTQ1_AIM0]},\
/* 5  */ {TLM_MTQ2_AIM0,         &AINT_packedTlm[PACKED_MTQ2_AIM0]},\
\
/* 6  */ {TLM_MTQ0_AIM1,         &AINT_packedTlm[PACKED_MTQ0_AIM1]},\
/* 7  */ {TLM_MTQ1_AIM1,         &AINT_packedTlm[PACKED_MTQ1_AIM1]},\
/* 8  */ {TLM_MTQ2_AIM1,         &AINT_packedTlm[PACKED_MTQ2_AIM1]},\
\
/* 9  */ {TLM_WHL0_CMD_SPEED,    &(ttcState.algCmd.wheel[WHEEL0])},\
/* 10 */ {TLM_WHL1_CMD_SPEED,    &(ttcState.algCmd.wheel[WHEEL1])},\
/* 11 */ {TLM_WHL2_CMD_SPEED,    &(ttcState.algCmd.wheel[WHEEL2])},\
/* 12 */ {TLM_WHL3_CMD_SPEED,    &(ttcState.algCmd.wheel[WHEEL3])},\
\
}

#define AINT_SENSOR_TLM_LOOKUP \
{\
/* 0 */  {TLM_MTM0_A,  &(ttcState.tlm.mag.tlm[MTM0][GDEF_X_AXIS])},\
/* 1 */  {TLM_MTM0_B,  &(ttcState.tlm.mag.tlm[MTM0][GDEF_Y_AXIS])},\
/* 2 */  {TLM_MTM0_C,  &(ttcState.tlm.mag.tlm[MTM0][GDEF_Z_AXIS])},\
/* 3 */  {TLM_MTM1_A,  &(ttcState.tlm.mag.tlm[MTM1][GDEF_X_AXIS])},\
/* 4 */  {TLM_MTM1_B,  &(ttcState.tlm.mag.tlm[MTM1][GDEF_Y_AXIS])},\
/* 5 */  {TLM_MTM1_C,  &(ttcState.tlm.mag.tlm[MTM1][GDEF_Z_AXIS])},\
\
/* 6 */  {TLM_SAS0_AZ_A, &(ttcState.tlm.sas.tlm[SAS0][AZ_DATA][CHAN_A])},\
/* 7 */  {TLM_SAS0_AZ_B, &(ttcState.tlm.sas.tlm[SAS0][AZ_DATA][CHAN_B])},\
/* 8 */  {TLM_SAS0_EL_A, &(ttcState.tlm.sas.tlm[SAS0][EL_DATA][CHAN_A])},\
/* 9 */  {TLM_SAS0_EL_B, &(ttcState.tlm.sas.tlm[SAS0][EL_DATA][CHAN_B])},\
\
/* 10 */ {TLM_SAS1_AZ_A, &(ttcState.tlm.sas.tlm[SAS1][AZ_DATA][CHAN_A])},\
/* 11 */ {TLM_SAS1_AZ_B, &(ttcState.tlm.sas.tlm[SAS1][AZ_DATA][CHAN_B])},\
/* 12 */ {TLM_SAS1_EL_A, &(ttcState.tlm.sas.tlm[SAS1][EL_DATA][CHAN_A])},\
/* 13 */ {TLM_SAS1_EL_B, &(ttcState.tlm.sas.tlm[SAS1][EL_DATA][CHAN_B])},\
\
/* 14 */ {TLM_SAS2_AZ_A, &(ttcState.tlm.sas.tlm[SAS2][AZ_DATA][CHAN_A])},\
/* 15 */ {TLM_SAS2_AZ_B, &(ttcState.tlm.sas.tlm[SAS2][AZ_DATA][CHAN_B])},\
/* 16 */ {TLM_SAS2_EL_A, &(ttcState.tlm.sas.tlm[SAS2][EL_DATA][CHAN_A])},\
/* 17 */ {TLM_SAS2_EL_B, &(ttcState.tlm.sas.tlm[SAS2][EL_DATA][CHAN_B])},\
\
/* 18 */ {TLM_WHL0_MEAS_SPEED, &(ttcState.tlm.wheel.tlm[WHEEL0])},\
/* 19 */ {TLM_WHL1_MEAS_SPEED, &(ttcState.tlm.wheel.tlm[WHEEL1])},\
/* 20 */ {TLM_WHL2_MEAS_SPEED, &(ttcState.tlm.wheel.tlm[WHEEL2])},\
/* 21 */ {TLM_WHL3_MEAS_SPEED, &(ttcState.tlm.wheel.tlm[WHEEL3])},\
\
/* 22 */ {TLM_MTM0, &AINT_packedTlm[PACKED_MTM0]}, \
/* 23 */ {TLM_MTM1, &AINT_packedTlm[PACKED_MTM1]},\
/* 24 */ {TLM_WHL01_SPEED, &AINT_packedTlm[PACKED_WHL_SPD_01]},\
/* 25 */ {TLM_WHL23_SPEED,&AINT_packedTlm[PACKED_WHL_SPD_23]},\
\
/* 31 */ {TLM_WHL4_SPEED, &AINT_packedTlm[PACKED_SWHL_SPD]},\
\
/* 38 */ {TLM_GYR1_RATE_XY, &AINT_packedTlm[PACKED_GYR1_XY]},\
/* 39 */ {TLM_GYR1_RATE_ZT, &AINT_packedTlm[PACKED_GYR1_ZT]},\
/* 40 */ {TLM_GYR2_RATE_XY, &AINT_packedTlm[PACKED_GYR2_XY]},\
/* 41 */ {TLM_GYR2_RATE_ZT, &AINT_packedTlm[PACKED_GYR2_ZT]},\
/* 42 */ {TLM_GYR3_RATE_XY, &AINT_packedTlm[PACKED_GYR3_XY]},\
/* 43 */ {TLM_GYR3_RATE_ZT, &AINT_packedTlm[PACKED_GYR3_ZT]},\
/* 44 */ {TLM_GYR0_RATE_XY, &AINT_packedTlm[PACKED_GYR0_XY]},\
/* 45 */ {TLM_GYR0_RATE_ZT, &AINT_packedTlm[PACKED_GYR0_ZT]},\
}

/* CheckID,   flag*/
#define AINT_FDIR_ENABLE_TABLE \
{\
   {0,   &ttcState.algConfig.fdir.Enable_NavMagTmValidCheck},\
   {1,   &ttcState.algConfig.fdir.Enable_NavMagMeasurementValidCheck},\
   {2,   &ttcState.algConfig.fdir.Enable_SunSensorTmValidCheck},\
   {3,   &ttcState.algConfig.fdir.Enable_SunSensorMeasurementValidCheck},\
   {4,   &ttcState.algConfig.fdir.Enable_MRWTmValidCheck},\
   {5,   &ttcState.algConfig.fdir.Enable_MRWMeasurementValidCheck},\
   {6,   &ttcState.algConfig.fdir.Enable_MRWSpeedTrackValidCheck},\
   {7,   &ttcState.algConfig.fdir.Enable_MRWTorqueCmdValidCheck},\
   {8,   &ttcState.algConfig.fdir.Enable_MRWSpeedCmdValidCheck},\
   {9,   &ttcState.algConfig.fdir.Enable_SP100MeasurementValidCheck},\
   {10, &ttcState.algConfig.fdir.Enable_SP100SpeedTrackValidCheck},\
   {11,   &ttcState.algConfig.fdir.Enable_MagOCValidCheck},\
   {12,   &ttcState.algConfig.fdir.Enable_SunOCValidCheck},\
   {13,   &ttcState.algConfig.fdir.Enable_ControllerValidCheck},\
}


/* ADD IN IF stR TRACKER AND gps INCLUDED
   9,	, &ttcState.algConfig.fdir.Enable_STRTmValidCheck,\
   10,	 &ttcState.algConfig.fdir.Enable_STRTimestampValidCheck,\
   11,	 &ttcState.algConfig.fdir.Enable_STRQuaternionValidCheck,\
   12,	 &ttcState.algConfig.fdir.Enable_STRBlindingValidCheck,\
   13,	 &ttcState.algConfig.fdir.Enable_STRDPUValidFlagCheck,\
   14,	 &ttcState.algConfig.fdir.Enable_GPSTmValidCheck,\
   15,	 &ttcState.algConfig.fdir.Enable_GPSMeasurementRangeCheck,\
   16,	 &ttcState.algConfig.fdir.Enable_GPSMeasurementValidCheck,\
   17,	 &ttcState.algConfig.fdir.Enable_MagOCValidCheck,\
   18,	 &ttcState.algConfig.fdir.Enable_SunOCValidCheck,\
   19,	 &ttcState.algConfig.fdir.Enable_ControllerValidCheck,\
*/

#endif
