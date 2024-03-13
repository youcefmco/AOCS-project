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
 * Last Update : $Date: 2014/09/15 12:56:50 $
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/include/AINT_AdcsInterface.h,v $
 * Revision    : $Revision: 1.3 $
 *
 * History:
 *
 * $Log: AINT_AdcsInterface.h,v $
 * Revision 1.3  2014/09/15 12:56:50  ytrichakis
 * Changes for TDS-1 Flight to include accepting drive file from automation process and removing sending sync cmds to AIM when in SBM
 *
 * Revision 1.2  2013/07/18 15:56:52  ytrichakis
 * fixed wheel bug(DR#14548) and applied the same solution in case of failed AIM
 *
 * Revision 1.1  2013/04/11 13:42:58  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/
#ifndef __AINT_H_
#define __AINT_H_


/*---------------------------------------------------------------------------
 * Includes
 */
#include <stdio.h>
#include "GDEF_GlobDefs.h"
#include "Adcs_IntDefs.h"

/*---------------------------------------------------------------------------
 * Defines and Macros (shared within this module)
 */
#define MAX_HEADER_CHARS (256)
#define FILENAME_SIZE    (255)

/*---------------------------------------------------------------------------
 * Typedefs (shared within this module)
 */
typedef struct
{
   tGDEF_UINT32 sec;  /*!< Seconds since 00:00 01-Jan-1970 */
   tGDEF_UINT16 msec; /*!< millisecond of time */
} ts_UNIX_TIME;

/*---------------------------------------------------------------------------
 * Data - (shared within this module)
 */
extern tsAOCS_TTC            * const AINT_pTtc;
extern tsALG_CONFIG          * const AINT_pAlgConfig;
extern tsALG_ATTITUDE_DEMAND * const AINT_pAttitude;
extern tsALG_SENSOR_TLM      * const AINT_pSensorTlm;
extern tsALG_TCMD            * const AINT_pAlgCmd;
extern tsALG_TLM             * const AINT_pAlgTlm;
extern tsALG_FDIR_STATE      * const AINT_pAlgFdir;
extern tGDEF_UINT8           * const AINT_pWheelFailNo;
 
/*---------------------------------------------------------------------------
 * Module Function Prototypes
 */
teGDEF_FUNC_STATUS      AINT_LoadDrive (tGDEF_UINT32 fnum);
teGDEF_FUNC_STATUS      AINT_AlgManager(void);
void 			        AINT_Init(teGDEF_BOOLEAN ResetAllFlag);
void                    AINT_SetModeReqState(teGDEF_BOOLEAN isAllowed);
teGDEF_BOOLEAN          AINT_GetModeReqState(void);
tsAOCS_TTC *            AINT_GetTtcState(void);
tsALG_CONFIG *          AINT_GetAlgConfig(void);
tsALG_ATTITUDE_DEMAND * AINT_GetAttitudeDemand(void);
tsALG_SENSOR_TLM *      AINT_GetSensorTlm(void);
tsALG_TCMD *            AINT_GetAlgCmds(void);
tsALG_TLM *             AINT_GetAlgTlm(void);
tsALG_MODE *            AINT_GetMode(void);
tsALG_ENABLE_SENSOR *   AINT_GetSensorEnableFlags(void);
tsALG_ENABLE_ACTUATOR * AINT_GetActuatorEnableFlags(void);
teGDEF_FUNC_STATUS      AINT_SetActuatorEnableState(teHMGR_HW_ACTUATORS actuator,tGDEF_UINT16 modeNo, tGDEF_UINT8 actNo, teGDEF_BOOLEAN state);
teGDEF_FUNC_STATUS      AINT_SetSensorEnableState(teHMGR_HW_SENSORS sensor,tGDEF_UINT16 modeNo, tGDEF_UINT8 snsNo, teGDEF_BOOLEAN state);
teGDEF_FUNC_STATUS      AINT_SetEnableStateFromDatabase(tGDEF_UCHAR ModeType);
teGDEF_FUNC_STATUS      AINT_SetFDIREnableFlags(tGDEF_UINT8 CheckID, tGDEF_UCHAR CheckState);
/*solicited requests for enable status in aint_logMode
 before requesting the packed act/sns flags set the aint_logmode */
tGDEF_UINT32            AINT_GetPackedActuatorEnableFlagsArray(void);
tGDEF_UINT32            AINT_GetPackedSensorEnableFlagsArray(void);
void                    AINT_Set_LogMode(tGDEF_UINT8 mode, teGDEF_BOOLEAN safeMode);
void                    AINT_SetDataInvalid(void);
void                    AINT_SetSampleRate(tGDEF_UINT8 rate);
void                    AINT_SetAlgTlm(tGDEF_UINT32 * pAlgTlm);
void                    AINT_SetTestState(tGDEF_UINT16 value);
tGDEF_UINT32 *          AINT_GetAlgTlmData(tGDEF_UINT16 chan);
tGDEF_UINT32 *          AINT_GetAlgCmdTlmData(tGDEF_UINT16 chan);
tGDEF_UINT32 *          AINT_GetSensorTlmData(tGDEF_UINT16 chan);
void                    SetAlgDelay(tGDEF_UINT8 mode,  tGDEF_INT16 ticks);
void                    AINT_SetFailedWheel(tGDEF_UINT8 whlId);
void                    AINT_SetFailedAIM(tGDEF_UINT8 aimId);

#endif
