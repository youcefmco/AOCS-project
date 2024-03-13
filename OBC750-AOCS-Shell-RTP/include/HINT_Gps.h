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
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/include/HINT_Gps.h,v $
 * Revision    : $Revision: 1.1 $
 *
 * History:
 *
 * $Log: HINT_Gps.h,v $
 * Revision 1.1  2013/04/11 13:42:58  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/

#ifndef _HINT_GPS_
#define _HINT_GPS_

/*---------------------------------------------------------------------------
 * Includes
 */
#include "Adcs_IntDefs.h"

/*---------------------------------------------------------------------------
 * Defines and Macros (shared within this module)
 */
#define BITS_IN_UINT32        (32)

/*---------------------------------------------------------------------------
 * Typedefs (shared within this module)
 */

/*---------------------------------------------------------------------------
 * Data - (shared within this module)
 */
 
/*---------------------------------------------------------------------------
 * Module Function Prototypes
 */
void               UTLM_GpsInitDataGathering(void);
void               UTLM_ProcGpsTlm(tGDEF_UINT16 chan, tGDEF_UCHAR * pData, teGDEF_BOOLEAN isNewFrame);
teGDEF_FUNC_STATUS UTLM_SetGpsData(tsGPS_STATE *pTtc, tGDEF_UCHAR isEnabled[]);

#endif
