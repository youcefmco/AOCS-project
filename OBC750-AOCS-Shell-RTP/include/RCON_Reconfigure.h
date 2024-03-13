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
 * Last Update : $Date: 2013/04/11 13:42:59 $
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/include/RCON_Reconfigure.h,v $
 * Revision    : $Revision: 1.1 $
 *
 * History:
 *
 * $Log: RCON_Reconfigure.h,v $
 * Revision 1.1  2013/04/11 13:42:59  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/
#ifndef __RCON_H_
#define __RCON_H_

/*---------------------------------------------------------------------------
 * Includes
 */
#include "GDEF_GlobDefs.h"

/*---------------------------------------------------------------------------
 * Defines and Macros (shared within this module)
 */

/*---------------------------------------------------------------------------
 * Typedefs (shared within this module)
 */
typedef enum
{
   RCON_NO_RECONFIG = 0,
   RCON_RECONFIG1 = 1,
   RCON_RECONFIG2 = 2,
   RCON_RECONFIG3 = 3,
} teAROH_RECONFIG_STATE;
#define RCON_MAX_RECONFIG_STATE  RCON_RECONFIG3 

/*---------------------------------------------------------------------------
 * Data - (shared within this module)
 */
extern tGDEF_UINT8 AROH_reconfigState;

/*---------------------------------------------------------------------------
 * Module Function Prototypes
 */
void        RCON_SetState(tGDEF_UINT8 value);
void        RCON_MathError();
tGDEF_UINT8 RCON_GetState();
tGDEF_UINT8 RCON_Reconfigure(tGDEF_UINT8 mode);

#endif /* _RCON_H */
