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
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/include/ATTC_AocsTTC.h,v $
 * Revision    : $Revision: 1.1 $
 *
 * History:
 *
 * $Log: ATTC_AocsTTC.h,v $
 * Revision 1.1  2013/04/11 13:42:58  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/

#ifndef __ATTC_H_
#define __ATTC_H_

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

/*---------------------------------------------------------------------------
 * Data - (shared within this module)
 */
 
/*---------------------------------------------------------------------------
 * Module Function Prototypes
 */
teGDEF_FUNC_STATUS ATTC_ProcessCmd(tGDEF_UINT16 cmd, tGDEF_UINT32 value);
teGDEF_FUNC_STATUS ATTC_TlmHandler(tGDEF_UINT16 chan, tGDEF_UINT32 ** ppData);
void               ATTC_Init(void);

#endif 
/* __ATTC_H */

