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
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/include/HINT_Wheel_10SP.h,v $
 * Revision    : $Revision: 1.1 $
 *
 * History:
 *
 * $Log: HINT_Wheel_10SP.h,v $
 * Revision 1.1  2013/04/11 13:42:59  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/

#ifndef __HINT_10SP_H
#define __HINT_10SP_H

/*---------------------------------------------------------------------------
 * Includes
 */
/* Microsat Wheel commands */
#define  MWHLCMD_ENABLE     (0)
#define  MWHLCMD_SPEED      (2)
#define  MWHLCMD_GAIN1      (3)
#define  MWHLCMD_GAIN2      (4)
#define  MWHLCMD_GAIN3      (5)
#define  MWHLCMD_RAMP       (6)
#define  MWHLCMD_SYNCH      (10)
#define  MWHLCMD_T_SAMPLE   (11)
#define  MWHLCMD_T_ACTUATE  (12)
#define  MWHLCMD_UNSOL_NODE (13)

#define WHEEL_SPD_OFFSET (1000000UL)

/* MIcrosat wheel telemetry */
#define MWHLTLM_SPEED (19)
#define WHL_ACTUATE_TIME 200

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

#endif /* __HINT_10SP_H */
