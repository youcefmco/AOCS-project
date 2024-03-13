/******************************************************************************
 * Project        Spain DMC
 * Subsystem      OBC Flight Code - AOCS Task
 * Author         Allon Jameson
 * Date           12/07/07
 ******************************************************************************/
/*!****************************************************************************
 * \file AFIH_FileHandler.h
 *   header file for AOCS Task File Handler
 *
 ******************************************************************************/
/******************************************************************************
 * Copyright (c) 2007 Surrey Satellite Technology, Ltd.
 * All rights reserved.
 *
 ******************************************************************************
 * Documentation : N/A
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
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/include/AFIH_Filehandler.h,v $
 * Revision    : $Revision: 1.1 $
 *
 * History:
 *
 * $Log: AFIH_Filehandler.h,v $
 * Revision 1.1  2013/04/11 13:42:58  ytrichakis
 * Initial Commit
 *
 * Revision 1.3  2012/08/10 09:10:44  ajameson
 * updated mode config id parsing. Now expects format "XXRRMMMM" where XX is used to identify the file on the syste (e.g. use AC) RR is the revison number and MMMM is the mode id (ie mode number). No restrictions have been placed on the XX or RR.
 *
 * Revision 1.2  2012/06/18 13:39:53  SFingerloos
 * include globdefs
 *
 * Revision 1.1  2012/02/21 15:22:27  ajameson
 * initial check in
 *
 * Revision 1.1  2012/02/09 17:49:41  ajameson
 * code updated following integration of shell lib and stubbed code
 *
 * Revision 1.1  2012/02/09 14:33:14  ajameson
 * initial check in of AOCS shell library functions
 *
 * Revision 1.1  2009/07/16 17:32:07  ajameson
 * initial check in from Sapphire based on N2 AOCS task (branch B_SAPP_AOCS tag PP_AOCSTASK_R0_3)
 *
 * Revision 1.1.6.2  2009/06/22 08:07:33  ajameson
 * replaced apm file handling with RSO tfl file handling
 *
 * Revision 1.1.6.1  2009/06/22 07:35:34  ajameson
 * add to sapphire branch
 *
 * Revision 1.1  2009/06/10 08:06:18  ajameson
 * initial check in of file handler module
 *
 *
 ******************************************************************************/

#ifndef __AFIH_H
#define __AFIH_H

/*---------------------------------------------------------------------------
 * Includes
 */
#include "GDEF_GlobDefs.h"

/*---------------------------------------------------------------------------
 * Defines and Macros (shared within this module)
 */
#define AFIH_NO_DRIVE_FILE   (0UL)
#define MAX_NUM_STORED_FILES (20) /* move to mission.h */
#define FILE_INFO_SIZE       (20)
#define AFIH_NO_FILE         (0UL)

/*---------------------------------------------------------------------------
 * Typedefs (shared within this module)
 */
typedef struct 
{
   tGDEF_UINT32 id;
   tGDEF_UINT8  rev;
   tGDEF_UINT32 fnum;
}tsAFIH_REG_ENTRY;

/*---------------------------------------------------------------------------
 * Data - (shared within this module)
 */
extern tGDEF_UINT32 AFIH_driveFileNum;
 
/*---------------------------------------------------------------------------
 * Module Function Prototypes
 */
teGDEF_FUNC_STATUS AFIH_ProcUploadedFile(tGDEF_UINT32 fnum);
teGDEF_FUNC_STATUS AFIH_GetFnum(tGDEF_UINT32 id, tGDEF_UINT32 *fileNumber);
teGDEF_FUNC_STATUS AFIH_GetRev(tGDEF_UINT32 id, tGDEF_UINT32 *pRevNo);

#endif /* __AFIH_H */
