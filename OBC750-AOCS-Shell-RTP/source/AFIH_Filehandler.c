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
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/source/AFIH_Filehandler.c,v $
 * Revision    : $Revision: 1.6 $
 *
 * History:
 *
 * $Log: AFIH_Filehandler.c,v $
 * Revision 1.6  2016/03/14 10:47:46  ytrichakis
 * Register with the SKED library
 *
 * Revision 1.3  2013/06/24 13:13:24  ytrichakis
 * Removed TODO's as agreed between YT and AJ (07/06/13)
 *
 * Revision 1.2  2013/04/17 14:17:40  ytrichakis
 * Check in with fixing the identation only
 *
 * Revision 1.1  2013/04/11 13:42:21  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "GDEF_GlobDefs.h"
#include "Mission.h"
#include "Adcs_mission.h"

#include "pfh.h"

#include "ACON_AocsConfig.h"
#include "ACON_Sked.h"
//#include "EWOD_EwodHandler_OBC750.h"
#include "EWOD_EwodHandler.h"
#include "AMOH_AocsModeHandler.h"
#include "AINT_AdcsInterface.h"
#include "AFIH_FileHandler.h"

/*-----------------------------------------------------------------------
 * Defines and Macros
 */

/*---------------------------------------------------------------------------
 * Typedefs
 */

/*---------------------------------------------------------------------------
 * Private Function Prototypes (declared as static; used only within this module)
 */
static teGDEF_FUNC_STATUS ProcFileHeader(tGDEF_UINT32 fnum, tGDEF_UINT8 * pType, tGDEF_CHAR *pInfo);
static teGDEF_FUNC_STATUS ProcFile(tGDEF_CHAR * pInfo, tGDEF_UINT32 fnum, tGDEF_UINT8 type);
static teGDEF_FUNC_STATUS AFIH_RegisterFile(tGDEF_UINT32 id, tGDEF_UINT32 fileNumber);

/*---------------------------------------------------------------------------
 * Local Data (declared as static; used only within this module)
 */
static tsAFIH_REG_ENTRY AFIH_fileReg[MAX_NUM_STORED_FILES] = {{0}};
static tGDEF_UINT16     AFIH_regHead                       = 0;
static tGDEF_UINT16     AFIH_regSize                       = 0;

/*---------------------------------------------------------------------------
 * Global Data
 */
tGDEF_UINT32 AFIH_driveFileNum = AFIH_NO_FILE;


/*!
 * \brief      Process Uploaded File
 *
 *             Process a file uploaded to the filesystem that AOCS task should handle.
 *          It extracts relevant information from the pacsat file header and then forwards the
 *          file on appropriately.
 *
 * \param  fnum File number of uploaded file
 * \return Function Status
 *
 ******************************************************************************/
teGDEF_FUNC_STATUS AFIH_ProcUploadedFile(tGDEF_UINT32 fnum)
{
   tGDEF_CHAR  sInfo[FILE_INFO_SIZE] = ""; /* message title obtained from pacsat header */
   tGDEF_UINT8 fType                 = 0;

   if(GDEF_SUCCESS != ProcFileHeader(fnum, &fType, sInfo))
   {
      strcpy(EWOD_sLogMsg, "AFIH_ProcUploadedFile Error - fnum ");
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", fnum);
      EWOD_WriteMessage (EWOD_sLogMsg);

      return GDEF_FAILURE;
   }

   /* process fileType */
   return ProcFile(sInfo, fnum, fType);

}


/*!
 * \brief      Process File Header
 *
 *             Extracts information from the pacsat file header.
 *               Data is obtained from the message title.
 *
 * \param  fnum File number of uploaded file
 * \param  pType pointer to the file type
 * \param  pInfo pointer to the file data
 * \return Function Status
 *
 ******************************************************************************/
teGDEF_FUNC_STATUS ProcFileHeader(tGDEF_UINT32 fnum, tGDEF_UINT8 * pType, tGDEF_CHAR *pInfo)
{
   FILE         *fp   = NULL;
   tGDEF_UINT16 len   = 0;
   tGDEF_INT16  err   = -1;
   tGDEF_UCHAR  *pHdr = NULL;
   tGDEF_INT8   p2[MAX_HEADER_CHARS] = {0};

   /* open file with filenumber */
   sprintf(p2,"%d",fnum); /* fopen points to \sdram:0 in the actual hardware */
   fp = fopen((char*)p2, "rb");
   if(fp == NULL)
   {
#ifdef REALTIME_DEBUG
      (void)printf("File not opened!\n");
#endif /* REALTIME_DEBUG */
      strcpy(EWOD_sLogMsg, "Unable to open file\n");
      EWOD_WriteMessage (EWOD_sLogMsg);
      return GDEF_FAILURE;
   }

   /* read the pacsat file header */
   pHdr = pfh_read_header(fp, (tGDEF_UINT16 *) &len);
   if(pHdr == NULL)
   {
#ifdef REALTIME_DEBUG
      (void)printf("Unable to open pfh!\n");
#endif /* REALTIME_DEBUG */
      strcpy(EWOD_sLogMsg, "Unable to open pfh\n");
      EWOD_WriteMessage (EWOD_sLogMsg);
      return GDEF_FAILURE;
   }

   /* read message title from pacsat header to get the message title & user filename */
   err = pfh_get(_PFH_TITLE , pHdr, FILE_INFO_SIZE, pInfo);
   if(err != 0)
   {
#ifdef REALTIME_DEBUG
      (void)printf("phf error\n");
#endif /* REALTIME_DEBUG */
      strcpy(EWOD_sLogMsg, "pfh_get title error ");
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", err);
      EWOD_WriteMessage (EWOD_sLogMsg);
      return GDEF_FAILURE;
   }

   /* get the file type from the pacsat header */
   err = pfh_get(_PFH_FILE_TYPE , pHdr, 1, pType);
   if(err != 0)
   {
#ifdef REALTIME_DEBUG
      (void)printf("error type\n");
#endif /* REALTIME_DEBUG */
      strcpy(EWOD_sLogMsg, "pfh_get ftype error ");
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", err);
      EWOD_WriteMessage (EWOD_sLogMsg);
      return GDEF_FAILURE;
   }

   /* close file and free header space */
   free(pHdr);
   fclose(fp);

   return GDEF_SUCCESS;
}


/*!
 * \brief      Process File
 *
 *             Processes a file depending on its filetype
 *
 * \param  pInfo pointer to file information
 * \param  fnum File number of uploaded file
 * \param  type file type
 * \return Function Status
 *
 ******************************************************************************/
teGDEF_FUNC_STATUS ProcFile(tGDEF_CHAR * pInfo, tGDEF_UINT32 fnum, tGDEF_UINT8 type)
{
   teGDEF_FUNC_STATUS retVal = GDEF_FAILURE;
   tGDEF_UINT32       id     = 0;


   /* log the MsgTitle */
   strcpy(EWOD_sLogMsg, "File title:  ");
   strncpy(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], pInfo, 8);
   EWOD_sLogMsg[21] = '\0';
   EWOD_WriteMessage(EWOD_sLogMsg);

   /* process file depending on file type */
   switch(type)
   {
   case AOCS_DRIVE_FTYPE:
   {
      /* load the drive file */
      retVal = AINT_LoadDrive(fnum);
   }
   break;

   case AOCS_CONFIG_FTYPE:
   {
      /* Get file id (note: top two bytes are not used) */
      id = strtoul(pInfo, NULL, 16);

      AFIH_RegisterFile(id, fnum);
   }
   break;

   default:
   {
      /* unknown file type */
      strcpy(EWOD_sLogMsg, "AFIH_ProcFile - unknown ftype: ");
      sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", type);
      EWOD_WriteMessage(EWOD_sLogMsg);
      retVal = GDEF_FAILURE;
   }
   }

   return retVal;

} /* END AFIH_ProcFile */






teGDEF_FUNC_STATUS AFIH_RegisterFile(tGDEF_UINT32 id, tGDEF_UINT32 fileNumber)
{
   tGDEF_UINT16 index = 0;
   tGDEF_UINT8  revNo = 0;

   revNo = (0x00FF0000 & id) >> 16; /* revision number is 2nd byte of id */
   id    = (0x0000FFFF & id);       /* mode id is the value to store     */

   /* check if the id has already been stored */
   for(index = 0; index < AFIH_regSize; index++)
   {
      /* overwrite file number if id already exist */
      if(AFIH_fileReg[index].id == id)
      {
         /* overwrite stored file number */
         AFIH_fileReg[index].fnum = fileNumber;
         AFIH_fileReg[index].rev  = revNo;

         strcpy(EWOD_sLogMsg, "AFIH_RegisterFile - Overwriting ConfigFile ID: ");
         sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", id);
         EWOD_WriteMessage(EWOD_sLogMsg);

         /* return as file registered */
         return GDEF_SUCCESS;
      }
   }

   /* as file not found, store values in array at next id */
   AFIH_fileReg[AFIH_regHead].fnum = fileNumber;
   AFIH_fileReg[AFIH_regHead].id   = id;
   AFIH_fileReg[AFIH_regHead].rev  = revNo;
   /* update head index */
   AFIH_regHead = (AFIH_regHead + 1) % MAX_NUM_STORED_FILES;

   /* update number of stored files */
   if(AFIH_regSize < MAX_NUM_STORED_FILES)
   {
      AFIH_regSize++;
   }

   strcpy(EWOD_sLogMsg, "AFIH_RegisterFile - Success register ConfigFile ID: ");
   sprintf(&EWOD_sLogMsg[strlen(EWOD_sLogMsg)], "%d", id);
   EWOD_WriteMessage(EWOD_sLogMsg);

   /* RETURN */
   return GDEF_SUCCESS;
}

/* Not needed */

teGDEF_FUNC_STATUS AFIH_DeregisterFile(tGDEF_UINT32 id, teGDEF_BOOLEAN removeFlag)
{
   tGDEF_UINT8        idExists     = 0;
   tGDEF_UINT16       index        = 0;
   tGDEF_UINT32       fnum         = 0;
   teGDEF_FUNC_STATUS returnCode   = GDEF_FAILURE;
   tGDEF_INT16        callStatus   = -1;       /* set  pessimistically */

   /* find the id */
   for(; index < AFIH_regSize; index++)
   {
      /* overwrite file number if id already exist */
      if(idExists == 0 && AFIH_fileReg[index].id == id)
      {
         /* flag the id exists */
         idExists = 1;

         /* reduce the number of stored files */
         AFIH_regSize--;

         /* update head index  */
         AFIH_regHead = AFIH_regSize;

         /* store fnum for file */
         fnum = AFIH_fileReg[index].fnum;

         /* log the removal */
         //sprintf(EWOD_sLogMsg, "AFIH_DeregisterFile - Deregistered (0x%X)\n", id);
         EWOD_WriteMessage(EWOD_sLogMsg);
      }


      /* once the id is found need to shift the array down */
      if(idExists)
      {
         AFIH_fileReg[index].id  = AFIH_fileReg[index+1].id;
         AFIH_fileReg[index].fnum = AFIH_fileReg[index+1].fnum;
      }
   }

   /* return success if id was found and removed */
   if(idExists)
   {
      if(removeFlag == GDEF_TRUE)
      {
         /* remove file */
         //callStatus = mfdir("", fnum, &dir);
         if (callStatus != 0)// _FNODIR_ERR) //&& dir.fnumber == fnum)
         {
            /* remove file and log action */
            //remove(dir.fname);

            strcpy(EWOD_sLogMsg, "AFIH_RemoveRegisteredFile -fnum (0x%lX) removed \n");//, fnum);
            EWOD_WriteMessage(EWOD_sLogMsg);
            returnCode = GDEF_SUCCESS;
         }
         else
         {
            strcpy(EWOD_sLogMsg, "Errorr - AFIH_RemoveRegisteredFile - unable to remove file (0x%lX)\n");//, fnum);
            EWOD_WriteMessage(EWOD_sLogMsg);
         }
      }
      else
      {
         returnCode = GDEF_SUCCESS;
      }
   }

   return returnCode;
}


teGDEF_FUNC_STATUS AFIH_GetFnum(tGDEF_UINT32 id, tGDEF_UINT32 *fileNumber)
{
   teGDEF_FUNC_STATUS returnCode = GDEF_FAILURE;
   //tGDEF_UINT16       index      = 0;

   /*lint --e(506) Constant value Boolean */
   assert(fileNumber != NULL);

	if (ACON_SKEDFilenumber[id] == 0)
	{
		returnCode = GDEF_FAILURE;
	}
	else
	{
		*fileNumber = ACON_SKEDFilenumber[id];
		returnCode = GDEF_SUCCESS;
	}
	
   return returnCode;
}

teGDEF_FUNC_STATUS AFIH_GetRev(tGDEF_UINT32 id, tGDEF_UINT32 *pRevNo)
{
   teGDEF_FUNC_STATUS returnCode = GDEF_FAILURE;
   tGDEF_UINT16       index      = 0;

   /*lint --e(506) Constant value Boolean */
   assert(pRevNo != NULL);

   /* find id from look-up table */
   for(index = 0; index < AFIH_regSize; index++)
   {
      if(AFIH_fileReg[index].id == id)
      {
         /* set output file number */
         *pRevNo = AFIH_fileReg[index].rev;

         /*Set the return code to indicate success */
         returnCode = GDEF_SUCCESS;
      } /* if(AFIH_fileReg[index].id == id) */
   } /* for(; index < AFIH_regSize; index++) */

   return returnCode;
}



tGDEF_UINT16 AFIH_GetRegSize(void)
{
   /* return the registry size */
   return AFIH_regSize;
}




