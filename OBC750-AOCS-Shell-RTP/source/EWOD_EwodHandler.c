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
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/source/EWOD_EwodHandler.c,v $
 * Revision    : $Revision: 1.11 $
 *
 * History:
 *
 * $Log: EWOD_EwodHandler.c,v $
 * Revision 1.11  2016/03/17 15:57:36  ytrichakis
 * Create FSM thread before CAN and check for EEROR in dispatcher queue
 *
 * Revision 1.7  2013/09/02 09:20:34  ytrichakis
 * AOCS Version before completion review 4/9/13
 *
 * Revision 1.6  2013/07/18 15:56:53  ytrichakis
 * fixed wheel bug(DR#14548) and applied the same solution in case of failed AIM
 *
 * Revision 1.5  2013/06/24 13:13:25  ytrichakis
 * Removed TODO's as agreed between YT and AJ (07/06/13)
 *
 * Revision 1.4  2013/06/21 13:54:39  ytrichakis
 * Send command 5 only to non-failed AIMs. Also set ewod closing tie to 1 day
 *
 * Revision 1.3  2013/04/23 15:49:22  ytrichakis
 * Added block wait till SKED msgQ is ready to open
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
#include <time.h>

#include "GDEF_GlobDefs.h"
#include "mission.h"
#include "Adcs_mission.h"
#include "endian_transposition.h"

#include "pfh.h"

#include "ACON_AocsConfig.h"
#include "AOCS_AocsShell.h"
#include "AOCS_AocsFSM.h"
#include "AMOH_AocsModeHandler.h"
#include "AROH_AroHandler.h"
#include "ATTC_AocsTtc.h"
#include "ATTC_EwodDefs.h"
#include "EWOD_EwodHandler.h"


/*-----------------------------------------------------------------------
 * Defines and Macros
 */
#define REF_TIME          (631843200L)
#define SIZEOF_FNAME      (9)
#define MAX_DATA_SIZE 	  (4)
#define CHARS_PER_CHANNEL (4)
#define EWOD_CLOSE_TIME   (3600)
/*
 *Write record - Upper word is zero to indicate non WOD data
 *             - Lower word is record type
 *             - UnixMs is record length
 */
#define EWOD_CHAN_LIST_TYPE (0x00000001)
#define EWOD_STR_TYPE       (0x00000002)
#define EWOD_CMD_TYPE       (0x00000003)
#define EWOD_CHAN_DATA_SIZE (6)
#define EWOD_MAX_NUM_CHANS  (150)

#define INVALID_TLM 0x7fffffff

#define ZIP_MOVE_CMD (3)
#define ZIP_MSG_SIZE (5)

#define EWOD_PERIOD 86400L
#define EWOD_MAX_RATE (100)

#define EWOD_MAX_RATE_MULTIPLIER   (20)
#define EWOD_DEFAULT_SAMPLE_PERIOD (4)
#define EWOD_SIZEOF_SAMPLE         (6)


/*---------------------------------------------------------------------------
 * Typedefs
 */
/* EWOD File Header, without the channel list */
typedef struct
{
   tGDEF_UINT16 signature;
   tGDEF_UINT16 version;
   tGDEF_UINT8  filetype;
   tGDEF_UINT16 bodyOffset;
   tGDEF_CHAR   satName[12];
   tGDEF_UCHAR  recordType;
   tGDEF_CHAR   name[30];
   time_t       startTime;
   tGDEF_UINT16 startTimeX;
   time_t       endTime;
   tGDEF_UINT16 endTimeX;
   time_t       samplePeriod;
   tGDEF_UINT16 samplePeriodX;

} __attribute__((__packed__))tsEWOD_EXTENDED_HEADER;

/*---------------------------------------------------------------------------
 * Private Function Prototypes (declared as static; used only within this module)
 */
static teGDEF_FUNC_STATUS OpenEwod(void);
static void SetCloseTime(void);
static void WriteChanList(void);
static void WriteMessage(tGDEF_CHAR * pMsg);
static void UpdateEwodBuffer(void);
static void GetEwodFilename(tGDEF_CHAR * fname, time_t tNow);
static void SetHeader(tsEWOD_EXTENDED_HEADER *pHdr, tGDEF_UINT16 numChans, time_t tNow);
/* 
 * Keep them as library functionality even though they are not used 
 */
#if 0
static void EWOD_WriteCommand(tsEWOD_COMMAND_MSG cmdMsg);
static void EWOD_WriteRecord(void * recData, tGDEF_UINT16 recType, size_t len);
#endif
/*---------------------------------------------------------------------------
 * Local Data (declared as static; used only within this module)
 */
static tGDEF_UINT32       EWOD_modeMaskLookup[NUM_MODES]        = EWOD_MODE_MASKS;
static const tGDEF_UINT32 EWOD_defaultModeMaskLookup[NUM_MODES] = EWOD_MODE_MASKS;
/* table to map telemtry points to telemetry from algs */
static const tsEWOD_CHAN_LOOKUP EWOD_tlmTable[]             = EWOD_TLM_TABLE;
static const tGDEF_UINT16 EWOD_sizeofTlmTable               = sizeof(EWOD_tlmTable) / sizeof(tsEWOD_CHAN_LOOKUP);

static FILE *           EWOD_fp                             = NULL;
static tGDEF_CHAR       EWOD_fname[SIZEOF_FNAME]            = "";
static time_t           EWOD_closeTime                      = 0;
static tGDEF_INT16      EWOD_checkSum                       = 0;
static teGDEF_BOOLEAN   EWOD_isZip                          = GDEF_TRUE;
static time_t           EWOD_time                           = 0;
static time_t           EWOD_samplePeriod                   = EWOD_DEFAULT_SAMPLE_PERIOD;
static time_t           EWOD_samplePeriodLookup[NUM_MODES]  = AOCS_EWOD_RATE;
static tsEWOD_TLM       EWOD_tlmBuf[EWOD_MAX_NUM_CHANS]     = {{0}};
static tGDEF_UINT16     EWOD_numChans                       = 0;
static tGDEF_UINT32     EWOD_typeMask                       = EWOD_INIT;
static tGDEF_UINT32     EWOD_prevTypeMask                   = EWOD_INIT;
static const tGDEF_UINT32 cEWOD_InvalidData                 = 0x7FFFFFFF;

/*---------------------------------------------------------------------------
 * Global Data
 */
void ZipFile(tGDEF_CHAR * filename);
tGDEF_CHAR EWOD_sLogMsg[EWOD_MAX_LOGMSG_SIZE];

#if 0
/*!
 *  \brief Routine to transpose between Big Endian format and Little Endian format a signed 16 bit integer.
 *  \param  tGDEF_INT16 - Signed 16 bit integer in Big Endian format.
 *  \return tGDEF_INT16 - Signed 16 bit integer in Little Endian format.
 */

tGDEF_INT16 endian_transpose_signed_sixteen_bit_integer(tGDEF_INT16 value)
{
#ifndef VX_SIM
   return (value << 8) | ((value >> 8) & 0xFF);
#else
   return value;
#endif /* VX_SIM */
}

/*!
 *  \brief Routine to transpose between Big Endian format and Little Endian format a signed 32 bit integer.
 *  \param  tGDEF_INT32 - Signed 32 bit integer in Big Endian format.
 *  \return tGDEF_INT32 - Signed 32 bit integer in Little Endian format.
 */

tGDEF_INT32 endian_transpose_signed_thirty_two_bit_integer(tGDEF_INT32 value)
{
#ifndef VX_SIM
   value = ((value << 8) & 0xFF00FF00) | ((value >> 8) & 0xFF00FF );
   return (value << 16) | ((value >> 16) & 0xFFFF);
#else
   return value;
#endif /* VX_SIM */
}
#endif
/*!
 * \brief     WriteMessage
 *
 *              Writes a log message to the Ewod file
 *
 * \param pMsg pointer to the message
 *
 ******************************************************************************/
void WriteMessage(tGDEF_CHAR * pMsg)
{
   tGDEF_UINT32 hdrData 					  = 0;
   tGDEF_UINT16 len     					  = 0;
   tGDEF_CHAR   sLogMsg[EWOD_MAX_LOGMSG_SIZE] = {0};

   /* copy message to local buffer (ensures we have a string) */
   strncpy(sLogMsg, pMsg, EWOD_MAX_LOGMSG_SIZE);
   len = strlen(sLogMsg);

   if (EWOD_fp)
   {
      /*
       * Write record - Upper word is zero to indicate non WOD data
       *     	      - Lower word is record type
       *        	  - UnixMs is record length
       */
      hdrData = endian_transpose_signed_thirty_two_bit_integer(EWOD_STR_TYPE);   /* String message */
      pfhics_fwrite(&hdrData, 4, 1, EWOD_fp,(tGDEF_UINT16 *)&EWOD_checkSum);
      /* Length of string + time */
      hdrData = endian_transpose_signed_thirty_two_bit_integer(len + 5);   /* Add space for the NULL + time */
      pfhics_fwrite(&hdrData, 2, 1, EWOD_fp,(tGDEF_UINT16 *)&EWOD_checkSum);
      /* write time */
      hdrData = endian_transpose_signed_thirty_two_bit_integer(time(NULL));
      pfhics_fwrite(&hdrData, 4, 1, EWOD_fp,(tGDEF_UINT16 *)&EWOD_checkSum);
      /* write the string (including the null) */
      pfhics_fwrite(pMsg, len + 1, 1, EWOD_fp,(tGDEF_UINT16 *)&EWOD_checkSum);
   }
}



/*!
 * \brief     SetCloseTime
 *
 *              Calculates & sets the next time the EWOD should be closed
 *
 * \return time of closure
 *
 ******************************************************************************/
void SetCloseTime(void)
{
#if 0
   time_t diffTime;

   /* get number of hours (integer) + 1 since a reference time: 09/01/1990 00:00:00 */
   diffTime = ((time(NULL) - REF_TIME) / 3600L) + 1;

   /* set close time as time of next hour */
   EWOD_closeTime = (diffTime * 100L) + REF_TIME;
#endif
   EWOD_closeTime = time(NULL) + EWOD_CLOSE_TIME;
#ifdef REALTIME_DEBUG
   (void)printf("Close time set to %ld\n",EWOD_closeTime);
#endif /* REALTIME_DEBUG */

}


/*!
 * \brief     OpenEwod
 *
 *              Opens the current EWOD with a given wod type mask
 *
 * \param wodmask initial ewod type mask
 *
 ******************************************************************************/
teGDEF_FUNC_STATUS OpenEwod(void)
{
   time_t       tNow                   = {0};
   tGDEF_CHAR   newFname[SIZEOF_FNAME] = {0};
   tsEWOD_EXTENDED_HEADER header       = {0};

   /* check if ewod open */
   if (EWOD_fp != NULL)
   {
      return GDEF_SUCCESS;
   }

   /* get current time */
   tNow = time(NULL);

   /* get ewod filename for current time */
   GetEwodFilename(newFname, tNow);

   /*
    *  If today's name is different from current name, either we changed
    *  days or we just started the program.
    */
   if (strncmp(newFname, EWOD_fname, SIZEOF_FNAME) != 0)
   {
      /* There is an old file name, so update it. */
      if (EWOD_fname[0] != '\0')
      {
         EWOD_fp = pfhics_fopen(EWOD_fname, "a",(tGDEF_UINT16 *)&EWOD_checkSum);
         EWOD_CloseLog();

         /* check if need to zip file */
         /* if (EWOD_isZip)
             ZipFile(EWOD_fname);*/
      }

      /* update new ewod filename */
      strncpy(EWOD_fname, newFname, SIZEOF_FNAME);
   }

   /* attempt to open the file */
   EWOD_fp = pfhics_fopen(EWOD_fname, "a",(tGDEF_UINT16 *)&EWOD_checkSum);

   /* if file not opened, doesn't exist */
   if (EWOD_fp == NULL)
   {
      /* open a new file w/ PFH */
      EWOD_fp = pfh_init_file(EWOD_fname, "", AOCS_EWOD_FTYPE);
      /* zero the checksum */
      EWOD_checkSum = 0;

      /* check file successfully created */
      if (EWOD_fp != NULL)
      {
         /* Update the tlm buffer to ensure have right tlm data */
         UpdateEwodBuffer();

         /* set up eod header and write to file */
         SetHeader(&header, EWOD_numChans, tNow);
         pfhics_fwrite(&header, sizeof(header), 1, EWOD_fp,(tGDEF_UINT16 *)&EWOD_checkSum);

         /* WriteChannel list to file */
         WriteChanList();
         pfhics_update(EWOD_fp, POST, EWOD_checkSum);    /* update PFH header */
         fseek(EWOD_fp, 0L, SEEK_END);                   /* go to EOF.        */
      }
      else
      {
         /* unable to create file so return failure */
         return GDEF_FAILURE;
      }
   }

   /* update the EWOD close time */
   SetCloseTime();

   return GDEF_SUCCESS;
}


/*!
 * \brief     GetEwodFilename
 *
 *              Gets the current EWOD filename based on the date & time
 *
 * \param fname pointer to filename string
 * \param tNow current time
 *
 ******************************************************************************/
void GetEwodFilename(tGDEF_CHAR * fname, time_t tNow)
{
   struct tm *tp;

   /* conver the time to struct */                     /* get the time    		*/
   tp = gmtime(&tNow);                                 /* break down time 		*/
   strncpy(fname, LOGNAME, SIZEOF_FNAME);              /* ETyymmdd is file name */
   sprintf(fname + 2, "%02d", (tp->tm_year % 100));
   sprintf(fname + 4, "%02d", tp->tm_mon + 1);
   sprintf(fname + 6, "%02d", tp->tm_mday);
}


void UpdateEwodBuffer(void)
{
   tGDEF_UINT16 numChans = 0;
   tGDEF_UINT8  index    = 0;

   /* gather task data */
   for (index = 0; index < EWOD_sizeofTlmTable; index++ )
   {
      if ((EWOD_typeMask & EWOD_tlmTable[index].type) == EWOD_tlmTable[index].type)
      {
         /* set up data */
         EWOD_tlmBuf[numChans].chan = EWOD_tlmTable[index].chan;
         ATTC_TlmHandler(EWOD_tlmBuf[numChans].chan, &EWOD_tlmBuf[numChans].pData);

         /* copy the ewod info */
         EWOD_tlmBuf[numChans].ewod = EWOD_tlmTable[index].ewod;
         numChans++;
      }

      if (numChans >= EWOD_MAX_NUM_CHANS)
      {
         EWOD_WriteMessage("Update Ewod Buffer: number of logging channels exceeds buffer \n");
         break;
      }
   }

   /* store num of chans */
   EWOD_numChans = numChans;
#ifdef REALTIME_DEBUG
   (void)printf("no of channels: %d\n",EWOD_numChans);
#endif /* REALTIME_DEBUG */

}


/*!
 * \brief     SetHeader
 *
 *              Sets up the EWOD file header
 *
 * \param pHdr pointer to the header struct
 * \param numChans number of channels in channel list
 * \param tNow current time
 *
 ******************************************************************************/
void SetHeader(tsEWOD_EXTENDED_HEADER *pHdr, tGDEF_UINT16 numChans, time_t tNow)
{
   struct tm *tp;
   time_t next_day = 0;
   /* clear header */
   memset(pHdr, 0, sizeof(tsEWOD_EXTENDED_HEADER));
   /* convert current time to time struct and clear clock part */
   tp = gmtime(&tNow);
   tp->tm_hour = 0;
   tp->tm_min  = 0;
   tp->tm_sec  = 0;
   /* copy sat name */
   strncpy(pHdr->satName, SATNAME, 12);
   /* copy header file name */
   strncpy(pHdr->name, "ADCS Log", sizeof(pHdr->name));
   pHdr->signature        = 0x8134;/* CANTLM_SIGNATURE In big endian format for 750*/
   pHdr->version          = 0;
   pHdr->filetype         = CANTLM_WOD;
   pHdr->bodyOffset       = endian_transpose_signed_sixteen_bit_integer(sizeof(tsEWOD_EXTENDED_HEADER) + (EWOD_SIZEOF_SAMPLE * numChans));
   pHdr->recordType       = 1;
   pHdr->startTime        = endian_transpose_signed_thirty_two_bit_integer(mktime(tp));
   pHdr->startTimeX       = 0;
   next_day = mktime(tp) + EWOD_PERIOD;
   pHdr->endTime          = endian_transpose_signed_thirty_two_bit_integer(next_day);
   pHdr->endTimeX         = 0;
   pHdr->samplePeriod     = endian_transpose_signed_thirty_two_bit_integer(EWOD_samplePeriod);
   pHdr->samplePeriodX    = 0;
}

/* TODO Add zip file whenever ready to do so*/
/*!
 * \brief     ZipFile
 *
 *              Zips the EWOD file using the zip task
 *
 * \param filename filename of file to zip
 *
 ******************************************************************************/
void ZipFile(tGDEF_CHAR * filename)
{
   tGDEF_UCHAR    zipMsg[ZIP_MSG_SIZE];
   tGDEF_UINT32 * pMsgData;
   tGDEF_INT16    err = -1;
   //struct DIR_ENTRY dir = {{0},{0},0,0,0,0,0,0,0};
   /* get dir struct for given file name YT */
   /*err = mfdir(filename, 0L, &dir);*/

   /* send an intertask message to SCZIP to compress the dat */
   if (err != 0)// _FNODIR_ERR)
   {
      /* set zip command  - first byte in zip message*/
      zipMsg[0] = ZIP_MOVE_CMD;
      /* set file number to zip */
      pMsgData  = (tGDEF_UINT32 *) & (zipMsg[1]);
      //*pMsgData = dir.fnumber;
      /* write message to zip task YT*/
      /*write_hit(zipMsg, ZIP_MSG_SIZE, "SCZIP");*/
   }
}


/*!
 * \brief     EWOD_WriteChannelList
 *
 *              Updates the EWOD buffer and writes the channel list
 *
 * \param wodmask ewod type mask
 * \param isNewList boolean flag to indicate a new channel list
 *
 ******************************************************************************/
void EWOD_WriteChannelList(teGDEF_BOOLEAN isNewList)
{
   tGDEF_UINT16 numChans = 0;
   tGDEF_UINT32 wodData  = 0;

   /* check if have an open file */
   OpenEwod();

   if (EWOD_fp == NULL)
   {
      return;
   }

   /* gather task data */
   UpdateEwodBuffer();

   /* if a new list then write special time time stamp */
   if (isNewList)
   {
      wodData = endian_transpose_signed_thirty_two_bit_integer(EWOD_CHAN_LIST_TYPE);
      pfhics_fwrite( &wodData, 4, 1, EWOD_fp,(tGDEF_UINT16 *)&EWOD_checkSum);
      /* Length of new list */
      wodData = endian_transpose_signed_thirty_two_bit_integer(((tGDEF_UINT32) numChans * EWOD_CHAN_DATA_SIZE) + sizeof(numChans));
#ifdef REALTIME_DEBUG
      (void)printf("woddata %d\n",wodData);
#endif /* REALTIME_DEBUG */
      pfhics_fwrite(&wodData, 2, 1, EWOD_fp,(tGDEF_UINT16 *)&EWOD_checkSum);
   }

   /* write the channel list to the file */
   WriteChanList();
}


/*!
 * \brief     WriteChanList
 *
 *              Writes the current channel list to the EWOD file
 *
 *
 ******************************************************************************/
void WriteChanList(void)
{
   tGDEF_UINT16       entryNo 		  = 0;
   tGDEF_UINT16       flipped_channel = 0;
   const tGDEF_UINT16 node            = endian_transpose_signed_sixteen_bit_integer(CANADDR_ADCS_PROCESS);
   const tGDEF_UINT16 channels        = endian_transpose_signed_sixteen_bit_integer(EWOD_numChans);

   tGDEF_UINT8 buffer[2+(EWOD_MAX_NUM_CHANS*6)];
   tGDEF_UINT32 len = 0;
   
   /* write number of channels */
   //pfhics_fwrite(&channels, 2, 1, EWOD_fp,(tGDEF_UINT16 *)&EWOD_checkSum);
   memcpy(&buffer[len], &channels, 2);
   len += 2;
   
   for (entryNo = 0; entryNo < EWOD_numChans; entryNo++)
   {
	  //get the channel
	  flipped_channel = endian_transpose_signed_sixteen_bit_integer(EWOD_tlmBuf[entryNo].chan);
	   memcpy(&buffer[len], &node, 2);
	   len += 2;

	   memcpy(&buffer[len], &flipped_channel, 2);
	   len += 2;

	   memcpy(&buffer[len], &(EWOD_tlmBuf[entryNo].ewod.formatId), 1);
	   len += 1;
	   
	   memcpy(&buffer[len], &(EWOD_tlmBuf[entryNo].ewod.len), 1);
	   len += 1;
   }
   
   pfhics_fwrite(buffer, len, 1, EWOD_fp,(tGDEF_UINT16 *)&EWOD_checkSum);
}


/*!
 * \brief     EWOD_WriteData
 *
 *              Writes sampled data to the Ewod file
 *
 * \param wodmask ewod type mask
 * \param unix currrent unix time seconds
 * \param unixMs currrent unix time milliseconds
 *
 ******************************************************************************/
void EWOD_WriteData(time_t unix, tGDEF_UINT16 unixMs )
{
   tGDEF_UINT8  index          = 0;
   tGDEF_UINT32 local_variable = 0;
   time_t flipped_unix         = endian_transpose_signed_thirty_two_bit_integer(unix);
   time_t flipped_unixMs       = endian_transpose_signed_sixteen_bit_integer(unixMs);   
   tGDEF_UINT32 len = 0;
     
   tGDEF_UINT8 buffer[6+(4*EWOD_MAX_NUM_CHANS)];
   
   /* check if have an open file */
   OpenEwod();

   /* check if opened Ewod file suceesfully and have some data */
   if((EWOD_fp == NULL) || 	(EWOD_numChans == 0))		   
   {
	   //No EWOD file so return
	   return;
   }
      
 
   /* write time stamp - unix time*/
   memcpy(&buffer[len], &flipped_unix,   4);
   len +=4;

   /* write time stamp - unix time milliseconds*/
   memcpy(&buffer[len], &flipped_unixMs, 2);   
   len +=2; 
   
   /* Write the task data */
   for (index = 0; index < EWOD_numChans; index++ )
   {
      /* ensure only print 4 chars per channel */
      if(EWOD_tlmBuf[index].ewod.len > CHARS_PER_CHANNEL)
      {
         EWOD_tlmBuf[index].ewod.len = CHARS_PER_CHANNEL;
      }
		
      local_variable = endian_transpose_signed_thirty_two_bit_integer(*EWOD_tlmBuf[index].pData);
      memcpy(&buffer[len], &local_variable, EWOD_tlmBuf[index].ewod.len);

      //update size of data
      len += EWOD_tlmBuf[index].ewod.len;		
   }
   
   //write the data record to the EWOD file
   pfhics_fwrite(buffer, len, 1, EWOD_fp, (tGDEF_UINT16*)&EWOD_checkSum);
}


/*!
 * \brief     EWOD_WriteMessage
 *
 *              Ensure ewod file open then writes a log message to it
 *
 * \param pMsg pointer to the message
 *
 ******************************************************************************/
void EWOD_WriteMessage(tGDEF_CHAR * pMsg)
{
   /* check if have an open file */
   OpenEwod();

   /* write the message to file */
   if(EWOD_fp != NULL)
      WriteMessage(pMsg);
#ifdef REALTIME_DEBUG
   printf(pMsg);
#endif

}

#if 0
/*!
 * \brief     EWOD_WriteCommand
 *
 *              Writes a command message to the Ewod file
 *
 * \param cmdMessage command message data
 *
 ******************************************************************************/
void EWOD_WriteCommand(tsEWOD_COMMAND_MSG cmdMsg)
{
   tGDEF_UINT32 hdrData;

   /* check if have an open file */
   OpenEwod();

   if (EWOD_fp != NULL)
   {
      //Write record - Upper word is zero to indicate non WOD data
      //          - Lower word is record type
      //          - UnixMs is record length
      hdrData = endian_transpose_signed_thirty_two_bit_integer(EWOD_CMD_TYPE) ;  //String message
      pfhics_fwrite(&hdrData, 4, 1, EWOD_fp,(tGDEF_UINT16 *)&EWOD_checkSum);
      //Length of command + time
      hdrData = endian_transpose_signed_thirty_two_bit_integer(sizeof(cmdMsg) + 4); //cmd msg + time
      pfhics_fwrite(&hdrData, 2, 1, EWOD_fp,(tGDEF_UINT16 *)&EWOD_checkSum);
      //write time
      hdrData = endian_transpose_signed_thirty_two_bit_integer(time(NULL));
      pfhics_fwrite(&hdrData, 4, 1, EWOD_fp,(tGDEF_UINT16 *)&EWOD_checkSum);
      //write the command
      pfhics_fwrite(&cmdMsg, sizeof(cmdMsg), 1, EWOD_fp,(tGDEF_UINT16 *)&EWOD_checkSum);
   }
}

/*!
 * \brief     EWOD_WriteRecord
 *
 *              Writes a record to the Ewod file
 *
 * \param recData pointer to the record data
 * \param recType record type
 * \param len length of record
 *
 ******************************************************************************/

void EWOD_WriteRecord(void * recData, tGDEF_UINT16 recType, size_t len)
{
   tGDEF_UINT32 hdrData;

   /* check if have an open file */
   OpenEwod();

   if (EWOD_fp != NULL)
   {
      //Write record - Upper word is zero to indicate non WOD data
      //          - Lower word is record type
      //          - UnixMs is record length
      hdrData = endian_transpose_signed_thirty_two_bit_integer((tGDEF_UINT32) recType);
      pfhics_fwrite(&hdrData, 4, 1, EWOD_fp,(tGDEF_UINT16 *)&EWOD_checkSum);
      //Length of string + time
      hdrData = endian_transpose_signed_thirty_two_bit_integer(len + 4);   //Add space for the time
      pfhics_fwrite(&hdrData, 2, 1, EWOD_fp,(tGDEF_UINT16 *)&EWOD_checkSum);
      //write time
      hdrData = endian_transpose_signed_thirty_two_bit_integer(time(NULL));
      pfhics_fwrite(&hdrData, 4, 1, EWOD_fp,(tGDEF_UINT16 *)&EWOD_checkSum);
      //write the string
      pfhics_fwrite(recData, len, 1, EWOD_fp,(tGDEF_UINT16 *)&EWOD_checkSum);
   }
}
#endif
/*!
 * \brief     EWOD_CloseLog
 *
 *              Closes the EWOD file and updates the pfh
 *
 *
 ******************************************************************************/
void EWOD_CloseLog(void)
{
   /* Log the closure. */
   if (EWOD_fp != NULL)
   {
      WriteMessage("Closing EWOD\n");
      pfhics_update(EWOD_fp, POST, EWOD_checkSum);
      fclose(EWOD_fp);
      EWOD_fp = NULL;
   }

}



/*!
 * \brief     EWOD_InitMode
 *
 *              Initialises the EWOD logging for a new mode
 *
 * \param mode mode number
 *
 ******************************************************************************/
void EWOD_InitMode(tGDEF_UINT8 mode)
{

   if(mode < NUM_MODES)
   {
      EWOD_typeMask = EWOD_modeMaskLookup[mode];
   }

   /* log updated ewod flag */
   strcpy(EWOD_sLogMsg, "EWOD Flag updated\n");
   EWOD_WriteMessage(EWOD_sLogMsg);
}

/*!
 * \brief     EWOD_GetModeMask
 *
 *              Gets the ewod mask for the current mode
 *
 * \param mode mode number
 *
 ******************************************************************************/
tGDEF_UINT32 EWOD_GetModeMask()
{
   return EWOD_typeMask;
}


/*!
 * \brief     EWOD_AddTlmType
 *
 *              Sets the ewod mask for a given mode
 *
 * \param mode mode number
 *
 ******************************************************************************/
void EWOD_AddTlmType(tGDEF_UINT8 mode, tGDEF_UINT8 typeNo)
{

   if((typeNo == 0) || (typeNo > EWOD_MAX_EWOD_TYPE_NO))
   {
      return;
   }

   /* add the type to the mask */
   EWOD_modeMaskLookup[mode] |=  0x01UL<< (typeNo-1);

   /* log updated ewod mask */   
   sprintf(EWOD_sLogMsg, "EWOD mode [%d] mask updated to [%u]\n", mode, EWOD_modeMaskLookup[mode]);
   EWOD_WriteMessage(EWOD_sLogMsg);

   /* if the update was for current mode then update typeMask */
   if( mode == AMOH_GetMode())
   {
      EWOD_typeMask = EWOD_modeMaskLookup[mode];
   }
}


/*!
 * \brief     EWOD_RemoveTlmType
 *
 *              Sets the ewod mask for a given mode
 *
 * \param mode mode number
 *
 ******************************************************************************/
void EWOD_RemoveTlmType(tGDEF_UINT8 mode, tGDEF_UINT8 typeNo)
{

   if((typeNo == 0) || (typeNo > EWOD_MAX_EWOD_TYPE_NO))
   {
      return;
   }

   /* add the type to the mask */
   EWOD_modeMaskLookup[mode] &= ~( 0x01UL<< (typeNo-1));

   /* log updated ewod mask */
   sprintf(EWOD_sLogMsg, "EWOD tlm [%d] mask updated to [%u]\n", mode, EWOD_modeMaskLookup[mode]);
   EWOD_WriteMessage(EWOD_sLogMsg);

   /* if the update was for current mode then update typeMask */
   if( mode == AMOH_GetMode())
   {
      EWOD_typeMask = EWOD_modeMaskLookup[mode];
   }
}


/*!
 * \brief     EWOD_RemoveTlmType
 *
 *              Sets the ewod mask for a given mode
 *
 * \param mode mode number
 *
 ******************************************************************************/
void EWOD_SetDefaultTlmType(tGDEF_UINT8 mode)
{
   /* check if mode valid */
   if(mode > MAX_MODE_VAL)
   {
      return;
   }

   /* reset the mode mask to default value; */
   EWOD_modeMaskLookup[mode] = EWOD_defaultModeMaskLookup[mode];

   /* log updated ewod mask */
   sprintf(EWOD_sLogMsg, "EWOD tlm [%d] mask set to default [%u]\n", mode, EWOD_modeMaskLookup[mode]);
   EWOD_WriteMessage(EWOD_sLogMsg);

   /* if the update was for current mode then update typeMask */
   if( mode == AMOH_GetMode())
   {
      EWOD_typeMask = EWOD_modeMaskLookup[mode];
   }
}


/*!
 * \brief     EWOD_Update
 *
 *              Updates the EWOD file if ready
 *
 *
 ******************************************************************************/
void EWOD_Update(void)
{
   /* Close EWOD file if it's past time */
   if (AOCS_timeNow > EWOD_closeTime)
   {
      EWOD_CloseLog();
   }
   /* check if ewod needs to be updated */
   if (AOCS_timeNow >= EWOD_time)
   {
      /* Check if the channels have changed */
      if (EWOD_prevTypeMask != EWOD_typeMask)
      {
         EWOD_WriteChannelList(GDEF_TRUE);
         EWOD_prevTypeMask = EWOD_typeMask;
      }

      /* ensure shell tlm is up to date (AJ - needed??) */
      ATTC_Init();

      /* Update the AOCS telemetry structure */
      EWOD_WriteData(AOCS_timeNow, 0);
      /* update the time for ewod update */
      EWOD_time = AOCS_timeNow + EWOD_samplePeriod;
   }
}


#ifdef _DONT_USE_
/*!
 * \brief     EWOD_SetWodPeriod
 *
 *              Updates the wod sampling period based on a given cycle rate and the multiplier
 * \param rate cycle rate
 *
 ******************************************************************************/
void EWOD_SetWodPeriod(tGDEF_UINT8 rate, tGDEF_UINT8 type)
{
   /* calculate new wod period */
   EWOD_samplePeriod = (time_t) rate * EWOD_rateMultiplier[type];
}
#endif


void EWOD_SetWodPeriodArray(tGDEF_UINT8 mode,  tGDEF_UINT8 value)
{
   if(mode  >= NUM_MODES)
   {
      return;
   }

   EWOD_samplePeriodLookup[mode]= value;
   
   //Update ewod sampling if in current mode
   if(mode == AMOH_GetMode())
   {
	   EWOD_SetPeriod(mode);
   }

}


/*!
 * \brief     EWOD_SetPeriod
 *
 *              Updates the wod sampling period based on a given cycle rate and the multiplier
 * \param rate cycle rate
 *
 ******************************************************************************/
void EWOD_SetPeriod(tGDEF_UINT8 mode)
{
   /* calculate new wod period */
   EWOD_samplePeriod = EWOD_samplePeriodLookup[mode];
}



/*!
 * \brief     EWOD_ZipFile
 *
 *              Sets the flag to zip an EWOD after use
 *
 *
 ******************************************************************************/
void EWOD_ZipFile(teGDEF_BOOLEAN zipFlag)
{
   EWOD_isZip = zipFlag;
}

