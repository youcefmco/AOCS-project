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
 * Last Update : $Date: 2015/11/17 14:22:06 $
 * CVS Source  : $Source: /OBDH/OBC750_VxWorks/OBC750-AOCS-Shell-RTP/include/ATTC_EwodDefs.h,v $
 * Revision    : $Revision: 1.4 $
 *
 * History:
 *
 * $Log: ATTC_EwodDefs.h,v $
 * Revision 1.4  2015/11/17 14:22:06  ytrichakis
 * Updated for new messgae handling + sked
 *
 * Revision 1.2  2013/09/02 09:20:34  ytrichakis
 * AOCS Version before completion review 4/9/13
 *
 * Revision 1.1  2013/04/11 13:42:58  ytrichakis
 * Initial Commit
 *
 ******************************************************************************/

#ifndef __EWOD_DEFS_H
#define __EWOD_DEFS_H

/*---------------------------------------------------------------------------
 * Includes
 */
#include "Adcs_ttcDefs.h"

/*---------------------------------------------------------------------------
 * Defines and Macros (shared within this module)
 */
//EWOD type masks (bit indicates the type number)
#define EWOD_NO_TYPES   0x00000000 /* type 0  = 0x00 */
#define EWOD_TYPE_SHELL 0x00000001 /* type 1  = 0x01 */
#define EWOD_TYPE_ALG   0x00000002 /* type 2  = 0x02 */
#define EWOD_TYPE_MAG   0x00000004 /* type 3  = 0x03 */
#define EWOD_TYPE_WHL   0x00000008 /* type 4  = 0x04 */
#define EWOD_TYPE_HPG   0x00000010 /* type 5  = 0x06 */
#define EWOD_TYPE_SAS   0x00000020 /* type 6  = 0x05 */
#define EWOD_TYPE_FSS   0x00000040 /* type 7  = 0x07 */
#define EWOD_TYPE_MGYR  0x00000080 /* type 8  = 0x08 */
#define EWOD_TYPE_SCAM  0x00000100 /* type 9  = 0x09 */
#define EWOD_TYPE_GPS   0x00000200 /* type 10 = 0x0A */
#define EWOD_TYPE_DEBUG 0x00000400 /* type 11 = 0x0B */


#define EWOD_TYPE_ALG_DBG 0x00000100

#define EWOD_MAX_EWOD_TYPE_NO (12) 

#define EWOD_SBM  EWOD_NO_TYPES
#define EWOD_DTM (EWOD_TYPE_SHELL | EWOD_TYPE_MAG | EWOD_TYPE_ALG)
#define EWOD_YTM (EWOD_TYPE_SHELL | EWOD_TYPE_MAG | EWOD_TYPE_ALG | EWOD_TYPE_WHL | EWOD_TYPE_SAS)
#define EWOD_CPM (EWOD_TYPE_SHELL | EWOD_TYPE_MAG | EWOD_TYPE_ALG | EWOD_TYPE_WHL | EWOD_TYPE_SAS)
#define EWOD_SAFE (EWOD_TYPE_SHELL | EWOD_TYPE_MAG | EWOD_TYPE_SAS)

#define EWOD_MODE_MASKS {EWOD_SBM, EWOD_DTM, EWOD_YTM, EWOD_CPM}

#define EWOD_INIT EWOD_SBM

#define FORMAT_UNDEF      (0)
#define MAX_NUM_WOD_CHANS (100)

/* YT Note:
 * Removed WHEEL4 as its noy used in 750 and it was a NULL pointer, replaced with another wheel 3
 * so size of table is still the same
 */

#define EWOD_TLM_TABLE \
{\
   /* ewod type         chan val          len  ewod  format id */\
   {EWOD_TYPE_SHELL, TLM_PROCTIME,         {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_SHELL, TLM_TIMERS_A,         {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_SHELL, TLM_TIMERS_B,         {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_SHELL, TLM_ARO,              {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_DEBUG, TLM_EWOD_MASK,        {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_SHELL, TLM_SHELL_STATUS,     {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_SHELL, TLM_HW_STATUS_EXT,    {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_SHELL, TLM_TIMERS_B,         {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_SHELL, TLM_FDIR_STATUS,      {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_SHELL, TLM_EXT_FDIR_STATUS,  {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_SHELL, TLM_ALG_STATUS,       {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_SHELL, TLM_DRIVE_FNUM,       {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_ALG,   TLM_POS_X,            {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_ALG,   TLM_POS_Y,            {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_ALG,   TLM_POS_Z,            {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_ALG,   TLM_VEL_X,            {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_ALG,   TLM_VEL_Y,            {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_ALG,   TLM_VEL_Z,            {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_ALG,   TLM_WX,               {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_ALG,   TLM_WY,               {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_ALG,   TLM_WZ,               {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_ALG,   TLM_ROLL,             {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_ALG,   TLM_PITCH,            {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_ALG,   TLM_YAW,              {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_ALG,   TLM_GUIDE_ROLL,       {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_ALG,   TLM_GUIDE_PITCH,      {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_ALG,   TLM_GUIDE_YAW,        {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_ALG,   TLM_DIPOLEX,          {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_ALG,   TLM_DIPOLEY,          {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_ALG,   TLM_DIPOLEZ,          {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_ALG,   TLM_MAG_OCXY,         {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_ALG,   TLM_MAG_OCZ,          {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_MAG,   TLM_MTQX,             {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_MAG,   TLM_MTQY,             {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_MAG,   TLM_MTQZ,             {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_DEBUG, TLM_MTQ0_AIM0,        {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_DEBUG, TLM_MTQ1_AIM0,        {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_DEBUG, TLM_MTQ2_AIM0,        {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_DEBUG, TLM_MTQ0_AIM1,        {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_DEBUG, TLM_MTQ1_AIM1,        {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_DEBUG, TLM_MTQ2_AIM1,        {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_MAG,   TLM_MTM0_A,           {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_MAG,   TLM_MTM0_B,           {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_MAG,   TLM_MTM0_C,           {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_MAG,   TLM_MTM1_A,           {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_MAG,   TLM_MTM1_B,           {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_MAG,   TLM_MTM1_C,           {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_SAS,   TLM_SAS0_AZ_A,        {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_SAS,   TLM_SAS0_AZ_B,        {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_SAS,   TLM_SAS0_EL_A,        {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_SAS,   TLM_SAS0_EL_B,        {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_SAS,   TLM_SAS1_AZ_A,        {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_SAS,   TLM_SAS1_AZ_B,        {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_SAS,   TLM_SAS1_EL_A,        {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_SAS,   TLM_SAS1_EL_B,        {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_SAS,   TLM_SAS2_AZ_A,        {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_SAS,   TLM_SAS2_AZ_B,        {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_SAS,   TLM_SAS2_EL_A,        {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_SAS,   TLM_SAS2_EL_B,        {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_SAS,   TLM_SAS_AZEL,         {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_DEBUG, TLM_MTM_X,            {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_DEBUG, TLM_MTM_Y,            {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_DEBUG, TLM_MTM_Z,            {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_WHL,   TLM_WHL0_MEAS_SPEED,        {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_WHL,   TLM_WHL1_MEAS_SPEED,        {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_WHL,   TLM_WHL2_MEAS_SPEED,        {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_WHL,   TLM_WHL3_MEAS_SPEED,        {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_WHL,   TLM_WHL4_MEAS_SPEED,        {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_WHL,  TLM_WHL0_CMD_SPEED,    {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_WHL,  TLM_WHL1_CMD_SPEED,    {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_WHL,  TLM_WHL2_CMD_SPEED,    {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_WHL,  TLM_WHL3_CMD_SPEED,    {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_WHL,  TLM_WHL4_CMD_SPEED,    {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_FSS,  TLM_FSS0_X1X2,    {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_FSS,  TLM_FSS0_Y1Y2,    {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_FSS,  TLM_FSS1_X1X2,    {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_FSS,  TLM_FSS1_Y1Y2,    {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_HPG,  TLM_GYR0_RATE_X,    {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_HPG,  TLM_GYR0_RATE_Y,    {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_HPG,  TLM_GYR0_RATE_Z,    {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_HPG,  TLM_GYR0_TEMP_X,   {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_HPG,  TLM_GYR0_TEMP_Y,   {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_HPG,  TLM_GYR0_TEMP_Z,   {4, FORMAT_UNDEF}}, \
   \
   {EWOD_TYPE_MGYR,  TLM_GYR1_RATE_XY,    {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_MGYR,  TLM_GYR1_RATE_ZT,    {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_MGYR,  TLM_GYR2_RATE_XY,    {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_MGYR,  TLM_GYR2_RATE_ZT,    {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_MGYR,  TLM_GYR3_RATE_XY,    {4, FORMAT_UNDEF}}, \
   {EWOD_TYPE_MGYR,  TLM_GYR3_RATE_ZT,    {4, FORMAT_UNDEF}}, \
}
/*---------------------------------------------------------------------------
 * Typedefs (shared within this module)
 */

/*---------------------------------------------------------------------------
 * Data - (shared within this module)
 */
 
/*---------------------------------------------------------------------------
 * Module Function Prototypes
 */









#endif /* _EWOD_DEFS_H */

