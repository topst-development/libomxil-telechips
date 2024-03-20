/**
  @file src/omx_comp_debug_levels.h

  Define the level of debug prints on standard err. The different levels can 
  be composed with binary OR.
  The debug levels defined here belong to OpenMAX components and IL core

  Copyright (C) 2007  STMicroelectronics and Nokia

  This library is free software; you can redistribute it and/or modify it under
  the terms of the GNU Lesser General Public License as published by the Free
  Software Foundation; either version 2.1 of the License, or (at your option)
  any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
  details.

  You should have received a copy of the GNU Lesser General Public License
  along with this library; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St, Fifth Floor, Boston, MA
  02110-1301  USA

  $Date: 2008/09/10 08:04:37 $
  Revision $Rev: 239 $
  Author $Author: B060934 $

*/

#ifndef OMX_COMP_DEBUG_LEVELS_H__
#define OMX_COMP_DEBUG_LEVELS_H__

#include <stdio.h>
#include <glib/gprintf.h>

#define tcc_printf (void)g_printf

/** Remove all debug output lines
 */
#define DEB_LEV_NO_OUTPUT  0

/** Messages explaing the reason of critical errors 
 */
#define DEB_LEV_ERR        1
 
/** Messages showing values related to the test and the component/s used
 */
#define DEB_LEV_PARAMS     2

/** Messages representing steps in the execution. These are the simple messages, because 
 * they avoid iterations 
 */
#define DEB_LEV_SIMPLE_SEQ 4

/** Messages representing steps in the execution. All the steps are described, 
 * also with iterations. With this level of output the performances are 
 * seriously compromised
 */
#define DEB_LEV_FULL_SEQ   8

/** Messages that indicates the beginning and the end of a function.
 * It can be used to trace the execution
 */
#define DEB_LEV_FUNCTION_NAME 16

#define DEB_LEV_TIME	32
#define DEFAULT_MESSAGES	32

#define DEB_LEV_TCC		64
#define DEB_LEV_TCC_ERR	128

/** All the messages - max value
 */
#define DEB_ALL_MESS   255

/** \def DEBUG_LEVEL is the current level do debug output on standard err */
#define DEBUG_LEVEL DEB_LEV_ERR//(DEB_LEV_ERR|DEFAULT_MESSAGES)//DEB_LEV_NO_OUTPUT
#if DEBUG_LEVEL > 0
//#define DEBUG(n, args...) do { if (DEBUG_LEVEL & (n)){fprintf(stderr, "OMX-"); fprintf(stderr, args);} } while (0)
//#define DEBUG(n, args...) do { if (DEBUG_LEVEL & (n)){printf("$$$-%d ",__LINE__); printf(args);} } while (0)
#ifdef HAVE_ANDROID_OS
#define DEBUG(n, args...) do { if (DEBUG_LEVEL & (n)){ ALOGE(args);} } while (0)
#else
#define DEBUG(n, args...) 
#endif
#else
#define DEBUG(n, args...) 
#endif

/* ANSI Output Definitions */
#define T_DEFAULT		"\033""[0m"
#define TS_BOLD			"\x1b[1m"
#define TS_ITALIC		"\x1b[3m"
#define TS_UNDER		"\x1b[4m"
#define TS_UPSET		"\x1b[7m"
#define TS_LINE			"\x1b[9m"

#define NS_BOLD			"\x1b[22m"
#define NS_ITALIC		"\x1b[23m"
#define NS_UNDER		"\x1b[24m"
#define NS_UPSET		"\x1b[27m"
#define NS_LINE			"\x1b[29m"

#define TC_BLACK		"\033""[30m"
#define TC_RED			"\033""[31m"
#define TC_GREEN		"\033""[32m"
#define TC_BOLD			"\033""[1m"
#define TC_YELLOW		"\033""[33m"
#define TC_BLUE			"\033""[34m"
#define TC_MAGENTA		"\033""[35m"	//磊全
#define TC_CYAN			"\033""[36m"	//没废
#define TC_WHITE		"\033""[37m"
#define TC_RESET		"\033""[39m"

#define BC_BLACK		"\x1b[40m"
#define BC_RED			"\x1b[41m"
#define BC_GREEN		"\x1b[42m"
#define BC_YELLOW		"\x1b[43m"
#define BC_BLUE			"\x1b[44m"
#define BC_MAGENTA		"\x1b[45m"	//磊全
#define BC_CYAN			"\x1b[46m"	//没废
#define BC_WHITE		"\x1b[47m"
#define BC_RESET		"\x1b[49m"

#define STYLE_TITLE		"\x1b[1;4;37;40m"
#define STYLE_SUBTITLE	"\x1b[1;37;40m"
#define STYLE_LINE1		"\x1b[1;37;40m"
#define STYLE_LINE2		"\x1b[1;37;40m"
#define STYLE_STEPLINE	"\x1b[1;32;40m"
#define STYLE_RESET		T_DEFAULT

int gs_iDbgmsgEnable;
int gs_iDbgmsgLevel_Audio; //(0:None, 1:Error, 2:Warn, 3:Info, 4:Debug, 5:All)
int gs_iDbgmsgLevel_Adec;  //(0:None, 1:Error, 2:Warn, 3:Info, 4:Debug, 5:All)
int gs_iDbgmsgLevel_Vdec;  //(0:None, 1:Error, 2:Warn, 3:Info, 4:Debug, 5:All)

#if defined(OMX_DEBUG_MODE)
#define CHECK_DECLOG_ENABLE() {\
	char *env = getenv("CLOG_ENABLE");\
	if( env ) gs_iDbgmsgEnable = (int)(env[0] - '0');\
	else      gs_iDbgmsgEnable = 0;\
}

#define CHECK_AUDIOLOG_LEVEL() {\
	char *env = getenv("CLOG_AUDIO");\
	if( env ) gs_iDbgmsgLevel_Audio = (int)(env[0] - '0');\
	else      gs_iDbgmsgLevel_Audio = 3;\
}

#define CHECK_ADECLOG_LEVEL() {\
	char *env = getenv("CLOG_ADEC");\
	if( env ) gs_iDbgmsgLevel_Adec = (int)(env[0] - '0');\
	else      gs_iDbgmsgLevel_Adec = 3;\
}

#define CHECK_VDECLOG_LEVEL() {\
	char *env = getenv("CLOG_VDEC");\
	if( env ) gs_iDbgmsgLevel_Vdec = (int)(env[0] - '0');\
	else      gs_iDbgmsgLevel_Vdec = 3;\
}
#else
#define CHECK_DECLOG_ENABLE() gs_iDbgmsgEnable = 0;
#define CHECK_AUDIOLOG_LEVEL() gs_iDbgmsgLevel_Audio = 3;
#define CHECK_ADECLOG_LEVEL() gs_iDbgmsgLevel_Adec = 3;
#define CHECK_VDECLOG_LEVEL() gs_iDbgmsgLevel_Vdec = 3;
#endif

extern int gs_iDbgmsgEnable;
extern int gs_iDbgmsgLevel_Audio; //(0:None, 1:Error, 2:Warn, 3:Info, 4:Debug, 5:All)
extern int gs_iDbgmsgLevel_Adec;  //(0:None, 1:Error, 2:Warn, 3:Info, 4:Debug, 5:All)
extern int gs_iDbgmsgLevel_Vdec;  //(0:None, 1:Error, 2:Warn, 3:Info, 4:Debug, 5:All)

#endif   //OMX_COMP_DEBUG_LEVELS_H__
