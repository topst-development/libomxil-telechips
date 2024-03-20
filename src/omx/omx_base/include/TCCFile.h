/****************************************************************************
 *   FileName    : TCCFile.h
 *   Description : 
 ****************************************************************************
 *
 *   TCC Version 1.0
 *   Copyright (c) Telechips Inc.
 *   All rights reserved 
 
This source code contains confidential information of Telechips.
Any unauthorized use without a written permission of Telechips including not limited to re-distribution in source or binary form is strictly prohibited.
This source code is provided ¡°AS IS¡± and nothing contained in this source code shall constitute any express or implied warranty of any kind, including without limitation, any warranty of merchantability, fitness for a particular purpose or non-infringement of any patent, copyright or other third party intellectual property right. No warranty is made, express or implied, regarding the information¡¯s accuracy, completeness, or performance. 
In no event shall Telechips be liable for any claim, damages or other liability arising from, out of or in connection with this source code or the use in the source code. 
This source code is provided subject to the terms of a Mutual Non-Disclosure Agreement between Telechips and Company.
*
****************************************************************************/

/******************************************************************************
*
*  (C)Copyright All Rights Reserved by Telechips Inc.
*                                   
*  This material is confidential and shall remain as such. 
*  Any unauthorized use, distribution, reproduction is strictly prohibited. 
*
*   FileName    : TCCFile.h
*   Description : 
*   TCC Version 1.0
*   Copyright (c) Telechips, Inc.
*   ALL RIGHTS RESERVED
*******************************************************************************/
#ifndef	TCC_FILE_H__
#define	TCC_FILE_H__

/******************************************************************************
* include 
******************************************************************************/
#include <stdio.h>
#include <stdlib.h>

/******************************************************************************
* typedefs & structure
******************************************************************************/


/******************************************************************************
* defines 
******************************************************************************/

#define	TCC_LINUX_FILE_SYTEM

//#define	TCC_FILE_DEBUG	// for Telechips File Systme Debugging 



#ifdef TCC_LINUX_FILE_SYTEM
#define	TCC_FILE			FILE					// File Handler

#define	TCC_SEEK_SET		SEEK_SET				// File Seek 
#define	TCC_SEEK_CUR		SEEK_CUR			//
#define	TCC_SEEK_END		SEEK_END			//

#define	TCC_FILE_OK			0
#define	TCC_FILE_ERROR		-1					
#define	TCC_FILE_EOF		EOF
#else

#endif



/******************************************************************************
* globals
******************************************************************************/

/******************************************************************************
* locals
******************************************************************************/


/******************************************************************************
* declarations
******************************************************************************/
TCC_FILE *TCC_fopen(char *path,char *mode);
int TCC_fclose(TCC_FILE *stream);
int TCC_fseek(TCC_FILE *stream, long offset, int whence);
long  TCC_ftell(TCC_FILE *stream);
void  TCC_rewind(TCC_FILE *stream);
int TCC_feof(TCC_FILE *stream);
int TCC_fgetc(TCC_FILE *stream);
char *TCC_fgets(char *s, int size, TCC_FILE *stream);
int TCC_fputs(char *s, TCC_FILE *stream);
int TCC_fread(char *s, int size_t, int size, TCC_FILE *stream);
int TCC_fwrite(char *s, int size_t, int size, TCC_FILE *stream);
void TCC_fStartMeasure(void);
unsigned long long TCC_fGetMeasure(unsigned long long *min, unsigned long long *max);
#endif //TCC_FILE_H__
