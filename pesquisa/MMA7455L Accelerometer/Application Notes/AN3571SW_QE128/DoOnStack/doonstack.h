/******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2006 Freescale Semiconductor, Inc.
* (c) Copyright 2001-2004 Motorola, Inc.
* ALL RIGHTS RESERVED.
*
***************************************************************************//*!
*
* @file      doonstack.h
*
* @author    B01119
* 
* @version   1.0.1.0
* 
* @date      May-2-2007
* 
* @brief     C header file for DoOnStack.asm
*
*******************************************************************************
*
* 
*
******************************************************************************/

#ifndef _DOONSTACK_H
#define _DOONSTACK_H

#ifdef __cplusplus
	extern "C" { /* our assembly functions have C calling convention */
#endif

void DoOnStack(void);
void FlashErase1(unsigned char *);
void FlashProg1(unsigned char *, unsigned char);
unsigned char FlashProgBurst(unsigned char *, unsigned char *, unsigned char);
// this definitions has been on this place only for  back compatibility
#define CopyRam2Flash (void)FlashProgBurst
#define CopyRam2FlashBurst CopyRam2Flash

#ifdef __cplusplus
	}
#endif
  
#endif /* _DOONSTACK_H */
