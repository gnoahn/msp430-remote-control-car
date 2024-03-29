/******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2008 Freescale Semiconductor, Inc.
* (c) Copyright 2001-2004 Motorola, Inc.
* ALL RIGHTS RESERVED.
*
* =================================================================== *
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY              *
* EXPRESSED OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE *
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR  *
* PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL FREESCALE OR             *
* ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,    *
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT        *
* NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;        *
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)            *
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, *
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)       *
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED *
* OF THE POSSIBILITY OF SUCH DAMAGE.                                  *
* =================================================================== *
*
***************************************************************************//*!
*
* @file      sci.h
*
* @author    R20246
* 
* @version   1.0.0.0
* 
* @date      MAR-6-2008
* 
* @brief     ION (MMA7455L)
*                           Ion MMA7455L using SPI or IIC Communication Protocol 
*                           in Measure mode, Level and Pulse Detection   
*
*******************************************************************************
*
* 
*
******************************************************************************/

#ifndef SCI_H
#define SCI_H

#include "derivative.h"

// FUNCTION DECLARATION
void SCI_Init(void);
void SCI_TxByte(unsigned char byte);

#endif //SCI_H 
