;******************************************************************************
;
; Freescale Semiconductor Inc.
; (c) Copyright 2004-2006 Freescale Semiconductor, Inc.
; (c) Copyright 2001-2004 Motorola, Inc.
; ALL RIGHTS RESERVED.
;
;**************************************************************************//*!
;
;   $File Name: doonstack.asm$
;
;   $Last Modified By: B01119$
;
;   $Version: 1.0.2.0$
;
;   $Date: May-2-2007$ 
;
;   @brief     DoOnStack.asm (see HCS08RM for details)
;
;******************************************************************************
;
; 
;
;*****************************************************************************/
; export symbols

; #pragma MESSAGE DISABLE A13003    ; Warning A13003: Value is truncated to one byte
                                 
            XDEF DoOnStack
            XDEF FlashErase1
            XDEF FlashProg1
            XDEF FlashProgBurst
            
            ; we use export 'Entry' as symbol. This allows us to
            ; reference 'Entry' either in the linker .prm file
            ; or from C/C++ later on
                       
; include derivative specific macros
            Include 'doonstack.inc'

; variable/data section
MY_ZEROPAGE: SECTION  SHORT
; Insert here your data definition. For demonstration, temp_byte is used.
;temp_byte ds.b 1

; code section
MyCode:     SECTION
;**************************************************************
; this assembly routine is called the C/C++ application
DoOnStack: 	pshx
			pshh ;save pointer to flash
			psha ;save command on stack
			ldhx #SpSubEnd ;point at last byte to move to stack;
SpMoveLoop: lda ,x ;read from flash
			psha ;move onto stack
			aix #-1 ;next byte to move
			cphx #SpSub-1 ;past end?
			bne SpMoveLoop ;loop till whole sub on stack
			tsx ;point to sub on stack
			tpa ;move CCR to A for testing
			and #$08 ;check the I mask
			bne I_set ;skip if I already set
			sei ;block interrupts while FLASH busy
			lda SpSubSize+6,sp ;preload data for command
			jsr ,x ;execute the sub on the stack
			cli ;ok to clear I mask now
			bra I_cont ;continue to stack de-allocation
I_set: 		lda SpSubSize+6,sp ;preload data for command
			jsr ,x ;execute the sub on the stack
I_cont: 	ais #SpSubSize+3 ;deallocate sub body + H:X + command ;H:X flash pointer OK from SpSub
			lsla ;A=00 & Z=1 unless PVIOL or ACCERR
			rts ;to flash where DoOnStack was called		
;**************************************************************
SpSub: 		ldhx LOW(SpSubSize+4),sp ;get flash address from stack
			sta 0,x ;write to flash; latch addr and data
			lda SpSubSize+3,sp ;get flash command
			sta FCMD ;write the flash command
			lda #mFCBEF ;mask to initiate command
			sta FSTAT ;[pwpp] register command
			nop ;[p] want min 4~ from w cycle to r
ChkDone: 	lda FSTAT ;[prpp] so FCCF is valid
			lsla ;FCCF now in MSB
			bpl ChkDone ;loop if FCCF = 0
SpSubEnd: 	rts ;back into DoOnStack in flash
SpSubSize: 	equ (*-SpSub)
;**************************************************************
FlashErase1:psha ;adjust sp for DoOnStack entry
			lda #(mFPVIOL+mFACCERR) ;mask
			sta FSTAT ;abort any command and clear errors
			lda #mPageErase ;mask pattern for page erase command
			bsr DoOnStack ;finish command from stack-based sub
			ais #1 ;deallocate data location from stack
			rts
;**************************************************************
FlashProg1: psha ;temporarily save entry data
			lda #(mFPVIOL+mFACCERR) ;mask
			sta FSTAT ;abort any command and clear errors
			lda #mByteProg ;mask pattern for byte prog command
			bsr DoOnStack ;execute prog code from stack RAM
			ais #1 ;deallocate data location from stack
			rts
;**************************************************************
SpSubBurst:	lda   	FSTAT 			;check FCBEF
    	and   	#mFCBEF 		;mask it
    	beq	  	SpSubBurst  ;loop if not empty
      ldhx LOW(SpSubSizeBurst+4),sp ;get source address from stack
			lda 0,x ; load source data byte
			aix #1  ; increment source address
			sthx (SpSubSizeBurst+4),sp ;save new source address to stack
			ldhx LOW(SpSubSizeBurst+8),sp ;get destionation address from stack
			sta 0,x  ;write to flash Latch
			aix #1 ; increment destination address
			sthx (SpSubSizeBurst+8),sp ;save new destionation address to stack
			lda #mBurstProg ; load Burst program command			
			sta FCMD ;write the flash command
			lda #mFCBEF ;mask to initiate command
			sta FSTAT ;[pwpp] register command			
			nop ;[p] want min 4~ from w cycle to r	.	
		  lda FSTAT ; load FSTAT to check ERRORs
			and #$30 ; check only FPVIOL and FACCERR
			beq FlashWriteOk
      lda #255 ;set up error flag 
      rts ;back into FlashProgBurst in flash
FlashWriteOk:	
      dbnz  SpSubSizeBurst+3,sp,SpSubBurst 
ChkDoneBurst: 	lda FSTAT ;[prpp] so FCCF is valid
			lsla ;FCCF now in MSB
			bpl ChkDoneBurst ;loop if FCCF = 0
			clra			
SpSubEndBurst: 	rts ;back into DoOnStack in flash
SpSubSizeBurst: 	equ (*-SpSubBurst)
;**************************************************************
FlashProgBurst: pshx ; save source address - low byte
      pshh ; save source address - high byte
      psha ;save length of data
      ; STACK - Dest. adrres 2-bytes, ret address 2- bytes, source address 2 - bytes, length 1 byte
			lda #(mFPVIOL+mFACCERR) ;mask
			sta FSTAT ;abort any command and clear errors	
			ldhx #SpSubEndBurst ;point at last byte to move to stack;
SpMoveLoopBurst: lda ,x ;read from flash
			psha ;move onto stack
			aix #-1 ;next byte to move
			cphx #SpSubBurst-1 ;past end?
			bne SpMoveLoopBurst ;loop till whole sub on stack	
			tsx ;point to sub on stack	
			tpa ;move CCR to A for testing
			and #$08 ;check the I mask
			bne I_setBurst ;skip if I already set
			sei ;block interrupts while FLASH busy
			jsr ,x ;execute the sub on the stack
			cli ;ok to clear I mask now
			bra I_contBurst ;continue to stack de-allocation
I_setBurst: jsr ,x ;execute the sub on the stack
I_contBurst: 	ais #SpSubSizeBurst+3 ;deallocate sub body + H:X + command ;H:X flash pointer OK from SpSub
			rts ;to flash where DoOnStack was called
;**************************************************************