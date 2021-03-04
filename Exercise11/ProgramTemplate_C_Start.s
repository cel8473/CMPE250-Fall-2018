            TTL Program Title for Listing Header Goes Here
;****************************************************************
;Descriptive comment header goes here.
;(What does the program do?)
;Name:  <Your name here>
;Date:  <Date completed here>
;Class:  CMPE-250
;Section:  <Your lab section, day, and time here>
;---------------------------------------------------------------
;Keil Template for KL46 Assembly with Keil C startup
;R. W. Melton
;November 13, 2017
;****************************************************************
;Assembler directives
            THUMB
            GBLL  MIXED_ASM_C
MIXED_ASM_C SETL  {TRUE}
            OPT   64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL46Z4.s     ;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;EQUates
MAX_STRING	EQU		79			;Maximum number of string characters allowed
;---------------------------------------------------------------
;Characters
CR          EQU  	0x0D		;ASCII value for carriage return
LF          EQU  	0x0A		;ASCII value for line fill
NULL        EQU  	0x00		;ASCII value for NULL
GT			EQU	 	0x3E		;ASCII value for greater than
LT			EQU	 	0x3C		;ASCII value for less than
BS			EQU	 	0x08		;ASCII value for backspace
;Record Structure for Queue
IN_PTR		EQU		0		
OUT_PTR		EQU		4
BUF_STRT	EQU		8
BUF_PAST	EQU		12
BUF_SIZE	EQU		16
NUM_ENQD	EQU		17
;Queue structure sizes
Q_BUF_SZ	EQU		4
Q_REC_SZ	EQU		18
TX_Q_SZ		EQU		80
RX_Q_SZ		EQU		80
;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;12:UART0 IRQ mask
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;12:UART0 IRQ pending status
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;12:UART0 IRQ mask
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port A
PORT_PCR_SET_PTA1_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTA2_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->09:Port A clock gate control (enabled)
;Use provided SIM_SCGC5_PORTA_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select
;         (PLLFLLSEL determines MCGFLLCLK' or MCGPLLCLK/2)
; 1=   16:PLLFLLSEL=PLL/FLL clock select (MCGPLLCLK/2)
SIM_SOPT2_UART0SRC_MCGPLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
SIM_SOPT2_UART0_MCGPLLCLK_DIV2 EQU \
    (SIM_SOPT2_UART0SRC_MCGPLLCLK :OR: SIM_SOPT2_PLLFLLSEL_MASK)
;---------------------------------------------------------------
;SIM_SOPT5
; 0->   16:UART0 open drain enable (disabled)
; 0->   02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)
;---------------------------------------------------------------
;UART0_BDH
;    0->  7:LIN break detect IE (disabled)
;    0->  6:RxD input active edge IE (disabled)
;    0->  5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)]) 
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDH_9600  EQU  0x01
;---------------------------------------------------------------
;UART0_BDL
;0x38->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600  EQU  0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select 
;        (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1  EQU  0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:RIE=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
;           10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
;           10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
;            (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV  EQU  0x00
;---------------------------------------------------------------
;UART0_C4
;    0-->  7:MAEN1=match address mode enable 1 (disabled)
;    0-->  6:MAEN2=match address mode enable 2 (disabled)
;    0-->  5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
;               = 1 + OSR for 3 <= OSR <= 31
;               = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
;  0-->  7:TDMAE=transmitter DMA enable (disabled)
;  0-->  6:Reserved; read-only; always 0
;  0-->  5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
;  0-->  1:BOTHEDGE=both edge sampling (rising edge only)
;  0-->  0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
;---------------------------------------------------------------
;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS  EQU  (UART0_S1_IDLE_MASK :OR: \
                            UART0_S1_OR_MASK :OR: \
                            UART0_S1_NF_MASK :OR: \
                            UART0_S1_FE_MASK :OR: \
                            UART0_S1_PF_MASK)
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
;             write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
;              write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  \
        (UART0_S2_LBKDIF_MASK :OR: UART0_S2_RXEDGIF_MASK)
;---------------------------------------------------------------
;****************************************************************
;MACROs
;****************************************************************
;Program
;C source will contain main ()
;Only subroutines and ISRs in this assembly source
            AREA    MyCode,CODE,READONLY
                
            EXPORT PUTCHAR
            EXPORT GETCHAR
            EXPORT UART0_IRQHandler
            EXPORT Init_UART0_IRQ
            EXPORT UART0_ISR
;>>>>> begin subroutine code <<<<<
;-----------------------------------------------------------------			
;Subroutine Name: Init_UART0_IRQ
;Initialize the KL64 for interrupts and initializes the receive
;and transmit queues.
;Inputs: None
;Outputs: None
;Registers Changed: APSR
;Uses: Enqueue, Dequeue
;-----------------------------------------------------------------	
Init_UART0_IRQ	PROC		{R0-R14}	;Specified registers value will not change after return
				PUSH		{R0-R3, LR}		;Push the contents of R0-R3 to the stack
;Initialize TxQueue and RxQueue
			LDR 	R0, =RxQueue
			LDR 	R1, =RxRecord
			MOVS 	R2, #RX_Q_SZ
			BL 		InitQueue
			LDR 	R0, =TxQueue
			LDR 	R1, =TxRecord
			MOVS 	R2, #TX_Q_SZ
			BL 		InitQueue	
; Select MCGPLLCLK / 2 as UART0 clock source
			LDR 	R1,=SIM_SOPT2
			LDR 	R2,=SIM_SOPT2_UART0SRC_MASK
			LDR 	R3,[R1,#0]
			BICS 	R3,R3,R2
			LDR 	R2,=SIM_SOPT2_UART0_MCGPLLCLK_DIV2
			ORRS	R3,R3,R2
			STR 	R3,[R1,#0]
; Enable external connection for UART0
			LDR 	R1,=SIM_SOPT5
			LDR 	R2,= SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
			LDR 	R3,[R1,#0]
			BICS 	R3,R3,R2
			STR 	R3,[R1,#0]
; Enable clock for UART0 module
			LDR 	R1,=SIM_SCGC4
			LDR 	R2,= SIM_SCGC4_UART0_MASK
			LDR 	R3,[R1,#0]
			ORRS 	R3,R3,R2
			STR 	R3,[R1,#0]
; Enable clock for Port A module
			LDR 	R1,=SIM_SCGC5
			LDR 	R2,= SIM_SCGC5_PORTA_MASK
			LDR 	R3,[R1,#0]
			ORRS 	R3,R3,R2
			STR 	R3,[R1,#0]
; Connect PORT A Pin 1 (PTA1) to UART0 Rx (J1 Pin 02)
			LDR 	R1,=PORTA_PCR1
			LDR 	R2,=PORT_PCR_SET_PTA1_UART0_RX
			STR 	R2,[R1,#0]
; Connect PORT A Pin 2 (PTA2) to UART0 Tx (J1 Pin 04)
			LDR 	R1,=PORTA_PCR2
			LDR 	R2,=PORT_PCR_SET_PTA2_UART0_TX
			STR 	R2,[R1,#0]
;Disable UART0 receiver and transmitter
			LDR 	R1,=UART0_BASE
			MOVS 	R2,#UART0_C2_T_R
			LDRB 	R3,[R1,#UART0_C2_OFFSET]
			BICS 	R3,R3,R2
			STRB 	R3,[R1,#UART0_C2_OFFSET]
;Initialize NVIC for UART0 interrupts
		     LDR 	R0, =UART0_IPR
			 LDR 	R1, =NVIC_IPR_UART0_MASK
		     LDR 	R2, =NVIC_IPR_UART0_PRI_3
		     LDR 	R3, [R0, #0]
			 BICS 	R3, R3, R1
             ORRS 	R3, R3, R2
	         STR 	R3, [R0, #0]
;Clear any pending UART0 Interrupts
		     LDR 	R0, =NVIC_ICPR
	 	     LDR 	R1, =NVIC_ICPR_UART0_MASK
		     STR 	R1, [R0, #0]
;Unmask UART0 interrupts
             LDR 	R0, =NVIC_ISER
		     LDR 	R1, =NVIC_ISER_UART0_MASK
		     STR 	R1, [R0, #0]
;Set UART0 for 9600 baud, 8N1 protocol
			LDR		R1, =UART0_BASE
			MOVS 	R2,#UART0_BDH_9600
			STRB 	R2,[R1,#UART0_BDH_OFFSET]
			MOVS 	R2,#UART0_BDL_9600
			STRB 	R2,[R1,#UART0_BDL_OFFSET]
			MOVS 	R2,#UART0_C1_8N1
			STRB 	R2,[R1,#UART0_C1_OFFSET]
			MOVS 	R2,#UART0_C3_NO_TXINV
			STRB 	R2,[R1,#UART0_C3_OFFSET]
			MOVS 	R2,#UART0_C4_NO_MATCH_OSR_16
			STRB 	R2,[R1,#UART0_C4_OFFSET]
			MOVS 	R2,#UART0_C5_NO_DMA_SSR_SYNC
			STRB 	R2,[R1,#UART0_C5_OFFSET]
			MOVS 	R2,#UART0_S1_CLEAR_FLAGS
			STRB 	R2,[R1,#UART0_S1_OFFSET]
			MOVS 	R2, \
					#UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
			STRB 	R2,[R1,#UART0_S2_OFFSET]
; Enable UART0 receiver, transmitter and transmit interrupts
			MOVS 	R2,#UART0_C2_TI_RI
			STRB 	R2,[R1,#UART0_C2_OFFSET]
			POP		{R0-R3, PC}		;Restore the contents of R1-R3
			BX		LR				;Branch and exchange(Return)
			ENDP					;Ends the subroutine

;-----------------------------------------------------------------			
;Subroutine Name: UART0_ISR
;Initialize interrupt service routine for UART0
;Inputs: None
;Outputs: None
;Registers Changed: APSR
;Uses: Enqueue, Dequeue
;-----------------------------------------------------------------
UART0_IRQHandler
UART0_ISR	PROC	{R0-R14}		;Specified registers value will not change after return
			CPSID	I				;Mask other interrupts
			PUSH	{R0-R3, LR}			;Push the contents of specified registers to the stack

;Check if TxInterrupt(TIE) is enabled			
			LDR 	R0, =UART0_BASE
			LDRB	R1, [R0, #UART0_C2_OFFSET]
			MOVS	R2, #0x80
			ANDS	R1, R1, R2
			CMP		R1, #0
			BEQ		CHECKRX		
			B		TXENABLED
			
;If TxInterrupt(TIE) check if TxInterrupt(TDRE) is enabled
TXENABLED	LDRB 	R1,[R0,#UART0_S1_OFFSET]
			MOVS 	R2, #0x80
			ANDS	R1, R1, R2
			CMP 	R1, #0
			BEQ 	CHECKRX
;If TxInterrupt(TDRE) is enabled		
			LDR 	R1, =TxRecord
			MOVS 	R2, #TX_Q_SZ
			BL 		Dequeue
			BCS 	DISABLETX
			
;If dequeue succeeds write character to UART0 data register 			
			LDR 	R1, =UART0_BASE			
			STRB 	R0, [R1, #UART0_D_OFFSET]
			
;End subroutine			
			B 		ENDISR

;Disable TxInterrupt if Dequeue fails			
DISABLETX	MOVS 	R1,#UART0_C2_T_RI
            STRB 	R1,[R0,#UART0_C2_OFFSET]
			B 		ENDISR

;Check if RxInterrupt is enabled		
CHECKRX		LDR 	R0, =UART0_BASE
			LDRB 	R1,[R0,#UART0_S1_OFFSET]
			MOVS 	R2, #0x10
			ANDS 	R1, R1, R2
			CMP 	R1, #0
			BEQ 	ENDISR
;If RxInterrupt is enabled enqueue into RxQueue
			LDRB 	R3, [R0, #UART0_D_OFFSET]
			LDR 	R1, =RxRecord
			MOVS 	R0, R3
			BL 		Enqueue

;Unmask interrupts and restore registers		
ENDISR		CPSIE	I				;Unmask other interrupts
			POP		{R0-R3, PC}		;Restore the contents of the specified register
			ENDP					;Ends the subroutine


;------------------------------------------------------------
;Subroutine Name: NEWLINE
;Moves the pointer to the next line
;Inputs:None
;Outputs:a carriage return and line fill to the commmand line 
;Registers Changed: R0
;Uses:PUTCHAR
;-------------------------------------------------------------
NEWLINE		PROC		{R0-R14}		;Specified registers value will not change after return
			PUSH		{R0,LR}			;Push the contents of LR to the stack
			MOVS		R0, #CR			;Store the ASCII for a carriage return in R0
			BL			PUTCHAR			;Perform a carriage return
			MOVS		R0, #LF			;Store the ASCII for line fill in R0
			BL			PUTCHAR			;Move the line pointer down a line
			POP			{R0,PC}			;Restore the contents of the specified register
			ENDP						;End the subroutine

;------------------------------------------------------------
;Subroutine Name: SETC
;Sets the C flag
;Inputs:None
;Outputs:APSR C(1)
;Registers Changed: None
;-------------------------------------------------------------	
SETC		PROC		{R0-R14}		;Specified registers value will not change after return			
			PUSH		{R0-R1}			;Push the contents of R0-R1 to the stack
			MRS			R3, APSR		;Transfers the contents of APSR into R3
			MOVS		R4, #0x20		;Move 0x20 into R4
			LSLS 		R4, R4, #24		;Logical Shift Left contents in R4
			ORRS		R3, R3, R4		;Logical OR the contents in R3 and R4
			MSR			APSR, R3		;Transfers the contents of R3 into APSR
			POP			{R0-R1}			;Restore the contents of the specified register
			BX			LR				;Branch and exchange(Return)
			ENDP						;End the subroutine
;------------------------------------------------------------
;Subroutine Name: CLEARC
;Clears the C flag
;Inputs:None
;Outputs:APSR C(0)
;Registers Changed: None
;-------------------------------------------------------------	
CLEARC		PROC		{R0-R14}		;Specified registers value will not change after return				
			PUSH		{R0-R1}			;Push the contents of R0-R1 to the stack
			MRS			R3, APSR		;Transers the contents of APSR into R3
			MOVS		R4, #0x20		;Move 0x20 into R4
			LSLS		R4, R4, #24		;Logical Shift Left contents in R4
			BICS		R3, R3, R4		;Logical and the contets in R3 and R4
			MSR			APSR, R3		;Transfers the contents of R3 into APSR
			POP			{R0-R1}			;Restore the contents of the specified register
			BX			LR				;Branch and exchange(Return)	
			ENDP	
;------------------------------------------------------------
;Subroutine Name: PQUEUE
;Prints the characters enqueued in the list
;Inputs:
;Outputs:None 
;Registers Changed: APSR
;-------------------------------------------------------------
PQUEUE		PROC		{R0-R14}			;Specified registers value will not change after return
			PUSH		{R0-R4,LR}			;Push the contents of specified registers to the stack
			LDR			R1, =QRECORD		;R1<--QRECORD
			LDRB		R2, [R1, #NUM_ENQD]	;Counter
			LDR			R3, [R1, #OUT_PTR]	;Address of out pointer
			LDR			R4, [R1, #BUF_PAST]	;Address past the buffer
LOOPP		SUBS		R2, R2, #1			;Subtract from num enqd	
			CMP			R2, #0				;Compare num enqd to zero
			BLT			ENDLOOPP			;Branch if less than zero
			CMP			R3, R4				;Compare OUT_PTR to BUF_PAST
			BLT			OKAY				;If OUT_PTR is less than or equal to then B OKAY
			LDR			R3, [R1, #BUF_STRT]	;Change OUT_PTR to BUF_STRT
OKAY		LDRB		R0, [R3, #0]		;Load element in queue into R0
			BL			PUTCHAR				;Put the string on the terminal
			MOVS		R0, #0x20			;Put a space on the terminal
			BL			PUTCHAR				;Put a space after each element
			ADDS		R3, R3, #1			;Add to the outpointer
			B			LOOPP				;Branch to LOOPP
ENDLOOPP	POP			{R0-R4,PC}			;Restore the contents of the specified register
			ENDP							;End the subroutine
;-----------------------------------------------------------------
;Subroutine Name: GETCHAR
;Dequeues a character from the RxQueue queue.
;Inputs: None
;Outputs: R0: Character Dequeued
;Registers Changed: R0
;Uses: Dequeue 
;-----------------------------------------------------------------
GETCHAR	PROC	{R0-R14}			;Specified registers value will not change after return
		PUSH	{R1, LR}			;Push the contents of specified registers to the stack
		LDR 	R1, =RxRecord		;R1<--Address of RxQueue record structure
		
GETOP	CPSID 	I					;Mask other interrupts
		BL		Dequeue				;Dequeue character at the top of the RxQueue and store in R0
		CPSIE	I					;Unmask other interrupts
		BCS		GETOP				;If Dequeue fails branch to PUTOP
		
		POP		{R1, PC}			;Restore the contents of the specified registers
		ENDP						;Ends the subroutine

;-----------------------------------------------------------------
;Subroutine Name: PUTCHAR
;Initialize interrupt service routine for UART0
;Inputs: R0: Character to enqueue
;Outputs: None
;Registers Changed: APSR
;Uses: Enqueue
;-----------------------------------------------------------------
PUTCHAR	PROC	{R0-R14}			;Specified registers value will not change after return
		PUSH	{R0-R1, LR}			;Push the contents of specified registers to the stack
		LDR		R1, =TxRecord		;R1<--Address of TxQueue record structure
		
PUTOP	CPSID	I					;Mask other interrupts
		BL		Enqueue				;Enqueue character in R0 into TxQueue
		CPSIE	I					;Unmask other interrupts
		BCS		PUTOP				;If enqueue fails branch to PUTOP

		;Enable TxInterrupt
		LDR		R0, =UART0_BASE		;R0<--UART0 Base Address 
		MOVS	R1, #UART0_C2_TI_RI	;R1<--Enable bits
		STRB	R1, [R0, #UART0_C2_OFFSET] ;Enable Transmit interrupts`
		
		POP		{R0-R1,PC}			;Restore the contents of the specified register
		ENDP						;Ends the subroutine
			
;-------------------------------------------------------------------
;Subroutine Name: GetStringSB
;Reads a string from the terminal keyboard
;Inputs: Memory address of the string
;Outputs:
;Register(s) Changed: APSR
;Uses: GETCHAR subroutine
;------------------------------------------------------------------- 
GetStringSB	PROC		{R1-R13}		;Specified registers value will not change after return
			PUSH 		{R1-R4, LR}		;Store contents in the specified registers in stack
			
			SUBS		R1, R1, #1		;Subtract 1 from the buffer limit
			MOVS		R3, #0			;Initialize the string pointer
			MOVS		R2, R0			;Creates a copy of the memory address of the string
			
LOOPGET		BL			GETCHAR			;Get the character typed in the terminal
			CMP			R0, #CR			;Comapare character in R0 to carriage return
			BEQ			ENDLOOPGET		;If R0 is a carriage return, endloop
			CMP			R0, #BS			;Compare character in R0 to backspace
			BNE			CONTINUE		;If it is not equal branch to CONTINUE
			SUBS		R3, R3, #1		;Decrease the counter
			B			LOOPGET			;Loop
CONTINUE	STRB		R0, [R2,R3]		;Otherwise, Store the character in R0 in R2's memory location
			BL			PUTCHAR			;Put the character entered on the terminal
			ADDS		R3, R3, #1		;Increment counter
			CMP			R3, R1			;Compare buffer and pointer
			BEQ			JUSTPUT			;If the buffer limit is the same as the pointer branch to JUSTPUT
			B			LOOPGET			;Loop
JUSTPUT		BL			GETCHAR			;Get the character typed in the terminal
			CMP			R0, #CR			;Comapare character in R0 to carriage return
			BEQ			ENDLOOPGET		;If R0 is a carriage return, endloop
			BL			PUTCHAR			;Put the character entered on the terminal
			B			JUSTPUT			;Branch to JUSTPUT
ENDLOOPGET	MOVS		R4, #NULL		;Store zero in R4
			MOVS		R0, R2			;Restore R0 memory location
			STRB		R4, [R0, R3]	;Store zero in the final location of the string
			POP			{R1-R4, PC}		;Restore the contents of specified registers
			ENDP
;-------------------------------------------------------------------
;Subroutine Name: PutStringSB
;Displays String stored in memory to the terminal screen
;Inputs: memory address of string(stored in R0)
;Outputs:
;Register(s) Changed: APSR
;Uses: PUTCHAR subroutine
;--------------------------------------------------------------------
PutStringSB	PROC		{R0-R14}		;Specified registers value will not change after return
			PUSH		{R0-R3, LR}		;Store specified registers on the stack
			
			MOVS		R3, #0			;Initialise the pointer
			MOVS		R2, R0			;Copy memory address stored in R0 into R2
			
LOOPPUT		LDRB		R0, [R2, R3]	;Load the contents of the memory value stored in R2 into R0
			ADDS		R3, R3, #1		;Increments the pointer
			CMP			R0, #NULL		;Compare R0 to NULL
			BEQ			ENDLOOPPUT		;If R0 contains zero endloop
			BL			PUTCHAR			;Call the PUTCHAR subroutine
			B			LOOPPUT			;Branch to Loop
ENDLOOPPUT	POP			{R0-R3, PC}		;Restore the contents of specified registers
			ENDP
;-------------------------------------------------------------------
;Subroutine Name: PutNumU
;Prints a text representation of a 
;Inputs: Unsigned word value to print
;Outputs:
;Register(s) Changed: APSR
;Uses: PUTCHAR Subroutine
;--------------------------------------------------------------------
PutNumU		PROC		{R0-R15}		;Specified registers value will not change after return
			PUSH		{R0-R4, LR}		;Store registers specified on the stack
			
			MOVS		R1, R0			;Store the number in R1 for DIVU
			LDR			R0, =10			;Store a digit place in R0 as the Divisor

LOOPNUM		MOVS		R4, R0			;Create a copy of the number
			BL			DIVU			;Divide the number by the digit place	
			ADDS		R0, #0x30
			ADDS		R0, R0, #NULL	;Add null terminate to the quotient of the division
			BL			PUTCHAR			;Put the quotient on the terminal
			MOVS		R0, R1			;Move the value in 
			ADDS		R0, #0x30		;Add the value 0x30 into R0 so the appropriate number can be displayed
			ADDS		R0, R0, #NULL	;Add a null terminate to the end of the number in R0
			BL			PUTCHAR			;Put the number on the terminal
			POP			{R0-R4, PC}		;Restore the contents of specified registers
			ENDP						;End Subroutine
;-------------------------------------------------------------------
;Subroutine Name: DIVU
;Divides a dividend in R1 by divisor(R0) and stores
;the quotient(R0) and remainder(R1)
;Inputs:dividend and divisor
;Outputs:quotient and remainder
;Register(s) Changed: R0, R1
;Uses: No subroutine
;--------------------------------------------------------------------
DIVU		PROC 	{R5-R14}		;Do not use R5-R14
            PUSH	{R2-R4}         ;Save contents of registers R2-R4
	
			CMP 	R0, #0          ;Check if Divisor is 0
			BEQ		SETFLAG         ;If zero branch to SETFLAG
			CMP		R1, #0          ;Check if Dividend is 0
			BEQ		ZERO            ;If zero branch to ZERO
		
			MOVS	R2, #0			;Quotient = 0
		
WHILE		CMP		R1, R0			;Compare Divdend and Divisor
			BLO		ENDWHILE		;If Divisor is less than dividend 	
			SUBS	R1, R1, R0		;Dividend = Dividend - Divisor
			ADDS	R2, R2, #1		;Quotient = Quotient + 1
			B		WHILE			;Restart Loop
ENDWHILE	MOVS	R0, R2			;Remainder = Dividend
			B		CLEARFLAG       ;Branch to CLEARFLAG
			
ZERO		MOVS	R0, #0          ;Set Remainder to 0
			B		CLEARFLAG       ;Branch to CLEARFLAG
			
;Clears the C flash without changing other flags
CLEARFLAG	MRS		R3, APSR		;Transers the contents of APSR into R3
			MOVS	R4, #0x20		;Move 0x20 into R4
			LSLS	R4, R4, #24		;Logical Shift Left contents in R4
			BICS	R3, R3, R4		;Logical and the contets in R3 and R4
			MSR		APSR, R3		;Transfers the contents of R3 into APSR
			B       FINISH          ;Branch to FINISH

;Sets the C flag without changing other flags
SETFLAG		MRS		R3, APSR		;Transfers the contents of APSR into R3
			MOVS	R4, #0x20		;Move 0x20 into R4
			LSLS 	R4, R4, #24		;Logical Shift Left contents in R4
			ORRS	R3, R3, R4		;Logical OR the contents in R3 and R4
			MSR		APSR, R3		;Transfers the contents of R3 into APSR
			
FINISH      POP		{R2-R4}         ;Restore contents of registers R2-R4
            BX      LR              ;Branch and exchange
			ENDP                    ;End subroutine
				
;-------------------------------------------------------------------
;Subroutine Name:InitQueue
;Initialises the Queue
;Inputs:R0:Contains the queue buffer
;		R1:Buffer record starting address
;		R2:Size of the buffer
;Outputs:None
;Register(s) Changed: APSR
;Uses:None
;------------------------------------------------------------------- 
InitQueue	PROC		{R0-R14}				;Register values would be unchanged upon return
			PUSH		{R0-R2}					;Store contents of Subroutine
			STR   		R0,[R1,#IN_PTR]			;IN_PTR<--Queue buffer address
			STR   		R0,[R1,#OUT_PTR] 		;OUT_PTR<--Queue buffer address
			STR   		R0,[R1,#BUF_STRT]  		;BUF_STRT<--Queue buffer address
			ADDS  		R0,R0,R2 				;R0<--Queue buffer address + Buffer size
			STR   		R0,[R1,#BUF_PAST]		;BUF_PAST<--Queue buffer address + Buffer size
			STRB  		R2,[R1,#BUF_SIZE] 		;BUF_SIZE<--Buffer size
			MOVS  		R0,#0 					;R0<--0
			STRB  		R0,[R1,#NUM_ENQD]		;NUM_ENQD<--0
			POP			{R0-R2}					;Restore contents of specified registers
			BX			LR						;Branch and Exchange
			ENDP								;End subroutine

;-------------------------------------------------------------------
;Subroutine Name:Enqueue
;Removes a specified character from the buffer 
;Inputs: R0: Character to Enqueue
;		 R1: Address of queue record structure
;Outputs:PSR: C(1)- Failure or C(0) Success
;Register(s) Changed: APSR
;Uses: CLEARC, SETC
;------------------------------------------------------------------- 
Enqueue 	PROC		{R0-R14}				;Registers value should not be changed upon return from subroutine
			PUSH		{R0-R5, LR}				;Store contents of specifed registers
			
			LDRB		R2, [R1, #NUM_ENQD]		;R2<--Adress of number of elements enqueued
			LDRB		R3, [R1, #BUF_SIZE]		;R3<--Gets the size of the buffer
						
			CMP			R2, R3					;Compare number of elements enqueue with buffer size
			BGE			FULL					;If number of elements enqueued is greater than the buffer then branch to full
			
			LDR			R3, [R1, #IN_PTR]		;Load Input pointer into R3
			STRB		R0, [R3, #0]			;Store the character in R0 in the memory address at R3	
			
			ADDS		R2, R2, #1				;Increment number of elements enqueued
			STRB		R2, [R1, #NUM_ENQD]		;Store number of elements enqueued
			
			ADDS		R3, R3, #1				;R4<--IN_PTR + 1
			STR			R3, [R1, #IN_PTR]		;IN_PTR + 1 -->QueueRecord
			
			LDR			R4, [R1, #BUF_PAST]		;R5<--BUFFER PAST
			CMP			R3, R4 					;Compare IN_PTR to Buffer past
			BLT			NORESIZE				;If IN_PTR is less than buffer size branch to NORESIZE
			
			LDR			R5, [R1, #BUF_STRT]		;R5<--Buffer Start Address
			STR			R5, [R1, #IN_PTR]		;IN_PTR<--Buffer Start		
			
NORESIZE	BL			CLEARC					;Clear C flag
			B			DONE					;Branch to Done
			
FULL		BL			SETC					;Set C flag
DONE		POP			{R0-R5, PC}				;Restore contents of specified register
			ENDP								;End Subroutine

;-------------------------------------------------------------------
;Subroutine Name:Dequeue
;Removes a specified character from the buffer
;Inputs:R1: Address of queue record structure
;Outputs:R0: Character Dequeued
;		 PSR: C(1)- Failure or C(0) Success
;Register(s) Changed: APSR
;Uses: None
;-------------------------------------------------------------------
Dequeue		PROC		{R0-R14}				;Registers value should not be changed upon return from subroutine
			PUSH		{R1-R5, LR}				;Store contents of specifed registers
			
			LDRB		R2, [R1, #NUM_ENQD]		;R2<--No. of elements enqueued
			CMP			R2, #0					;Compare number of elements and zero
			BLE			EMPTY					;If theres nothing to dequeue branch to EMPTY	
			
			LDR			R3, [R1, #OUT_PTR]		;R3<--OUT_PTR
			LDRB		R0, [R3, #0]			;R0<--M[OUT_PTR]
			
			SUBS		R2, R2, #1				;Decrement num of elements
			STRB		R2, [R1, #NUM_ENQD] 	;No of Elements<--R2	
			
			ADDS		R3, R3, #1				;OUT_PTR + 1
			STR			R3, [R1, #OUT_PTR]		;OUT_PTR<-- OUT_PTR + 1
			
			LDR			R4, [R1, #BUF_PAST]		;R4<--BUF_PAST
			CMP			R3, R4					;Compare outpointer to buffer past
			BLT			NORESIZE1				;If outpointer is less than buffer past then branch to NORESIZE1
			
			LDR			R5, [R1, #BUF_STRT]		;R5<--Buffer Start Address		
			STR			R5, [R1, #OUT_PTR]		;IN_PTR<--Buffer Start
			
NORESIZE1	BL			CLEARC					;Clear C flag (Success)
			B			DONE1					;Branch to Done
			
EMPTY		BL			SETC					;Set C flag	(Failed)		
DONE1		POP			{R1-R5, PC}				;Restore contents of specified register
			ENDP								;End Subroutine
				
;-------------------------------------------------------------------
;Subroutine Name:PutNumHex
;Prints to the terminal screen the text hexadecimal representation of the
;unsigned word value in R0.
;Inputs: R0:Hexadecimal number
;Outputs:Hexadecimal number to terminal screen
;Register(s) Changed: APSR
;Uses: None
;------------------------------------------------------------------- 
PutNumHex	PROC		{R0-R14}				;Registers value should not be changed upon return from subroutine
			PUSH		{R0-R3, LR}				;Store contents of specifed registers
			MOVS		R1, #0					;Initialise Counter
			LDR			R2, =0xF0000000			;Store mask in R2
			MOVS		R4, #28					;Store LSRS shift number
LOOP		MOVS		R3, R0					;Copy the hexadecimal value
			CMP			R1, #7					;Compare counter to 7
			BGT			ENDLOOP					;If counter is 7 or greater endloop
			ANDS		R0, R0, R2				;AND mask with hexadecimal number in R0
			LSRS		R0, R0, R4				;Shift the number to the first byte
			ADDS		R0, R0, #0x30 			;add 0x30 to R0 to represent ASCII text
			CMP			R0, #0x39				;Comapare 0x30 + R0 to 0x39
			BLE			NUMBER					;If R0 is less than 0x39 then branch to NUMBER
			ADDS		R0, R0, #0x07			;Add 0x07 to account for numbers
NUMBER		BL			PUTCHAR					;Put the character on the terminal
			LSRS		R2, R2, #4				;Shift out the first bit in the original hex in R0
			MOVS		R0, R3					;Restore original hex value
			SUBS		R4, R4, #4				;Reduce LSRS shift number by a byte
			ADDS		R1, R1, #1				;Increment counter
			B			LOOP					;Loop
ENDLOOP		POP			{R0-R3,PC}				;Restore contents of specified register
			ENDP								;End Subroutine
				
;-------------------------------------------------------------------
;Subroutine Name:PutNumUB
;Prints to the terminal screen the text decimal representation of the
;unsigned byte value in R0.
;Inputs: R0:Unsigned Hex number
;Outputs:Decimal number to terminal screen 
;Register(s) Changed: APSR
;Uses:PutNumU
;------------------------------------------------------------------- 				
PutNumUB 	PROC		{R0-R14}				;Registers value should not be changed upon return from subroutine	
			PUSH		{R0-R1,LR}				;Store contents of specifed registers
			MOVS		R1, #0xFF				;Store mask
			ANDS		R0, R0, R1				;AND Hex value in R0 with mask
			BL			PutNumU					;Put the number on the terminal
			POP			{R0-R1,PC}				;Restore contents of specified register
			ENDP								;End Subroutine
	
;>>>>>   end subroutine code <<<<<
            ALIGN
;**********************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
PROMPT		DCB		"Type a queue command (D,E,H,P,S):" , NULL 
FAIL		DCB		"Failure:" , NULL
SUC			DCB		"Success:", NULL
ENQ			DCB		"Character to enqueue:" , NULL
HELP		DCB		"d (dequeue), e (enqueue), h (help), p (print), s (status)", NULL
STAT		DCB		"Status:", NULL
IN			DCB		"  In=0x", NULL
OUT			DCB		"  Out=0x", NULL
NUM			DCB		"  Num=", NULL
;>>>>>   end constants here <<<<<
;**********************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
QBUFFER		SPACE		Q_BUF_SZ	;Queue buffer size
			ALIGN
QRECORD		SPACE		Q_REC_SZ	;Queue record size
			ALIGN
RxQueue		SPACE		RX_Q_SZ		;Queue buffer size
			ALIGN
RxRecord	SPACE		Q_REC_SZ	;Queue record size
			ALIGN
TxQueue		SPACE		TX_Q_SZ		;Queue buffer size
			ALIGN
TxRecord	SPACE		Q_REC_SZ	;Queue record size
			ALIGN
STRING		SPACE		79			;String Space
;>>>>>   end variables here <<<<<
            END
