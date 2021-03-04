            TTL Program Title for Listing Header Goes Here
;****************************************************************
;Name:  Chris Larson
;Date:  16 October 2018
;Class:  CMPE-250
;Section:  Section 2 Tuesday 2pm
;---------------------------------------------------------------
;Keil Template for KL46
;R. W. Melton
;February 5, 2018
;****************************************************************
;Assembler directives
            THUMB
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL46Z4.s     ;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;EQUates
SIZERSW     EQU     16
MAX_STRING  EQU     80
;Management record structure field displacements
IN_PTR		EQU		0
OUT_PTR		EQU		4
BUF_STRT	EQU		8
BUF_PAST	EQU		12
BUF_SIZE	EQU		16
NUM_ENQD	EQU		17
;Queue structure sizes
Q_BUF_SZ    EQU     80  ;Room for 80 characters
Q_REC_SZ    EQU     18  ;Management record size
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
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ICER_PIT_MASK    EQU  PIT_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;22:PIT IRQ pending status
;12:UART0 IRQ pending status
NVIC_ICPR_PIT_MASK    EQU  PIT_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
;--PIT
PIT_IRQ_PRIORITY    EQU  0
NVIC_IPR_PIT_MASK   EQU  (3 << PIT_PRI_POS)
NVIC_IPR_PIT_PRI_0  EQU  (PIT_IRQ_PRIORITY << UART0_PRI_POS)
;--UART0
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ISER_PIT_MASK    EQU  PIT_IRQ_MASK
;---------------------------------------------------------------
;PIT_LDVALn:  PIT load value register n
;31-00:TSV=timer start value (period in clock cycles - 1)
;Clock ticks for 0.01 s at 24 MHz count rate
;0.01 s * 24,000,000 Hz = 240,000
;TSV = 240,000 - 1
PIT_LDVAL_10ms  EQU  239999
;---------------------------------------------------------------
;PIT_MCR:  PIT module control register
;1-->    0:FRZ=freeze (continue'/stop in debug mode)
;0-->    1:MDIS=module disable (PIT section)
;               RTI timer not affected
;               must be enabled before any other PIT setup
PIT_MCR_EN_FRZ  EQU  PIT_MCR_FRZ_MASK
;---------------------------------------------------------------
;PIT_TCTRLn:  PIT timer control register n
;0-->   2:CHN=chain mode (enable)
;1-->   1:TIE=timer interrupt enable
;1-->   0:TEN=timer enable
PIT_TCTRL_CH_IE  EQU  (PIT_TCTRL_TEN_MASK :OR: PIT_TCTRL_TIE_MASK)

;---------------------------------------------------------------
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
            IMPORT  Startup
Reset_Handler  PROC  {}
main
;---------------------------------------------------------------
;Mask interrupts
            CPSID   I
;KL46 system startup with 48-MHz system clock
            BL      Startup
;---------------------------------------------------------------
;>>>>> begin main program code <<<<< 
;        BL Init_PIT_IRQ    ;Initializes PIT
        BL Init_UART0_IRQ   ;Initializes IRQ
 
Start   LDR     R0,=INSTRUCT
        BL PutString	;Puts the instructions out and waits for a character
GetLoop MOVS 	R0,#'>'	;Displays the String variable with ">" around the string
        BL PutChar
        LDR     R3,=OPEN
        BL GetString
Time    MOVS    R0,#'<'
;        BL PIT_ISR
        ;Time stuff  
        CMP     R0,R3
        BEQ     Grant
        LDR     R0,=DENY
        BL PutString
        B   Start    
        
Grant   LDR     R0,=GRANT
        BL PutString
        LDR     R0,=COMPLETE
        BL PutString
;>>>>>   end main program code <<<<<
;Stay here
            B       .
            ENDP
            LTORG
;>>>>> begin subroutine code <<<<<
;****************************************************************
;Init_PIT_IRQ    PROC {R0-R14}
;        PUSH {R0-R3,LR}
;        ;Enable clock for PIT module             
;        LDR   R0,=SIM_SCGC6             
;        LDR   R1,=SIM_SCGC6_PIT_MASK             
;        LDR   R2,[R0,#0]             
;        ORRS  R2,R2,R1            
;        STR   R2,[R0,#0]
;        
;        ;Disable PIT timer 0 
;        LDR   R0,=PIT_CH0_BASE 
;        LDR   R1,=PIT_TCTRL_TEN_MASK 
;        LDR   R2,[R0,#PIT_TCTRL_OFFSET] 
;        BICS  R2,R2,R1
;        STR   R2,[R0,#PIT_TCTRL_OFFSET]
;        
;        ;Set PIT interrupt priority 
;        LDR     R0,=PIT_IPR 
;        LDR     R1,=NVIC_IPR_PIT_MASK 
;        ;LDR     Rk,=NVIC_IPR_PIT_PRI_0 
;        LDR     R3,[R0,#0] 
;        BICS    R3,R3,R1
;        ;ORRS    Rl,Rl,Rk
;        STR     R3,[R0,#0] 
;        ;Clear any pending PIT interrupts 
;        LDR     R0,=NVIC_ICPR 
;        LDR     R1,=NVIC_ICPR_PIT_MASK 
;        STR     R1,[R0,#0]
;        
;        ;Unmask PIT interrupts 
;        LDR     R0,=NVIC_ISER 
;        LDR     R1,=NVIC_ISER_PIT_MASK 
;        STR     R1,[R0,#0]
;        
;        ;Enable PIT module 
;        LDR   R0,=PIT_BASE 
;        LDR   R1,=PIT_MCR_EN_FRZ 
;        STR   R1,[R0,#PIT_MCR_OFFSET] 
;        ;Set PIT timer 0 period for 0.01 s 
;        LDR   R0,=PIT_CH0_BASE 
;        LDR   R1,=PIT_LDVAL_10ms 
;        STR   R1,[R0,#PIT_LDVAL_OFFSET]
;        ;Enable PIT timer 0 interrupt 
;        LDR   R1,=PIT_TCTRL_CH_IE 
;        STR   R1,[R0,#PIT_TCTRL_OFFSET]
;        
;        POP 	{R0-R3, PC}
;		ENDP
;;****************************************************************
;PIT_ISR PROC {R0-R14}
;        PUSH {R0-R2}
;        CPSID   I
;        LDR R0,=RunStopWatch
;        CMP R0,#0
;        BEQ CLRPIT
;        ADDS R0,R0,#1
;        STR R0,[R0,#0]
;        
;        ;Clear any pending PIT interrupts 
;CLRPIT  LDR     R0,=NVIC_ICPR 
;        LDR     R1,=NVIC_ICPR_PIT_MASK 
;        STR     R1,[R0,#0]
;        CPSIE   I
;        POP 	{R0-R3, PC}
;		ENDP
;****************************************************************
Init_UART0_IRQ	PROC		{R0-R14}	;Specified registers value will not change after return
				PUSH		{R0-R3, LR}		;Push the contents of R0-R3 to the stack
;Initialize TxQueue and RxQueue
			LDR 	R0, =RxBuffer
			LDR 	R1, =RxRecord
			MOVS 	R2, #MAX_STRING
			BL 		InitQueue
			LDR 	R0, =TxBuffer
			LDR 	R1, =TxRecord
			MOVS 	R2, #MAX_STRING
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
			MOVS 	R2,#UART0_C2_T_RI
			STRB 	R2,[R1,#UART0_C2_OFFSET]
			POP		{R0-R3, PC}		;Restore the contents of R1-R3
			ENDP					;Ends the subroutine

;-----------------------------------------------------------------	
;Initializes the queue record structure at the addresss in R1 for the
;empty queue buffer at the address in R0 of size, given by R2
InitQueue  PROC    {R0-R14}
		PUSH	{R0-R3, LR}
		STR		R0,[R1,#IN_PTR]     ;Stores the in pointer
        MOVS    R3,R0
		STR		R0,[R1,#OUT_PTR]    ;Stores the out pointer
        MOVS    R3,R0
		STR		R0,[R1,#BUF_STRT]   ;Stores the buff start
        MOVS    R3,R0
        ;MOVS    R2,#Q_BUF_SZ
		ADDS	R0,R0,R2
		STR		R0,[R1,#BUF_PAST]   ;Stores the buff past
		STRB 	R2,[R1,#BUF_SIZE]   ;Stores the buff size
		MOVS 	R0,#0
		STRB 	R0,[R1,#NUM_ENQD]   ;Stores the num enqueued
		POP 	{R0-R3, PC}
		ENDP
;****************************************************************
UART0_ISR       PROC {R0-R14}
                CPSID I				;Mask other interrupts
                PUSH {LR}			;Push any registers used, except {R0-R3,R12} 
                LDR R0,=UART0_BASE
                MOVS R1,#UART0_C2_TIE_MASK
                LDRB R2,[R0,#UART0_C2_OFFSET]
                TST R2,R1
                BEQ RXQ
                MOVS R1,#UART0_S1_TDRE_MASK
                LDRB R2,[R1,#UART0_S1_OFFSET]
                TST R2,R1			;if (TxInteruptEnabled) then
                BEQ RXQ             ;TIE = 1 in UART0_C2 
                LDR R1,=TxRecord    ;Dequeue character from TxQueue 
                BL Dequeue			
                BCS FailISR			;if (dequeue successful) then 
                BL PutChar
                B RXQ
FailISR         MOVS R3,#UART0_C2_T_RI			;Disable TxInterrupt  ;UART0_C2_T_RI
                STRB R3,[R2,#UART0_C2_OFFSET]
RXQ             MOVS R1,#UART0_S1_RDRF_MASK
                LDR R2,[R0,#UART0_S1_OFFSET]
                TST R2,R1
                BEQ END_ISR
                LDR R1,=RxRecord
                BL Enqueue			;Enqueue character in RxQueue 
END_ISR         POP {PC}    		;Pop any registers pushed above 
                CPSIE I				;Unmask other interrupts				
                ENDP
;------------------------------------------------------------
;Attempts to get a character from the queue whose record structure's address is in R1
;If the queue is not empty, dequeues a single character from the queue to R0 and 
;clears the flag, to report dequeue successful, else flag is set to 1 and fails
Dequeue PROC {R0-R14}
		PUSH	{R1-R7, LR}
		LDRB    R2,[R1,#NUM_ENQD]	;R2 = NumberEnqueued
		CMP		R2,#0				;If(R2 = 0)
		BEQ	ZERODQ
		LDR		R3,[R1,#OUT_PTR]	;R3 = Outpointer
		SUBS	R2,R2,#1            ;Number Enqueued --
		ADDS	R3,R3,#1            ;Outpointer ++
		LDR		R4,[R1,#BUF_PAST]   ;R4 = BufferPast
		CMP		R3,R4               ;If(Outpointer points out side queue buffer)
		BLT	NoAdjD                      
		LDR		R3,[R1,#BUF_STRT]   ;Adjusts the Outpointer
        B ZERODQ
		
NoAdjD	MRS 	R6, APSR		    ;Clears the flag on return
		MOVS 	R7,#0X20		    ;Changes the C flag to 0
		LSLS 	R7, R7, #24
		BICS 	R6, R6, R7
		MSR 	APSR, R6
		B EndDeQ
		
ZERODQ	MRS 	R6, APSR		    ;Sets the flag on return
		MOVS 	R7,#0x20		    ;Changes the C flag to a 1
		LSLS 	R7, R7, #24
		ORRS 	R6, R6, R7
		MSR 	APSR, R6
		
EndDeQ	STRB	R2,[R1,#NUM_ENQD]   ;Number Enqueued = R2
		STR		R3,[R1,#OUT_PTR]    ;Outpointer = R3
		STR		R4,[R1,#BUF_STRT]   ;BufferStart = R4
		POP		{R1-R7, PC}
		ENDP
;****************************************************************
;Attempts to put a character in the queue whose queue record structure's address is in R1
; if the queue is not full, enqueues the single character from R0 to the queue, and set the C flag to 1 else sets
; the flag to 0 reporing a failure
Enqueue PROC    {R0-R14}
		PUSH	{R0-R7, LR}
		LDRB	R2,[R1,#NUM_ENQD]   ;R2 = Number Enqueued
		LDRB    R3,[R1,#BUF_SIZE]   ;R3 = Buffer Size
        LDR		R4,[R1,#IN_PTR]     ;R4 = Inpointer
		CMP		R2,R3               ;If(NumEnqueued < BuffSize)
		BGE		FULL                ;Queue is full
		LDR		R5,[R1,#BUF_STRT]   ;R5 = BufferStart
        LDR		R6,[R1,#BUF_PAST]
		STRB	R0,[R4,#0]          ;New Element into the Queue at the Inpointer
		ADDS	R2,R2,#1            ;Number Enqueued ++
        STRB    R2,[R1,#NUM_ENQD]
		ADDS	R4,R4,#1            ;Inpointer ++
		CMP		R4,R6               ;If(Inpointer >= BufferPast
		BLT		NoAdjE              
		STR		R5,[R1,#IN_PTR]     ;Adjust Inpointer to BufferStart
		
NoAdjE	MRS 	R6, APSR		    ;Clears the flag on return
		MOVS 	R7,#0X20		    ;Changes the C flag to 0
		LSLS 	R7, R7, #24
		BICS 	R6, R6, R7
		MSR 	APSR, R6
		B EndEnQ

FULL	MRS 	R6, APSR	    	;Sets the flag on return
		MOVS 	R7,#0x20		    ;Changes the C flag to a 1
		LSLS 	R7, R7, #24
		ORRS 	R6, R6, R7
		MSR 	APSR, R6

EndEnQ
		STR		R4,[R1,#IN_PTR]     ;Inpointer = R4
		POP		{R0-R7, PC}
		ENDP
;****************************************************************
;Moves the user to the next line
NewLine PROC {R0-R14}	
        PUSH {R0,LR}	;Makes sure the registers does get changed in the main loop
        MOVS R0,#0x0D	
        BL PutChar		;Send the user to the next line 
        MOVS R0,#0x0A
        BL PutChar
        POP {R0,PC}
        ENDP
;****************************************************************
;Gets a character from the Putty line that the user typed in
GetChar PROC 	{R1-R14}			    ;Procs R0-R14
		PUSH	{R1, LR}			    ;Pushes R0-R3 to the stack
        LDR R1,=RxRecord                 ;Loads Queue address
GetCLoop CPSID I                        ;Mask other interrupts
        BL Dequeue                      ;Dequeue character from RxQueue
        CPSIE I                         ;Unmask other interrupts
        BCS GetCLoop                    ;Loop until successful
		POP	{R1, PC}				    ;Pops R0-R3 from the stack
		ENDP
;****************************************************************
;Puts the character stored in R0 onto the Putty line
PutChar	PROC 	{R1-R14}			    ;Procs R0-R14
		PUSH	{R1-R2, LR}				;Pushes R0-R3 to the stack
        LDR R1,=TxRecord
PutLoop CPSID I                         ;Mask other interrupts
        BL Enqueue                      ;Enqueue character
        CPSIE I                         ;Unmask other interupts
        BCS PutLoop                     ;Loop until successful
        LDR R1,=UART0_BASE
        MOVS R2,#UART0_C2_TI_RI         ;Enable TxInterrupt
        STRB R2,[R1,#UART0_C2_OFFSET]   
		POP	 {R1-R2, PC}			        ;Pops R0-R3 from the stack
		ENDP
;****************************************************************
PutNumUB PROC    {R0-R14}           
        PUSH	{R1,LR}
        MOVS    R1,#0xFF            ;Prints to the terminal screen the decimal representation of the unsigned world in R0
        ANDS    R0,R0,R1            ;Masks the word
        B PutNumU
        POP     {R1,PC}
        ENDP
;****************************************************************
;Uses GetChar recursively to get a string from the user and stores it to memory
;Once the user hits the enter key it stops getting new characters
GetString PROC 	{R0-R13}			    ;Procs R0-R14
		PUSH	{R1-R5,LR}			    ;Pushes R1-R5 and LR to the stack
        CMP     R1,#0					;If the length of the string is zero ends
        BEQ Endget
        MOVS    R2,R0					;Stores the address in R2
        MOVS    R4,#0					;Offset
        SUBS    R1,R1,#1				;String length--
        BEQ NewLabel				
CharL   BL GetChar						;Gets the first character
        CMP     R0,#0x0D				;If(R0 = null)
        BEQ Null
        BL PutChar						;Displays the character
        STRB    R0,[R2,R4]				;Stores to the String variable
        ADDS    R4,#1					;Increment offset
        CMP     R4,R1					;Goes until R4 is larger than the length
        BLO     CharL
        
Null    MOVS    R1,#0					;Sets the character to nothing
        STRB    R1,[R2,R4]				;Stores to the String variable
        
NewLabel BL GetChar						;If(R0 != null) loop until it does
        CMP     R0,#0x0D
        BNE NewLabel
        
Done    BL PutChar						;Puts the character done
        MOVS    R0,#0x0A				;New Line
        BL PutChar
        POP	    {R1-R5,PC}
        
Endget	BL GetChar						;If (R0 != null) loop until it does
        CMP     R0,#0x0D
        BNE     Endget
        B Done
        ENDP
;****************************************************************
;Uses PutString recursively to display a string stored at the String variable memory address 
;until it reaches the null character 
PutString PROC	{R0-R14}			    ;Procs R0-R14
		PUSH	{R0-R2,LR}			    ;Pushes R1-R5 and LR to the stack
		MOVS	R2,R0				    ;Sets R2 to the address stored in R0
        CMP     R1,#0
        BEQ Endput
        ADDS    R1,R1,R2
LoadPut	LDRB    R0,[R2,#0]			    ;Loads R2 with an offset of R4 into R0
		CMP	    R0,#0			        ;Compares R0 to 0, it ends when the program finds a zero
		BEQ	Endput			            ;If 0 end program
		BL 	PutChar			    	    ;Runs PutChar
        ADDS    R2,R2,#1
    
		B	LoadPut				
Endput	POP	{R0-R2,PC}			        ;Pops R1-R5 and PC to the stack
        ENDP
;****************************************************************
;Uses DIVU recursively to divide the word by 10 and stores the value into R1 
;And puts the numbers on Putty line
PutNumU	PROC	{R1-R14}			    ;Procs R1-R14
        PUSH    {R1-R2,LR}
        MOVS R1,#10						;Sets the divisor to 10
        MOVS R2,#0						;Counter = 0
        
Loop1   BL DIVU							;Divides the word by 10
        PUSH    {R1}					;Pushes R1
        ADDS 	R2,R2,#1				;Counter increment
        CMP 	R0,#0					;If(String = 0) Loop2
        BEQ Loop2					
        B  Loop1						;Else Loop1
        
Loop2   CMP R2,#0						;If(Counter = 0)
        BEQ ExitLoop2					
        POP     {R1}					
        ADDS R1,#'0'					;Makes the character Ascii
        MOVS R0,R1						;Changes R0 and R1 for PutChar to work
        BL PutChar
        SUBS R2,R2,#1					;Counter-- 
        B Loop2
ExitLoop2
        POP {R1-R2,PC}
        BX LR
        ENDP
;****************************************************************	
;Takes the number stored in R0 and divides it by the number stored in R1 until
;The number in R0 is larger than R1
;Returns the Quotient in R0 and the remainder in R1
DIVU			PROC {R2-R14}
				PUSH {R2-R4}
				MOVS R2,#0			;quotient = zero)
				
				CMP R1,#0			;if(divisor = zero)
				BEQ	FLAGCHANGE
				
				CMP R0,#0			;if(dividend = zero)
				BEQ ZERO
				
WHILE			CMP R0,R1			;Is Dividend greater or equal to Divisor
				BLO CLEAR
				
				SUBS R0, R0, R1		;Dividend = Dividend - Divisor
				ADDS R2, R2, #1		;Quotient + 1
				B WHILE			
				
ZERO			MOVS R0,#0			;Both quotient and remainder set to zero
				MOVS R1,#0
				B CLEAR

FLAGCHANGE		MRS R3, APSR		;Sets the flag on return
				MOVS R4,#0x20		;Changes the C flag to a 1
				LSLS R4, R4, #24
				ORRS R3, R3, R4
				MSR APSR, R3
				B DONE				;Goes to finish
				
CLEAR			MRS R3, APSR		;Clears the flag on return
				MOVS R4,#0X20		;Changes the C flag to 0
				LSLS R4, R4, #24
				BICS R3, R3, R4
				MSR APSR, R3

DONE
				MOVS R1, R0			
				MOVS R0, R2
				POP {R2,R3,R4}
				BX LR
				ENDP
;****************************************************************
                
;>>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Vector Table Mapped to Address 0 at Reset
;Linker requires __Vectors to be exported
            AREA    RESET, DATA, READONLY
            EXPORT  __Vectors
            EXPORT  __Vectors_End
            EXPORT  __Vectors_Size
            IMPORT  __initial_sp
            IMPORT  Dummy_Handler
            IMPORT  HardFault_Handler
__Vectors 
                                      ;ARM core vectors
            DCD    __initial_sp       ;00:end of stack
            DCD    Reset_Handler      ;01:reset vector
            DCD    Dummy_Handler      ;02:NMI
            DCD    HardFault_Handler  ;03:hard fault
            DCD    Dummy_Handler      ;04:(reserved)
            DCD    Dummy_Handler      ;05:(reserved)
            DCD    Dummy_Handler      ;06:(reserved)
            DCD    Dummy_Handler      ;07:(reserved)
            DCD    Dummy_Handler      ;08:(reserved)
            DCD    Dummy_Handler      ;09:(reserved)
            DCD    Dummy_Handler      ;10:(reserved)
            DCD    Dummy_Handler      ;11:SVCall (supervisor call)
            DCD    Dummy_Handler      ;12:(reserved)
            DCD    Dummy_Handler      ;13:(reserved)
            DCD    Dummy_Handler      ;14:PendableSrvReq (pendable request 
                                      ;   for system service)
            DCD    Dummy_Handler      ;15:SysTick (system tick timer)
            DCD    Dummy_Handler      ;16:DMA channel 0 xfer complete/error
            DCD    Dummy_Handler      ;17:DMA channel 1 xfer complete/error
            DCD    Dummy_Handler      ;18:DMA channel 2 xfer complete/error
            DCD    Dummy_Handler      ;19:DMA channel 3 xfer complete/error
            DCD    Dummy_Handler      ;20:(reserved)
            DCD    Dummy_Handler      ;21:command complete; read collision
            DCD    Dummy_Handler      ;22:low-voltage detect;
                                      ;   low-voltage warning
            DCD    Dummy_Handler      ;23:low leakage wakeup
            DCD    Dummy_Handler      ;24:I2C0
            DCD    Dummy_Handler      ;25:I2C1
            DCD    Dummy_Handler      ;26:SPI0 (all IRQ sources)
            DCD    Dummy_Handler      ;27:SPI1 (all IRQ sources)
            DCD    UART0_ISR          ;28:UART0 (status; error)
            DCD    Dummy_Handler      ;29:UART1 (status; error)
            DCD    Dummy_Handler      ;30:UART2 (status; error)
            DCD    Dummy_Handler      ;31:ADC0
            DCD    Dummy_Handler      ;32:CMP0
            DCD    Dummy_Handler      ;33:TPM0
            DCD    Dummy_Handler      ;34:TPM1
            DCD    Dummy_Handler      ;35:TPM2
            DCD    Dummy_Handler      ;36:RTC (alarm)
            DCD    Dummy_Handler      ;37:RTC (seconds)
            DCD    Dummy_Handler      ;38:PIT (all IRQ sources)
            DCD    Dummy_Handler      ;39:I2S0
            DCD    Dummy_Handler      ;40:USB0
            DCD    Dummy_Handler      ;41:DAC0
            DCD    Dummy_Handler      ;42:TSI0
            DCD    Dummy_Handler      ;43:MCG
            DCD    Dummy_Handler      ;44:LPTMR0
            DCD    Dummy_Handler      ;45:Segment LCD
            DCD    Dummy_Handler      ;46:PORTA pin detect
            DCD    Dummy_Handler      ;47:PORTC and PORTD pin detect
__Vectors_End
__Vectors_Size  EQU     __Vectors_End - __Vectors
            ALIGN
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
INSTRUCT DCB "Enter the access code.",0
SIZE DCB 32
DENY DCB "--Access Denied",0
GRANT DCB "--Access Granted",0
COMPLETE DCB "Mission Completed!",0
OPEN DCB "opensesame",0

;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
RxBuffer    SPACE   Q_BUF_SZ
            ALIGN
RxRecord    SPACE   Q_REC_SZ
            ALIGN
TxBuffer    SPACE   Q_BUF_SZ
            ALIGN
TxRecord    SPACE   Q_REC_SZ
            ALIGN
RunStopWatch    SPACE   SIZERSW
            ALIGN
QBuffer     SPACE   Q_BUF_SZ    ;Q contents
            ALIGN
QRecord     SPACE   Q_REC_SZ    ;Q management record
            ALIGN
String      SPACE   MAX_STRING
            ALIGN
Count       SPACE   4
;>>>>>   end variables here <<<<<
            ALIGN
            END