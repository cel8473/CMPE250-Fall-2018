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
CR          EQU  0x0D
LF          EQU  0x0A
NULL        EQU  0x00
HexL	    EQU     4
SHIFT	    EQU		24
ALPHA	    EQU		55		
NUM_C	    EQU		48
Lenth	    EQU		79
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port A
PORT_PCR_SET_PTA1_UART0_RX EQU (PORT_PCR_ISF_MASK :OR: PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTA2_UART0_TX EQU (PORT_PCR_ISF_MASK :OR: PORT_PCR_MUX_SELECT_2_MASK)
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
; (PLLFLLSEL determines MCGFLLCLK' or MCGPLLCLK/2)
; 1= 16:PLLFLLSEL=PLL/FLL clock select (MCGPLLCLK/2)
SIM_SOPT2_UART0SRC_MCGPLLCLK EQU (1 << SIM_SOPT2_UART0SRC_SHIFT)
SIM_SOPT2_UART0_MCGPLLCLK_DIV2 EQU (SIM_SOPT2_UART0SRC_MCGPLLCLK :OR: SIM_SOPT2_PLLFLLSEL_MASK)
;---------------------------------------------------------------
;SIM_SOPT5
; 0-> 16:UART0 open drain enable (disabled)
; 0-> 02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR EQU (SIM_SOPT5_UART0ODE_MASK :OR: SIM_SOPT5_UART0RXSRC_MASK :OR: SIM_SOPT5_UART0TXSRC_MASK)
;--------------------------------------------------------------- 

;---------------------------------------------------------------
;UART0_BDH
; 0-> 7:LIN break detect IE (disabled)
; 0-> 6:RxD input active edge IE (disabled)
; 0-> 5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (BUSCLK / [9600 * (OSR + 1)])
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;BUSCLK is 24 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDH_9600 EQU 0x01
;---------------------------------------------------------------
;UART0_BDL
;26->7-0:SBR[7:0] (BUSCLK / [9600 * (OSR + 1)])
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;BUSCLK is 24 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600 EQU 0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select
; (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1 EQU 0x00
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
UART0_C2_T_R EQU (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
; 10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
; 10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
; (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV EQU 0x00
;--------------------------------------------------------------- 

;UART0_C4
; 0--> 7:MAEN1=match address mode enable 1 (disabled)
; 0--> 6:MAEN2=match address mode enable 2 (disabled)
; 0--> 5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
; = 1 + OSR for 3 <= OSR <= 31
; = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16 EQU 0x0F
UART0_C4_NO_MATCH_OSR_16 EQU UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
; 0--> 7:TDMAE=transmitter DMA enable (disabled)
; 0--> 6:Reserved; read-only; always 0
; 0--> 5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
; 0--> 1:BOTHEDGE=both edge sampling (rising edge only)
; 0--> 0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC EQU 0x00
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
UART0_S1_CLEAR_FLAGS EQU (UART0_S1_IDLE_MASK :OR: UART0_S1_OR_MASK :OR: UART0_S1_NF_MASK :OR: UART0_S1_FE_MASK :OR: UART0_S1_PF_MASK)
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
; write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
; write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS EQU (UART0_S2_LBKDIF_MASK :OR: UART0_S2_RXEDGIF_MASK)
;****************************************************************
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
					BL		INIT		;initialize device
main_Loop			LDR		R0,=EnterHex			;First string print
FirstNumG			MOVS	R1,#Lenth
					BL		PutString
					MOVS	R1,#HexL				;Get hex value and determine if it is valid
					LDR		R0,=Hex1
					BL		GetHexIntMulti
					BCS		Error
					LDR		R0,=EnterHex2			;Get second hex value and determine if it is valid
SecondNumG			MOVS	R1,#Lenth
					BL		PutString
					MOVS	R1,#HexL
					LDR		R0,=Hex2
					BL		GetHexIntMulti
					BCS		Error2
					MOVS	R1,#Lenth				;Determine sum and see if it is valid, overflow
					LDR		R0,=String
					LDR		R0,=SumS
					BL		PutString
					LDR		R2,=Hex2
					MOVS	R3,#HexL
					LDR		R1,=Hex1
					LDR		R0,=SUM
					BL		AddIntMulti
					BCS		Ovfl
					MOVS	R1,#HexL
					LDR		R0,=SUM
					BL		PutHexIntMulti
					BL		NewLine
					B		main_Loop				;Restart loop
Error				LDR		R0,=Invalid				;First hex was invalid
					B		FirstNumG
Error2				LDR		R0,=Invalid				;Second hex was invalid
					B		SecondNumG
Ovfl				LDR		R0,=Overflow			;Overflow occured while adding
					BL		PutString
					BL		NewLine
					B		main_Loop
;>>>>>   end main program code <<<<<
            B       .
            ENDP
;>>>>> begin subroutine code <<<<<
;*************************************************************************
;Adds the n-word unsigned number in memory starting at the address
;in R2 to the n-word unsigned number in memory starting at the address in R1, and
;stores the result to memory starting at the address in R0, where the value in R3 is n.
;Thee subroutine uses ADCS to add word by word, starting from the least significant
;words of the augend and addend and working to the most significant words. If the
;result is a valid n-word unsigned number, it returns with the APSR C bit clear as the
;return code for success; otherwise it returns with the APSR C bit set as the return
;code for overflow
AddIntMulti		PROC	{R0-R14}
				PUSH	{R1-R7,LR}	
				LSLS	R3,R3,#3		;Shift word amount to multiply by 8
				MOVS	R4,#0			;Set counter/Index as r4
				SUBS	R4,R4,#1
				PUSH	{R0}
				MOVS	R0,#0			;R0 is carry over and equals 0
AddLoop			CMP		R4,R3			;Add each component of R1 and R2
				BGE		CompleteAdd
				LDRB	R5,[R1,R3]
				LDRB	R6,[R2,R3]
				ADDS	R7,R5,R6
				ADDS	R7,R7,R0		;Add the carry over to the result
				CMP		R7,#15
				BGT		Carry			;Determine if there is carry over to the next bit
				POP		{R0}
				STRB	R7,[R0,R3]		;Store result in SUM
				PUSH	{R0}
				MOVS	R0,#0
				SUBS	R3,R3,#1		;Increment by 1
				B		AddLoop
Carry			SUBS	R7,R7,#16		;Carry over occured set R0 to 1 and determine value of current bit
				POP		{R0}
				STRB	R7,[R0,R3]
				SUBS	R3,R3,#1
				PUSH	{R0}
				MOVS	R0,#1
				B		AddLoop
CompleteAdd		CMP		R0,#1			;Determine if overflow occurred
				BEQ		Ovflow
				PUSH	{R3-R4}
				MRS		R4,APSR					;clear the C flag
				MOVS	R3,#0x20
				LSLS	R3,R3,#SHIFT
				BICS	R4,R4,R3
				MSR		APSR,R4
				POP		{R3-R4}
				B		EndAdd
Ovflow			PUSH	{R2-R4}
				MRS		R2,APSR					;set the C flag
				MOVS	R3,#0x20
				LSLS	R3,R3,#SHIFT
				ORRS	R4,R4,R3
				MSR		APSR,R4
				POP		{R2-R4}
EndAdd			POP		{R0-R7,PC}
				BX		LR
				ENDP
;****************************************************************
;Gets an n-word unsigned number from the user typed in text
;hexadecimal representation, and stores it in binary in memory starting at the address
;in R0, where the value in R1 is n.  e subroutine reads characters typed by the user
;until the enter key is pressed by calling the subroutine GetStringSB. It then converts
;the ASCII hexadecimal representation input by the user to binary, and it stores the
;binary value to memory at the address specified in R0. If the result is a valid n-word
;unsigned number, it returns with the APSR C bit clear; otherwise, it returns with the
;APSR C bit set.
GetHexIntMulti	PROC	{R0-R14}
				PUSH	{R1-R4,LR}
				LSLS	R1,R1,#3				;Shift word value to multiply by 8
				ADDS	R1,R1,#1				;Add one
				BL		GetString				;Get hex number
				SUBS	R1,R1,#2				;Determine if each character
CheckNum		CMP		R1,#0					;Is numeric or alpha, upper or lower case
				BLT		EndClear
				LDRB	R2,[R0,R1]
				CMP		R2,#48
				BLT		NotHex
				CMP		R2,#57
				BGT		Alpha
				SUBS	R2,R2,#48				;change numeric character to binary
continueC		STRB	R2,[R0,R1]				;Store binary value of character
				SUBS	R1,R1,#1
				B		CheckNum
NotHex			PUSH	{R2-R4}					;If the character is not a hex value return with the clear flag set.
				MRS		R2,APSR					;set the C flag
				MOVS	R3,#0x20
				LSLS	R3,R3,#SHIFT
				ORRS	R4,R4,R3
				MSR		APSR,R4
				POP		{R2-R4}
				B		EndGetHex
Alpha			CMP		R2,#65					;Check if characters are uppercase alpha
				BLT		NotHex
				CMP		R2,#70
				BGT		SmallAlpha
				SUBS	R2,R2,#ALPHA			;Change uppercase alpha to binary
				B		continueC
SmallAlpha		CMP		R2,#97					;Check if characters are lower case
				BLT		NotHex
				CMP		R2,#102
				BGT		NotHex
				SUBS	R2,R2,#87				;Change lower case alpha to binary
				B		continueC
EndClear		PUSH	{R3-R4}
				MRS		R4,APSR					;clear the C flag
				MOVS	R3,#0x20
				LSLS	R3,R3,#SHIFT
				BICS	R4,R4,R3
				MSR		APSR,R4
				POP		{R3-R4}
EndGetHex		POP		{R1-R4,PC}
				BX		LR
				ENDP
;*************************************************************************
;Outputs an n-word unsigned number, from memory starting at the
;address in R0, to the terminal in text hexadecimal representation using 8n hex digits,
;where the value in R1 is n.
PutHexIntMulti	PROC	{R0-R14}
				PUSH	{R0-R5,LR}
				MOVS	R4,#0
				MOVS	R3,#0
				MOVS	R6,#0
				LSLS	R1,R1,#3
				MOVS	R5,#28
				SUBS	R1,R1,#1
PutHexLoop		LDRB	R2,[R0,R6]
				LSLS	R2,R2,R5
				ADDS	R4,R2,R4
				ADDS	R6,R6,#1
				ADDS	R3,R3,#1
				SUBS	R5,R5,#4
				CMP		R3,#7
				BGT		PutHexPrint
				B		PutHexLoop
PutHexPrint		PUSH	{R0}
				MOVS	R0,R4
				BL		PutNumHex
				POP		{R0}
				MOVS	R3,#0
				MOVS	R4,#0
				MOVS	R5,#28
				CMP		R1,R6
				BGE		PutHexLoop
				POP		{R0-R5,PC}
				BX		LR	
				ENDP

;*************************************************************************
;Prints to the terminal screen the text hexadeimcal of the unsigned byte value in R0
PutNumHex PUSH	{R0 - R3, LR}
        MOVS	R3,#28		        ;Initialize Counter
		MOVS	R2,R0               ;Saves R0 to R2
        
PNHLoop MOVS	R0,R2               ;R0 <- R2
		LDR		R1,=0x0000000F      ;Gets a single char from hex number and displays it
		LSLS	R1,R1,R3
		ANDS	R0,R0,R1
		LSRS	R0,R0,R3
		CMP		R0,#9
		BLE		PNHNumber		    ;If digit is numeric
		ADDS	R0,R0,#55           ;Value to go from 0xA to 'A'
		BAL		PNHPrint            ;If the digit is alphabetic
		
PNHNumber ADDS	R0,R0,#48           ;Value to go from 0x0 to 0

PNHPrint BL		PutChar             ;Prints the character
		SUBS	R3,R3,#4			;Counter--
		BLO		PNHDone             
		BAL		PNHLoop
        
PNHDone POP		{R0 - R3, PC}
;****************************************************************
PutNumUB PROC    {R0-R14}           
        PUSH	{R1,LR}
        MOVS    R1,#0xFF            ;Prints to the terminal screen the decimal representation of the unsigned world in R0
        ANDS    R0,R0,R1            ;Masks the word
        B PutNumU
        POP     {R1,PC}
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
INIT PROC {R0-R14}										;Procs R0-R14
	 PUSH {R0-R3}										;Pushes R0-R3 to the stack
;Select MCGPLLCLK / 2 as UART0 clock source				
	LDR 	R0,=SIM_SOPT2									
	LDR 	R1,=SIM_SOPT2_UART0SRC_MASK						
	LDR 	R2,[R0,#0]										
	BICS 	R2,R2,R1										
	LDR 	R1,=SIM_SOPT2_UART0_MCGPLLCLK_DIV2
	ORRS 	R2,R2,R1
	STR 	R2,[R0,#0]
;Enable external connection for UART0
	LDR R0,=SIM_SOPT5
	LDR R1,= SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
	LDR R2,[R0,#0]
	BICS R2,R2,R1
	STR R2,[R0,#0]
;Enable clock for UART0 module
	LDR R0,=SIM_SCGC4
	LDR R1,= SIM_SCGC4_UART0_MASK
	LDR R2,[R0,#0]
	ORRS R2,R2,R1
	STR R2,[R0,#0]
;Enable clock for Port A module
	LDR R0,=SIM_SCGC5
	LDR R1,= SIM_SCGC5_PORTA_MASK
	LDR R2,[R0,#0]
	ORRS R2,R2,R1
	STR R2,[R0,#0]
;Connect PORT A Pin 1 (PTA1) to UART0 Rx (J1 Pin 02)
	LDR R0,=PORTA_PCR1
	LDR R1,=PORT_PCR_SET_PTA1_UART0_RX
	STR R1,[R0,#0]
;Connect PORT A Pin 2 (PTA2) to UART0 Tx (J1 Pin 04)
	LDR R0,=PORTA_PCR2
	LDR R1,=PORT_PCR_SET_PTA2_UART0_TX
	STR R1,[R0,#0] 
;Disable UART0 receiver and transmitter
	LDR R0,=UART0_BASE
	MOVS R1,#UART0_C2_T_R
	LDRB R2,[R0,#UART0_C2_OFFSET]
	BICS R2,R2,R1
	STRB R2,[R0,#UART0_C2_OFFSET]
;Set UART0 for 9600 baud, 8N1 protocol
	MOVS R1,#UART0_BDH_9600
	STRB R1,[R0,#UART0_BDH_OFFSET]
	MOVS R1,#UART0_BDL_9600
	STRB R1,[R0,#UART0_BDL_OFFSET]
	MOVS R1,#UART0_C1_8N1
	STRB R1,[R0,#UART0_C1_OFFSET]
	MOVS R1,#UART0_C3_NO_TXINV
	STRB R1,[R0,#UART0_C3_OFFSET]
	MOVS R1,#UART0_C4_NO_MATCH_OSR_16
	STRB R1,[R0,#UART0_C4_OFFSET]
	MOVS R1,#UART0_C5_NO_DMA_SSR_SYNC
	STRB R1,[R0,#UART0_C5_OFFSET]
	MOVS R1,#UART0_S1_CLEAR_FLAGS
	STRB R1,[R0,#UART0_S1_OFFSET]
	MOVS R1,#UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
	STRB R1,[R0,#UART0_S2_OFFSET]
;Enable UART0 receiver and transmitter
	MOVS R1,#UART0_C2_T_R
	STRB R1,[R0,#UART0_C2_OFFSET] 
	POP{R0-R3}											;Pops R0-R3 from the stack
	BX LR												;Branch exchange the list register
	ENDP	
;****************************************************************
;Gets a character from the Putty line that the user typed in
GetChar PROC 	{R1-R14}			    ;Procs R0-R14
		PUSH	{R1-R3}			        ;Pushes R0-R3 to the stack
;Poll RDRF until UART0 ready to receive           
		LDR   	R1,=UART0_BASE
		MOVS  	R2,#UART0_S1_RDRF_MASK         
PollRx  LDRB    R3,[R1,#UART0_S1_OFFSET]          
		ANDS  	R3,R3,R2         
		BEQ   	PollRx 
;Receive character and store in R0         
		LDRB  	R0,[R1,#UART0_D_OFFSET]
		POP	{R1-R3}				         ;Pops R0-R3 from the stack
		BX LR					         ;Branch exchange the list register
		ENDP
;****************************************************************
;Puts the character stored in R0 onto the Putty line
PutChar	PROC 	{R1-R14}			    ;Procs R0-R14
		PUSH	{R1-R3}				    ;Pushes R0-R3 to the stack
;Poll TDRE until UART0 ready to transmit           
		LDR   	R1,=UART0_BASE         
		MOVS  	R2,#UART0_S1_TDRE_MASK
PollTx  LDRB    R3,[R1,#UART0_S1_OFFSET]          
		ANDS  	R3,R3,R2         
		BEQ   	PollTx 
;Transmit character stored in R0         
		STRB  	R0,[R1,#UART0_D_OFFSET] 
		POP	    {R1-R3}			        ;Pops R0-R3 from the stack
		BX LR					        ;Branch exchange the list register
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
		PUSH	{R0-R5,LR}			    ;Pushes R1-R5 and LR to the stack
		MOVS	R2,R0				    ;Sets R2 to the address stored in R0
        CMP     R1,#0
        BEQ Endput
        ADDS    R1,R1,R2
Loading	LDRB    R0,[R2,#0]			    ;Loads R2 with an offset of R4 into R0
		CMP	    R0,#0			        ;Compares R0 to 0, it ends when the program finds a zero
		BEQ	Endput			            ;If 0 end program
		BL 	PutChar			    	    ;Runs PutChar
        ADDS    R2,R2,#1
        CMP     R2,R1
        BGE Endput
		B	Loading				
Endput	POP	{R0-R5,PC}			        ;Pops R1-R5 and PC to the stack
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
            DCD    Dummy_Handler      ;28:UART0 (status; error)
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
EnterHex	DCB		" Enter first 128-bit hex number:	0x",0
EnterHex2	DCB		"Enter 128-bit hex number to add:	0x",0
Invalid		DCB		"	     Invalid number--try again:	0x",0
SumS		DCB		"                            Sum: 0x",0
Overflow	DCB		"OVERFLOW",0
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
String		SPACE	79
Hex1		SPACE	16*HexL
			ALIGN
Hex2		SPACE	16*HexL
			ALIGN
SUM			SPACE	16*HexL
;>>>>>   end variables here <<<<<
            ALIGN
            END