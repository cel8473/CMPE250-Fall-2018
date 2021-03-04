            TTL Program Title for Listing Header Goes Here
;****************************************************************
;Name:  Chris Larson
;Date:  2 October 2018
;Class:  CMPE-250
;Section:  Section 2, Tuesday 2pm
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
MAX_STRING EQU 79
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
;--------------------------------------------------------------- 
;****************************************************************
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
            IMPORT  Startup
            IMPORT  LengthStringSB
Reset_Handler  PROC  {}
main
;---------------------------------------------------------------
;Mask interrupts
             CPSID   I
;KL46 system startup with 48-MHz system clock
            BL      Startup
;---------------------------------------------------------------
;
;>>>>> begin main program code <<<<<
;Starts and setups the UART0
;Prompts the user for a letter
;Depending on the letter (G,I,L,P) runs a subroutine specific to the letter
;G stores a letter in the string variable
;I clears the string variable
;L prints the length of the string variable
;P prints the string variable
        BL INIT
 
Start   LDR R0,=INSTRUCT
        BL PutString	;Puts the instructions out and waits for a character
        
GetLoop BL GetChar
        CMP R0,#71		;If (R0 = g)
        BEQ G
        CMP R0,#103		;If (R0 = G)
        BEQ G
        CMP R0,#73		;If (R0 = i)
        BEQ I
        CMP R0,#105		;If (R0 = I)
        BEQ I
        CMP R0,#76		;If (R0 = l)
        BEQ L
        CMP R0,#108		;If (R0 = L)
        BEQ L
        CMP R0,#80		;If (R0 = p)
        BEQ P
        CMP R0,#112		;If (R0 = P)
        BEQ P
        B   GetLoop		;Else tries to get another char
        
G       MOVS R0,#71		;Displays a capital G
        BL PutChar
        BL NewLine
        MOVS R0,#'<'	;Displays a < on a new line to show it is ready to store
        BL PutChar
        LDR R0,=String	
        BL GetString	;Gets the String from the user and stores to the String variable
        B Start			;Goes back to Start to wait for the next command
        
I       MOVS R0,#73		;Displays a capital I
        BL PutChar
        MOVS R2,#0
        MOVS R3,#0
        LDR R0,=String	;Gets the String address
iLoop   LDRB R5,[R0,R2]	;Clears until it reaches a null character
        CMP R5,R3
        BEQ ExitI
        STRB R3,[R0,R2]	;Stores it back to the String variable
        ADDS R2,R2,#1	;Adds 1 to the offset to clear the next character
        B iLoop
ExitI   BL NewLine		
        B Start			;Goes back to Start to wait for the next command

L       MOVS R0,#76		;Displays a capital L
        BL PutChar
        BL NewLine
        LDR R0,=LENGTH	;Displays the "Length:" string
        BL PutString
		LDR R0,=String
		BL LengthStringSB
        BL PutNumU		;Calculates and displays the length of the String variable
        BL NewLine
        B Start			;Goes back to Start to wait for the next command
        
P       MOVS R0,#80		;Displays a capital P
        BL PutChar
        BL NewLine
        MOVS R0,#'>'	;Displays the String variable with ">" around the string
        BL PutChar
        LDR R0,=String
        BL PutString
        MOVS R0,#'>'
        BL PutChar
        BL NewLine
        B Start			;Goes back to Start to wait for the next command
  
        
;>>>>>   end main program code <<<<<
;Stay here
            B       .
            ENDP
;>>>>> begin subroutine code <<<<<
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
	LDR R0,=SIM_SOPT2									
	LDR R1,=SIM_SOPT2_UART0SRC_MASK						
	LDR R2,[R0,#0]										
	BICS R2,R2,R1										
	LDR R1,=SIM_SOPT2_UART0_MCGPLLCLK_DIV2
	ORRS R2,R2,R1
	STR R2,[R0,#0]
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
INSTRUCT DCB "Type a string command(g,p,l,i):",0
SIZE DCB 32
LENGTH DCB "Length:",0
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
String      SPACE   MAX_STRING  
;>>>>>   end variables here <<<<<
            ALIGN
            END