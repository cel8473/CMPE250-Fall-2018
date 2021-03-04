/*********************************************************************/
/* <Guessing game >                                   */
/* Name:  <Vikram Singh Chris Larson>                                           */
/* Date:  <12/4/2018>                                           */
/* Class:  CMPE 250                                                  */
/* Section:  <1>                                     */
/*-------------------------------------------------------------------*/
/* Template:  R. W. Melton                                           */
/*            March 30, 2018                                         */
/*********************************************************************/
#include "MKL46Z4.h"
#include "Exercise12_1.h"

#define FALSE      (0)
#define TRUE       (1)

#define MAX_STRING (79)
#define MAX_TIME		(11)
#define ONE_SEC			(1)
/* ADC0 */
#define ADC_MAX (0x3FFu)
#define ADC_VALUES (0x400u)
/* ------------------------------------------------------------------*/
/* ADC0_CFG1:  ADC0 configuration register 1                         */
/*  0-->31-8:(reserved):read-only:0                                  */
/*  1-->   7:ADLPC=ADC low-power configuration                       */
/* 10--> 6-5:ADIV=ADC clock divide select                            */
/*           Internal ADC clock = input clock / 2^ADIV               */
/*  1-->   4:ADLSMP=ADC long sample time configuration               */
/* 10--> 3-2:MODE=conversion mode selection                          */
/*           00=(DIFF'):single-ended 8-bit conversion                */
/*              (DIFF):differential 9-bit 2's complement conversion  */
/*           01=(DIFF'):single-ended 12-bit conversion               */
/*              (DIFF):differential 13-bit 2's complement conversion */
/*           10=(DIFF'):single-ended 10-bit conversion               */
/*              (DIFF):differential 11-bit 2's complement conversion */
/*           11=(DIFF'):single-ended 16-bit conversion               */
/*              (DIFF):differential 16-bit 2's complement conversion */
/* 01--> 1-0:ADICLK=ADC input clock select                           */
/*           00=bus clock                                            */
/*           01=bus clock / 2                                        */
/*           10=alternate clock (ALTCLK)                             */
/*           11=asynchronous clock (ADACK)                           */
/* BUSCLK = CORECLK / 2 = PLLCLK / 4                                 */
/* PLLCLK is 96 MHz                                                  */
/* BUSCLK is 24 MHz                                                  */
/* ADCinCLK is BUSCLK / 2 = 12 MHz                                   */
/* ADCCLK is ADCinCLK / 4 = 3 MHz                                    */
#define ADC0_CFG1_ADIV_BY2 (2u)
#define ADC0_CFG1_MODE_SGL10 (2u)
#define ADC0_CFG1_ADICLK_BUSCLK_DIV2 (1u)
#define ADC0_CFG1_LP_LONG_SGL10_3MHZ (ADC_CFG1_ADLPC_MASK | \
              (ADC0_CFG1_ADIV_BY2 << ADC_CFG1_ADIV_SHIFT) | \
                                     ADC_CFG1_ADLSMP_MASK | \
            (ADC0_CFG1_MODE_SGL10 << ADC_CFG1_MODE_SHIFT) | \
                               ADC0_CFG1_ADICLK_BUSCLK_DIV2)
/*-------------------------------------------------------------*/
/* ADC0_CFG2:  ADC0 configuration register 2                   */
/*  0-->31-8:(reserved):read-only:0                            */
/*  0--> 7-5:(reserved):read-only:0                            */
/*  0-->   4:MUXSEL=ADC mux select:  A channel                 */
/*  0-->   3:ADACKEN=ADC asynchronous clock output enable      */
/*  0-->   2:ADHSC=ADC high-speed configuration:  normal       */
/* 00--> 1-0:ADLSTS=ADC long sample time select (ADK cycles)   */
/*           default longest sample time:  24 total ADK cycles */
#define ADC0_CFG2_CHAN_A_NORMAL_LONG (0x00u)
/*---------------------------------------------------------   */
/* ADC0_SC1:  ADC0 channel status and control register 1      */
/*     0-->31-8:(reserved):read-only:0                        */
/*     0-->   7:COCO=conversion complete flag (read-only)     */
/*     0-->   6:AIEN=ADC interrupt enabled                    */
/*     0-->   5:DIFF=differential mode enable                 */
/* 10111--> 4-0:ADCH=ADC input channel select (23 = DAC0_OUT) */
#define ADC0_SC1_ADCH_AD23 (23u)
#define ADC0_SC1_SGL_DAC0 (ADC0_SC1_ADCH_AD23)
/*-----------------------------------------------------------*/
/* ADC0_SC2:  ADC0 status and control register 2             */
/*  0-->31-8:(reserved):read-only:0                          */
/*  0-->   7:ADACT=ADC conversion active                     */
/*  0-->   6:ADTRG=ADC conversion trigger select:  software  */
/*  0-->   5:ACFE=ADC compare function enable                */
/*  X-->   4:ACFGT=ADC compare function greater than enable  */
/*  0-->   3:ACREN=ADC compare function range enable         */
/*            0=disabled; only ADC0_CV1 compared             */
/*            1=enabled; both ADC0_CV1 and ADC0_CV2 compared */
/*  0-->   2:DMAEN=DMA enable                                */
/* 01--> 1-0:REFSEL=voltage reference selection:  VDDA       */
#define ADC0_SC2_REFSEL_VDDA (1u)
#define ADC0_SC2_SWTRIG_VDDA (ADC0_SC2_REFSEL_VDDA)
/*-------------------------------------------------------------*/
/* ADC0_SC3:  ADC0 status and control register 3               */
/* 31-8:(reserved):read-only:0                                 */
/*  0-->   7:CAL=calibration                                   */
/*          write:0=(no effect)                                */
/*                1=start calibration sequence                 */
/*          read:0=calibration sequence complete               */
/*               1=calibration sequence in progress            */
/*  0-->   6:CALF=calibration failed flag                      */
/* 00--> 5-4:(reserved):read-only:0                            */
/*  0-->   3:ADCO=ADC continuous conversion enable             */
/*           (if ADC0_SC3.AVGE = 1)                            */
/*  0-->   2:AVGE=hardware average enable                      */
/* XX--> 1-0:AVGS=hardware average select:  2^(2+AVGS) samples */
#define ADC0_SC3_SINGLE (0x00u)
#define ADC0_SC3_CAL (ADC_SC3_CAL_MASK | ADC_SC3_AVGE_MASK | \
                                           ADC_SC3_AVGS_MASK)
/*-------------------------------------------------------------*/
/* DAC */
#define DAC_DATH_MIN   0x00u
#define DAC_DATL_MIN   0x00u
/*---------------------------------------------------------------------*/
/* DAC_C0:  DAC control register 0                                   */
/* 1-->7:DACEN=DAC enabled                                             */
/* 1-->6:DACRFS=DAC reference select VDDA                              */
/* 0-->5:DACTRGSEL=DAC trigger select (X)                              */
/* 0-->4:DACSWTRG=DAC software trigger (X)                             */
/* 0-->3:LPEN=DAC low power control:high power                         */
/* 0-->2:(reserved):read-only:0                                        */
/* 0-->1:DACBTIEN=DAC buffer read pointer top flag interrupt enable    */
/* 0-->0:DACBBIEN=DAC buffer read pointer bottom flag interrupt enable */
#define DAC_C0_ENABLE  (DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK)
/*----------------------------------------*/
/* DAC_C1:  DAC control register 1      */
/* 0-->  7:DMAEN=DMA disabled             */
/* 0-->6-3:(reserved)                     */
/* 0-->  2:DACBFMD=DAC buffer mode normal */
/* 0-->  1:(reserved)                     */
/* 0-->  0:DACBFEN=DAC buffer disabled    */
#define DAC_C1_BUFFER_DISABLED  (0x00u)
/*-------------------------------------------------------------*/
/* PORTx_PCRn                                                  */
/*     -->31-25:(reserved):read-only:0                         */
/*    1-->   24:ISF=interrupt status flag (write 1 to clear)   */
/*     -->23-20:(reserved):read-only:0                         */
/* 0000-->19-16:IRQC=interrupt configuration (IRQ/DMA diabled) */
/*     -->15-11:(reserved):read-only:0                         */
/*  ???-->10- 8:MUX=pin mux control                            */
/*     -->    7:(reserved):read-only:0                         */
/*    ?-->    6:DSE=drive strengh enable                       */
/*     -->    5:(reserved):read-only:0                         */
/*    0-->    4:PFE=passive filter enable                      */
/*     -->    3:(reserved):read-only:0                         */
/*    0-->    2:SRE=slew rate enable                           */
/*    0-->    1:PE=pull enable                                 */
/*    x-->    0:PS=pull select                                 */
/* Port E */
#define PTE30_MUX_DAC0_OUT (0u << PORT_PCR_MUX_SHIFT)
#define SET_PTE30_DAC0_OUT (PORT_PCR_ISF_MASK | PTE30_MUX_DAC0_OUT)
#define PTE31_MUX_TPM0_CH4_OUT (3u << PORT_PCR_MUX_SHIFT)
#define SET_PTE31_TPM0_CH4_OUT (PORT_PCR_ISF_MASK | \
                                PTE31_MUX_TPM0_CH4_OUT)
/*-------------------------------------------------------*/
/* SIM_SOPT2                                             */
/*   -->31-28:(reserved):read-only:0                     */
/*   -->27-26:UART0SRC=UART0 clock source select         */
/* 01-->25-24:TPMSRC=TPM clock source select (MCGFLLCLK) */
/*   -->23-19:(reserved):read-only:0                     */
/*   -->   18:USBSRC=USB clock source select             */
/*   -->   17:(reserved):read-only:0                     */
/*  1-->   16:PLLFLLSEL=PLL/FLL clock select (MCGFLLCLK) */
/*   -->15- 8:(reserved):read-only:0                     */
/*   --> 7- 5:CLKOUTSEL=CLKOUT select                    */
/*   -->    4:RTCCLKOUTSEL=RTC clock out select          */
/*   --> 3- 0:(reserved):read-only:0                     */
#define SIM_SOPT2_TPMSRC_MCGPLLCLK (1u << SIM_SOPT2_TPMSRC_SHIFT)
#define SIM_SOPT2_TPM_MCGPLLCLK_DIV2 (SIM_SOPT2_TPMSRC_MCGPLLCLK | \
                                            SIM_SOPT2_PLLFLLSEL_MASK)
/*------------------------------------------------------------------*/
/* TPMx_CONF:  Configuration (recommended to use default values     */
/*     -->31-28:(reserved):read-only:0                              */
/* 0000-->27-24:TRGSEL=trigger select (external pin EXTRG_IN)       */
/*     -->23-19:(reserved):read-only:0                              */
/*    0-->   18:CROT=counter reload on trigger                      */
/*    0-->   17:CSOO=counter stop on overflow                       */
/*    0-->   16:CSOT=counter stop on trigger                        */
/*     -->15-10:(reserved):read-only:0                              */
/*    0-->    9:GTBEEN=global time base enable                      */
/*     -->    8:(reserved):read-only:0                              */
/*   00-->  7-6:DBGMODE=debug mode (paused in debug)                */
/*    0-->    5:DOZEEN=doze enable                                  */
/*     -->  4-0:(reserved):read-only:0                              */
/* 15- 0:COUNT=counter value (writing any value will clear counter) */
#define TPM_CONF_DEFAULT (0u)
/*------------------------------------------------------------------*/
/* TPMx_CNT:  Counter                                               */
/* 31-16:(reserved):read-only:0                                     */
/* 15- 0:COUNT=counter value (writing any value will clear counter) */
#define TPM_CNT_INIT (0u)
/*------------------------------------------------------------------------*/
/* TPMx_MOD:  Modulo                                                      */
/* 31-16:(reserved):read-only:0                                           */
/* 15- 0:MOD=modulo value (recommended to clear TPMx_CNT before changing) */
/* Period = 3 MHz / 50 Hz */
#define TPM_MOD_PWM_PERIOD_20ms (60000u)
/*------------------------------------------------------------------*/
/* TPMx_CnSC:  Channel n Status and Control                         */
/* 0-->31-8:(reserved):read-only:0                                  */
/* 0-->   7:CHF=channel flag                                        */
/*              set on channel event                                */
/*              write 1 to clear                                    */
/* 0-->   6:CHIE=channel interrupt enable                           */
/* 1-->   5:MSB=channel mode select B (see selection table below)   */
/* 0-->   4:MSA=channel mode select A (see selection table below)   */
/* 1-->   3:ELSB=edge or level select B (see selection table below) */
/* 0-->   2:ELSA=edge or level select A (see selection table below) */
/* 0-->   1:(reserved):read-only:0                                  */
/* 0-->   0:DMA=DMA enable                                          */
#define TPM_CnSC_PWMH (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK)
/*--------------------------------------------------------*/
/* TPMx_CnV:  Channel n Value                             */
/* 31-16:(reserved):read-only:0                           */
/* 15- 0:MOD (all bytes must be written at the same time) */
/*--------------------------------------------------------------*/
/* TPMx_SC:  Status and Control                                 */
/*    -->31-9:(reserved):read-only:0                            */
/*   0-->   8:DMA=DMA enable                                    */
/*    -->   7:TOF=timer overflow flag                           */
/*   0-->   6:TOIE=timer overflow interrupt enable              */
/*   0-->   5:CPWMS=center-aligned PWM select (edge align)      */
/*  01--> 4-3:CMOD=clock mode selection (count each TPMx clock) */
/* 100--> 2-0:PS=prescale factor selection                      */
/*    -->        can be written only when counter is disabled   */
#define TPM_SC_CMOD_CLK (1u)
#define TPM_SC_PS_DIV16 (0x4u)
#define TPM_SC_CLK_DIV16 ((TPM_SC_CMOD_CLK << TPM_SC_CMOD_SHIFT) | \
                          TPM_SC_PS_DIV16)
/*- -----*/
/* Servo */
#define SERVO_POSITIONS  (5)
int getRandom(int count){
    return count%4;
}

int score(int rand, int round, char input, int time){
	/*Calculate the score by determinig if input is valid the percent of time used and the round it was*/
	if(input == 'n' || input == 'N'){
		if(rand != 0){
			return 0;
		}
		else{
			return (round);
		}
	}
	else if(input == 'b' || input == 'B'){
		if(rand != 3){
			return 0;
		}
		else{
			return round;
			}
	}
	else if(input == 'r' || input == 'R'){
		if(rand != 2){
			return 0;
		}
		else{
			return round;
			}
	}
	else if(input == 'g' || input == 'G'){
		if(rand != 1){
			return 0;
		}
		else{
			return round;
			}
	}
	else{
		return 0;
	}
}
void putcolor(int random){
	if(random == 0){
		PutStringSB("None", MAX_STRING);
	}
	else if(random == 1){
		PutStringSB("Green",MAX_STRING);
	}
	else if(random == 2){
		PutStringSB("Red",MAX_STRING);}
	else{
		PutStringSB("Both", MAX_STRING);
	}
}
int main (void) {
	char input = 'X';
	int random;
	int count;
    int Round;
	int Score = 0;
	int s;
	int key;
  __asm("CPSID   I");  /* mask interrupts */
  /* Perform all device initialization here */
  /* Before unmasking interrupts            */
	Init_UART0_IRQ();
    Init_PIT_IRQ();
    Init_LED();
	
  __asm("CPSIE   I");  /* unmask interrupts */

  for (;;) { /* do forever */
		/* Game instruction*/
		PutStringSB("The game consists of ten rounds of digital value matching.",MAX_STRING);
		NewLine();
        PutStringSB( "In each round, the game program lights the LEDs on the  I",MAX_STRING);
        NewLine();
		PutStringSB("Freedom board to a random combination of colors from none, ",MAX_STRING);
		NewLine();
        PutStringSB("red, green, and both, and the user has to type the first",MAX_STRING);
        NewLine();
		PutStringSB("character of the color, 'r' or 'R' for red, 'g' or 'G' ",MAX_STRING); 
		NewLine();
        PutStringSB("for green, 'n' or 'N' for none, and 'B' or 'b' for both. Each",MAX_STRING);
        NewLine();
		PutStringSB("round lasts 11-the round that it is in seconds and ends", MAX_STRING); 
		NewLine();
        PutStringSB("when time runs out or when a guess occurrs. Good Luck!!!!!!!", MAX_STRING);
        NewLine();
		PutStringSB("TO START THE GAME PRESS ANY KEY",MAX_STRING);
		/*Start play*/
		GetChar();
        StartStopWatch();
        Round = 1;
		/*Run through ten rounds of play*/
		for(int x = 0; x<10;x++){
            
            NewLine();
			PutStringSB("Round ", MAX_STRING);
			PutNumU(Round);
			NewLine();
            count = getCount();
			StartStopWatch();
            random = getRandom(count);
			/*Get the random value and configure the leds to the value*/
			if(random == 0){
				Turn_OFF();
			}
			else if(random == 1){
				Turn_G();
			}
			else if(random == 2){
				Turn_R();
			}
			else{
				Turn_B();
			}
			PutStringSB(">",MAX_STRING);
			/*Start the stop watch and have user guess until time runs out*/
			while(getCount()<(11-Round)*100){
				key = KeyPressed();
				if(key != 0){
					input = GetChar();
					break;
				}
			}
		
			/*Calculate the score and let the user know what it is before prompting the user to 
			 enter any key to start the next round*/
			count = getCount();
			PutChar(input);
			s = score(random,Round,input,count);
			if(count >= (MAX_TIME-Round)*100){
				PutStringSB(":	Out of time--color was ",MAX_STRING);
				putcolor(random);
				NewLine();
			}
			else if (s == 0){
				PutStringSB(":	Wrong ",MAX_STRING);
                putcolor(random);
				NewLine();
			}
			else{
				PutStringSB(":	Correct color was ",MAX_STRING);
				putcolor(random);
                Score += s;
				NewLine();
			}
			PutStringSB("Score total:",MAX_STRING);
			PutNumU(Score);
			NewLine();
			Round+=1;
			PutStringSB("Press a key to continue",MAX_STRING);
			GetChar();
			NewLine();
		}
		PutStringSB("Final Score: ",MAX_STRING);
		PutNumU(Score);
        NewLine();
  } /* do forever */

  return (0);
} /* main */
