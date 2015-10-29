 /*----------------------------------------------------------------------------
 * The SysTick interrupt example code is borrowed
 * from the uVision/ARM development tools.
 * The code was modified to include other interrupts and FlexTimers
 *----------------------------------------------------------------------------*/

#include "MK64F12.h"
#include "FlexTimer.h"
#include "PIT_Timer.h"
#include "programmable_delay.h"
#include <stdio.h>
#include <stdint.h>
#include "main.h"



uint32_t LEDOn, LEDOff;
uint16_t adc_data;
uint32_t TempVar = 0;


extern uint8_t Flex_Timer_Test(void);

/*----------------------------------------------------------------------------
  Function that initializes LEDs
 *----------------------------------------------------------------------------*/
void LED_Initialize(void) {

  SIM->SCGC5     |= ((1UL <<  13) | (1UL <<  10));     /* Enable Port B,E Clock    */

  // Old code from Keil
  //PORTB->PCR[22]  = (1UL <<  8);                     /* PTB22 is GPIO pin ALT 1  */
  //PORTB->PCR[21]  = (1UL <<  8);                     /* PTB21 is GPIO pin ALT 1  */
  //PORTE->PCR[26]  = (1UL <<  8);                     /* PTE26 is GPIO pin ALT 1  */
  PORTB_PCR22 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
  PORTB_PCR21 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
  PORTE_PCR26 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;

  // Old code from Keil
  //PTB->PDDR = (led_mask[1] | led_mask[0]);			/* PTB->PDDR is same as GPIOB_PDDR */
  //PTE->PDDR = led_mask[2];
  //Set GPIO Direction
  BITBAND_REG(GPIOB_PDDR,22) = 1;  						//Same as GPIOB_PDDR = GPIO_PDDR_PDD(22);
  BITBAND_REG(GPIOB_PDDR,21) = 1;
  BITBAND_REG(GPIOE_PDDR,26) = 1;

  // Old code from Keil
  // Switch LEDs off
  //PTB->PDOR = (led_mask[1] | led_mask[0]);			/* PTB->PDOR is same as GPIOB_PDOR */
  //PTE->PDOR = led_mask[2];
  BITBAND_REG(GPIOB_PDOR,22) = 1;  						//Same as GPIOA_PDOR = GPIO_PDOR_PDO(22);
  BITBAND_REG(GPIOB_PDOR,21) = 1;
  BITBAND_REG(GPIOE_PDOR,26) = 1;

}

#define NUM_LEDS  3                     /* Number of LEDs                     */

const uint32_t led_mask[] = { 1UL << 22,
                              1UL << 21,
                              1UL << 26 };

GPIO_Type * led_addr[]    = { PTB,		//Note PTB is same as ((GPIO_Type *)PTB_BASE)
                              PTB,
                              PTE };    //Note PTE is same as ((GPIO_Type *)PTE_BASE)

/*----------------------------------------------------------------------------
  Function that turns on requested LED
 *----------------------------------------------------------------------------*/
void LED_On (unsigned int idx) {
  if (idx < NUM_LEDS) {
    led_addr[idx]->PCOR = led_mask[idx];
  }
}

/*----------------------------------------------------------------------------
  Function that turns off requested LED
 *----------------------------------------------------------------------------*/
void LED_Off (unsigned int idx) {
  if (idx < NUM_LEDS) {
    led_addr[idx]->PSOR = led_mask[idx];
  }
}

/*----------------------------------------------------------------------------
  Main function
 *----------------------------------------------------------------------------*/
typedef enum {DISCOVERY, ACCURACY, SPEED} mode;
int32_t num = 0;
int32_t count = 0;
mode state = DISCOVERY;
mode statePrevious = SPEED;
int main (void) {
  // Configure board specific pin muxing
  hardware_init();

  // Initialize UART terminal
  dbg_uart_init();

  PRINTF("\r\nRunning the myProject project.\r\n");

  SystemCoreClockUpdate();                 /* Update system core clock       */
  SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK
  			| SIM_SCGC5_PORTC_MASK   | SIM_SCGC5_PORTD_MASK)
  			| SIM_SCGC5_PORTE_MASK;		    /* Enable Port A, B, C, D & E Clocks */
  LED_Initialize();                         /* LED Initialization             */
  InitFlexTimer0CH0();
  PORTC_PCR1 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK; // PTC1 ALT4// DSE = 1, High drive strength
//	FTM0_SC |= FTM_SC_PS(FTM0_CH0_CLK_PRESCALE); PRESCALER
//	FTM0_MOD = FTM0_CH0_MOD_VALUE;    			 MOD VALUE
//  FTM0_C0V = FTM0_CH0_MOD_VALUE/20;    		 DUTY CYCLE

  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC_PCR6 = 0x90100 | PORT_PCR_DSE_MASK;
  GPIOC_PDDR |= (0 << 6);
  PORTC_ISFR = PORT_ISFR_ISF(0x40);
  NVIC_EnableIRQ(PORTC_IRQn);

  while (1){
	  switch (state) {
	  case DISCOVERY:
		  if (state!=statePrevious && statePrevious==SPEED) {
			  statePrevious = state;
			  LED_On(state);
			  printf("DISCOVERY MODE!\r\n");
			  // CHANGE PWM
			  FTM0_MOD = FTM0_CH0_MOD_VALUE;
			  FTM0_C0V = FTM0_CH0_MOD_VALUE/33.333;
		  }
		  break;
	  case ACCURACY:
		  if (state!=statePrevious && statePrevious==DISCOVERY) {
			  statePrevious = state;
			  LED_On(state);
			  printf("ACCURACY MODE!\r\n");
			  // CHANGE PWM
			  FTM0_MOD = FTM0_CH0_MOD_VALUE;
			  FTM0_C0V = FTM0_CH0_MOD_VALUE/19.5; //13.333
		  }
		  break;
	  case SPEED:
		  if (state!=statePrevious && statePrevious==ACCURACY) {
			  statePrevious = state;
			  LED_On(state);
			  printf("SPEED MODE!\r\n");
			  // CHANGE PWM
			  FTM0_MOD = FTM0_CH0_MOD_VALUE;
			  FTM0_C0V = FTM0_CH0_MOD_VALUE/13.33; // 8.333
		  }
		  break;
	  default: break;
	  }
  }
}

void PORTC_IRQHandler(void) {
	LED_Off(state);
	state = count++%3;
	PORTC_ISFR = PORT_ISFR_ISF(0x40);
}
