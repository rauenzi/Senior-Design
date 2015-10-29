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

void initializeADC0() {
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK; /*Enable the ADC0 Clock*/
	ADC0_CFG1 |= ADC_CFG1_MODE(0);
	ADC0_SC1A |= ADC_SC1_ADCH(31) | ADC_SC1_AIEN_MASK;//00101 to read
		ADC0_CFG2 |= ADC_CFG2_MUXSEL(1) | ADC_CFG2_ADHSC_MASK;
		NVIC_EnableIRQ(ADC0_IRQn);
}

void initializeAD5b() {
	PORTD->PCR[1] |= PORT_PCR_MUX(0); // Asserts PTD1 as ADC0_SE5b
}

unsigned int readChannelAD5b() {
	ADC0_SC1A = ADC_SC1_ADCH(5); //Write to SC1A to start conversion
	while(ADC0_SC2 & ADC_SC2_ADACT_MASK); //Conversion in progress
	while(!(ADC0_SC1A & ADC_SC1_COCO_MASK)); //Wait until conversion complete
	return ADC0_RA;
}

void initializeLineCameraGPIO() {
	PORTD->PCR[3] |= PORT_PCR_MUX(1); // Set as GPIO
	PORTD->PCR[2] |= PORT_PCR_MUX(1); // Set as GPIO
	BITBAND_REG(GPIOD_PDDR,3) = 1;   // sets as output
	BITBAND_REG(GPIOD_PDDR,2) = 1;   // sets as output
}


void Delay(void){
  for(int dly=0;dly<250;dly++);
}

#define SI_FLIP (GPIOD_PTOR = (1<<3))
#define CLK_FLIP (GPIOD_PTOR = (1<<2))

void pit_init(void)
{
	// Enable PIT clock
	SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;

	// Enable Blue LED clock and MUX
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

	// Turn on PIT
	PIT->MCR = 0;

	// Configure PIT to produce an interrupt every .005s
	PIT->CHANNEL[0].LDVAL = 900000;	// 1/20Mhz = 50ns   (.005s/50ns)-1= 99,999 cycles or 0x1869F
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK; // Enable interrupt and enable timer

	// Enable interrupt registers ISER and ICPR
	NVIC_EnableIRQ(PIT0_IRQn);
}

void ImageCapture(int* ImageData){//this is a pointer to a single character in memory

  unsigned char i;
  SI_FLIP;
  Delay();
  CLK_FLIP;
  Delay();
  SI_FLIP;
  ImageData[0] = readChannelAD5b();// inputs data from camera (first pixel)
  CLK_FLIP;

  for(i=1;i<128;i++)
  {
	  CLK_FLIP;
	  ImageData[i] = readChannelAD5b(); // inputs data from camera (one pixel each time through loop)
	  CLK_FLIP;
  }
}

#define THRESHOLD (3.3 /*volts*/ / 0.03125) // 8-bit A/D
void printAll(int* stuff) {
	PRINTF("\r\n\r\n");
	for (int i = 0; i <= 128; i++) { // ignore the first and last 16 bits in the camera frame
	  if (stuff[i] < THRESHOLD)
		  PRINTF("1"); // black (low intensity)
	  else
		  PRINTF("0"); // white (high intensity)
//	  if (stuff[i] < THRESHOLD)
//		  PRINTF("Value lo: %d\r\n", stuff[i]); // black (low intensity)
//	  else
//		  PRINTF("Value high: %d\r\n", stuff[i]); // white (high intensity)
//PRINTF("Data " + stuff[i]);
	}
}

void enablePorts() {
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; /*Enable Port B Clock Gate Control*/
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK; /*Enable Port E Clock Gate Control*/
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; /*Enable Port C Clock Gate Control*/
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; /*Enable Port A Clock Gate Control*/
}

/*----------------------------------------------------------------------------
  Main function
 *----------------------------------------------------------------------------*/
int data[128];
int main (void) {
  // Configure board specific pin muxing
  hardware_init();
  // Initialize UART terminal
  dbg_uart_init();
  PRINTF("\r\nRunning the Line Camera project.\r\n");
  SystemCoreClockUpdate();                 /* Update system core clock       */
  enablePorts();

//  Flex_Timer_Test();
  initializeADC0();
//  initializeAD5b();
  initializeLineCameraGPIO();
  InitFlexTimer0CH0();
  PORTC_PCR1 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK; // PTC1 ALT4// DSE = 1, High drive strength
  pit_init();
  NVIC_EnableIRQ(FTM0_IRQn);

  while (1){
//	  PRINTF("Reading: \r\n");
//	  ImageCapture(data);
//	  printAll(data);
//	  PRINTF("\r\n\r\n");
  }
}

unsigned int fallingCount = 0;

void FTM0_IRQHandler(void) {
	//	PRINTF("\r\n THIS IS INTERRUPT: %d C0V: %d\r\n", fallingCount, FTM0_CNT);
	int temp;
	temp = FTM0_C0SC;
	FTM0_C0SC &= ~FTM_CnSC_CHF_MASK;
	ADC0_SC1A = ADC_SC1_ADCH(5) | ADC_SC1_AIEN_MASK; //Write to SC1A to start conversion
	fallingCount++;
	GPIOD_PCOR |= (1 << 3);
	if (fallingCount>=128) {
		FTM0_C0V = 0;
	}
}

void ADC0_IRQHandler(void) {
	data[fallingCount] = ADC0_RA;
	if (fallingCount>=128) {
		printAll(data);
	}
}

void PIT0_IRQHandler(void)
{
	// Clear interrupt
	PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;
	GPIOD_PSOR |= (1 << 3);
	FTM0_C0V = 200/2;
	fallingCount=0;
}
