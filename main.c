/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#define RED_LED_0 11 // PortC Pin 11
#define RED_LED_1 10 // PortC Pin 10
#define RED_LED_2 6 // PortC Pin 6
#define RED_LED_3 5 // PortC Pin 5
#define RED_LED_4 4 // PortC Pin 4
#define RED_LED_5 3 // PortC Pin 3
#define RED_LED_6 0 // PortC Pin 0
#define RED_LED_7 7 // PortC Pin 7

#define GREEN_LED_0 9 // PortC Pin 9
#define GREEN_LED_1 8 // PortC Pin 8
#define GREEN_LED_2 5 // PortA Pin 5
#define GREEN_LED_3 4 // PortA Pin 4
#define GREEN_LED_4 12 // PortA Pin 12
#define GREEN_LED_5 4 // PortD Pin 4
#define GREEN_LED_6 1 // PortA Pin 1
#define GREEN_LED_7 2 // PortA Pin 2

#define MASK(x) (1 << (x))

#define PTB0_Pin 0
#define PTB1_Pin 1

#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128

void initGPIO(void) {
	// Enable Clock to PORTA, PORTC and PORTD
	SIM->SCGC5 |= ((SIM_SCGC5_PORTA_MASK) | (SIM_SCGC5_PORTC_MASK) | (SIM_SCGC5_PORTD_MASK));
	
	// Configure MUX settings to make all pins GPIO
	// Using the 2 rows at lower right
	
	// First row
	PORTC->PCR[RED_LED_0] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[RED_LED_0] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[RED_LED_1] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[RED_LED_1] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[RED_LED_2] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[RED_LED_2] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[RED_LED_3] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[RED_LED_3] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[RED_LED_4] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[RED_LED_4] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[RED_LED_5] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[RED_LED_5] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[RED_LED_6] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[RED_LED_6] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[RED_LED_7] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[RED_LED_7] |= PORT_PCR_MUX(1);
	
	// Second row
	PORTC->PCR[GREEN_LED_0] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED_0] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_LED_1] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED_1] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[GREEN_LED_2] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[GREEN_LED_2] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[GREEN_LED_3] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[GREEN_LED_3] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[GREEN_LED_4] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[GREEN_LED_4] |= PORT_PCR_MUX(1);
	
	PORTD->PCR[GREEN_LED_5] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[GREEN_LED_5] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[GREEN_LED_6] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[GREEN_LED_6] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[GREEN_LED_7] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[GREEN_LED_7] |= PORT_PCR_MUX(1);
	
	// Set Data Direction Registers for PortB and PortD
	
	// First row
	PTC->PDDR |= MASK(RED_LED_0);
	PTC->PDDR |= MASK(RED_LED_1);
	PTC->PDDR |= MASK(RED_LED_2);
	PTC->PDDR |= MASK(RED_LED_3);
	PTC->PDDR |= MASK(RED_LED_4);
	PTC->PDDR |= MASK(RED_LED_5);
	PTC->PDDR |= MASK(RED_LED_6);
	PTC->PDDR |= MASK(RED_LED_7);
	
	// Second row
	PTC->PDDR |= MASK(GREEN_LED_0);
	PTC->PDDR |= MASK(GREEN_LED_1);
	PTA->PDDR |= MASK(GREEN_LED_2);
	PTA->PDDR |= MASK(GREEN_LED_3);
	PTA->PDDR |= MASK(GREEN_LED_4);
	PTD->PDDR |= MASK(GREEN_LED_5);
	PTA->PDDR |= MASK(GREEN_LED_6);
	PTA->PDDR |= MASK(GREEN_LED_7);
}

void initPWM(void){
	
	//Enable clock for Port B
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	//Clear PTB0_Pin and PTB1_Pin settings and configure as PWM
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);
	
	//Enable clock for TPM1
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	
	//Clear the TPMSRC bits and set the clock source as MCGFLLCLK clock or MCGPLLCLK/2  
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	TPM1->MOD = 0;
	TPM1_C0V = 0;
	
	//Increment on every LPTPM counter clock, set pre-scaling to 128 and operate in count up
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1)) | (TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	//Set as edge-aligned PWM with high true pulses
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

void initUART2(uint32_t baud_rate)
{
	uint32_t divisor, bus_clock;
	
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);
	
	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
	
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	
	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
	divisor = bus_clock / (baud_rate * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;
	
	UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
}



 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void app_main (void *argument) {
 
  // ...
  for (;;) {}
}
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	initGPIO();
	initUART2(BAUD_RATE);
	initPWM();
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(app_main, NULL, NULL);    // Create application main thread
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
