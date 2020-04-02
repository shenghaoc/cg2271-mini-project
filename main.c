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
#define UART_TX_PORTE0 0
#define UART_RX_PORTE1 1
#define UART1_INT_PRIO 128


osSemaphoreId_t mySem_Green;

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

void initUART1(uint32_t baud_rate)
{
	uint32_t divisor, bus_clock;

	// enable clock to UART1 and Port E
	SIM->SCGC4 |= SIM_SCGC4_UART1_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	// connect UART1 to pins for PTE0, PTE1
	PORTE->PCR[UART_RX_PORTE1] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE1] |= PORT_PCR_MUX(3);

	// ensure rx and rx interrupt are disabled before configuration
	UART1->C2 &= ~((UART_C2_RE_MASK) | (UART_C2_RIE_MASK));

	// Set baud rate to 4800 baud
	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
	divisor = bus_clock / (baud_rate * 16);
	UART1->BDH = UART_BDH_SBR(divisor >> 8);
	UART1->BDL = UART_BDL_SBR(divisor);

	// No parity, 8 bits, two stop bits, other settings;
	UART1->C1 = UART1->S2 = 0 UART1->C3 = 0;

	// Enable transmitter and transmitter interrupt
	UART1->C2 |= ((UART_C2_RE_MASK) | (UART_C2_RIE_MASK));

	/* Enable Interrupts */
	NVIC_SetPriority(UART1_IRQn, 128);
	NVIC_ClearPendingIRQ(UART1_IRQn);
	NVIC_EnableIRQ(UART1_IRQn);

}

void UART1_IRQHandler(void) {
	if ((UART1->S1 & UART_S1_RDRF_MASK)) {
		rx_data = UART1->D;

	if (rx_data == 0x00) {
			osSemaphoreRelease(mySem_Green);
		}
	}
}


void led_green_thread (void *argument){
	//...
	for (;;){
		osSemaphoreAcquire(mySem_Green, osWaitForever);
		led_control(Green, led_on);
		osDelay(1000);
		led_control(Green, led_off);
		osDelay(1000);
	}
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
	initUART1(BAUD_RATE);
	initPWM();
	// ...

	mySem_Green = osSemaphoreNew(1,0,NULL);
	osKernelInitialize();                 // Initialize CMSIS-RTOS
	osThreadNew(app_main, NULL, NULL);    // Create application main thread
	osKernelStart();                      // Start thread execution
	for (;;) {}
}
