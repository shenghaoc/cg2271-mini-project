/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/

#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1

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

#define BUZZER_PIN 0

#define LEFT_FRONT_WHEEL_PIN1 2
#define LEFT__FRONT_WHEEL_PIN2 3
#define RIGHT__FRONT_WHEEL_PIN1 29
#define RIGHT__FRONT_WHEEL_PIN2 30
#define LEFT_BACK_WHEEL_PIN1 31
#define LEFT_BACK_WHEEL_PIN2 24
#define RIGHT_BACK_WHEEL_PIN1 25
#define RIGHT_BACK_WHEEL_PIN2 0

#define BAUD_RATE 9600
#define UART_TX_PORTE0 0
#define UART_RX_PORTE1 1
#define UART1_INT_PRIO 128

// buzzer set LSB, LED set 2nd LSB
osEventFlagsId_t disconnected_flag, connecting_flag, connected_flag, disconnecting_flag;


// one device, different different behavior for each connection state, working throughout
osMutexId_t buzzerMutex, greenMutex, redMutex;

// Car does not move throughout, therefore use semaphore
osSemaphoreId_t mySem_Wheels;

enum color_t{Red, Green};
enum state_t{led_on, led_off};

volatile uint8_t rx_data = 0;
volatile uint8_t x = 0;
volatile uint8_t y = 0;


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
	SIM_SCGC5 |= ((SIM_SCGC5_PORTA_MASK) | (SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTE_MASK));

	//Clear PTB0_Pin and PTB1_Pin settings and configure as PWM
	PORTB->PCR[BUZZER_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[BUZZER_PIN] |= PORT_PCR_MUX(3);

	PORTB->PCR[LEFT_FRONT_WHEEL_PIN1] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[LEFT_FRONT_WHEEL_PIN1] |= PORT_PCR_MUX(3);
	PORTB->PCR[LEFT__FRONT_WHEEL_PIN2] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[LEFT__FRONT_WHEEL_PIN2] |= PORT_PCR_MUX(3);

	PORTE->PCR[RIGHT__FRONT_WHEEL_PIN1] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[RIGHT__FRONT_WHEEL_PIN1] |= PORT_PCR_MUX(3);
	PORTE->PCR[RIGHT__FRONT_WHEEL_PIN2] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[RIGHT__FRONT_WHEEL_PIN2] |= PORT_PCR_MUX(3);

	PORTE->PCR[LEFT_BACK_WHEEL_PIN1] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[LEFT_BACK_WHEEL_PIN1] |= PORT_PCR_MUX(3);
	PORTE->PCR[LEFT_BACK_WHEEL_PIN2] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[LEFT_BACK_WHEEL_PIN2] |= PORT_PCR_MUX(3);

	PORTE->PCR[RIGHT_BACK_WHEEL_PIN1] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[RIGHT_BACK_WHEEL_PIN1] |= PORT_PCR_MUX(3);
	PORTA->PCR[RIGHT_BACK_WHEEL_PIN2] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[RIGHT_BACK_WHEEL_PIN2] |= PORT_PCR_MUX(3);

	//Enable clock for TPM1
	SIM->SCGC6 |= ((SIM_SCGC6_TPM0_MASK) | (SIM_SCGC6_TPM1_MASK) | (SIM_SCGC6_TPM2_MASK));

	//Clear the TPMSRC bits and set the clock source as MCGFLLCLK clock or MCGPLLCLK/2
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

	TPM0->MOD = 0;
	TPM0_C0V = 0;

	TPM1->MOD = 0;
	TPM1_C0V = 0;

	TPM2->MOD = 0;
	TPM2_C0V = 0;

	//Increment on every LPTPM counter clock, set pre-scaling to 128 and operate in count up
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1)) | (TPM_SC_PS(7));
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK);

	//Increment on every LPTPM counter clock, set pre-scaling to 128 and operate in count up
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1)) | (TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);

	//Increment on every LPTPM counter clock, set pre-scaling to 128 and operate in count up
	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM2->SC |= (TPM_SC_CMOD(1)) | (TPM_SC_PS(7));
	TPM2->SC &= ~(TPM_SC_CPWMS_MASK);

	//Set as edge-aligned PWM with high true pulses
	TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	//Set as edge-aligned PWM with high true pulses
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	//Set as edge-aligned PWM with high true pulses
	TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
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
	UART1->C1 = UART1->S2 = UART1->C3 = 0;

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

		if (rx_data == 0x01) {
			osEventFlagsSet(connecting_flag, 0x0000003);
		} else if (rx_data == 0x02) {
			// press music icon to disconnect
			osEventFlagsSet(connected_flag, NULL);
			osEventFlagsSet(disconnecting_flag, 0x0000001);
		} else {
			osSemaphoreRelease(mySem_Wheels);
			x = rx_data;
			rx_data = UART1->D;
			y = rx_data;
		}
	}
}

// flash entire rows
void led_control(enum color_t color, enum state_t state)
{
	if (state == led_on)
	{
		if (color == Red)
		{
			// all Cs
			PTC->PCOR = MASK(RED_LED_0);
			PTC->PCOR = MASK(RED_LED_1);
			PTC->PCOR = MASK(RED_LED_2);
			PTC->PCOR = MASK(RED_LED_3);
			PTC->PCOR = MASK(RED_LED_4);
			PTC->PCOR = MASK(RED_LED_5);
			PTC->PCOR = MASK(RED_LED_6);
			PTC->PCOR = MASK(RED_LED_7);
		}
		else if (color == Green) 
		{
			// CCAAADAA
			PTC->PCOR = MASK(GREEN_LED_0);
			PTC->PCOR = MASK(GREEN_LED_1);
			PTA->PCOR = MASK(GREEN_LED_2);
			PTA->PCOR = MASK(GREEN_LED_3);
			PTA->PCOR = MASK(GREEN_LED_4);
			PTD->PCOR = MASK(GREEN_LED_5);
			PTA->PCOR = MASK(GREEN_LED_6);
			PTA->PCOR = MASK(GREEN_LED_7);
		}
	}
	else
	{
		if (color == Red)
		{
			// all Cs
			PTC->PSOR = MASK(RED_LED_0);
			PTC->PSOR = MASK(RED_LED_1);
			PTC->PSOR = MASK(RED_LED_2);
			PTC->PSOR = MASK(RED_LED_3);
			PTC->PSOR = MASK(RED_LED_4);
			PTC->PSOR = MASK(RED_LED_5);
			PTC->PSOR = MASK(RED_LED_6);
			PTC->PSOR = MASK(RED_LED_7);
		}
		else if (color == Green) 
		{
			// CCAAADAA
			PTC->PSOR = MASK(GREEN_LED_0);
			PTC->PSOR = MASK(GREEN_LED_1);
			PTA->PSOR = MASK(GREEN_LED_2);
			PTA->PSOR = MASK(GREEN_LED_3);
			PTA->PSOR = MASK(GREEN_LED_4);
			PTD->PSOR = MASK(GREEN_LED_5);
			PTA->PSOR = MASK(GREEN_LED_6);
			PTA->PSOR = MASK(GREEN_LED_7);
		}
	}
}

//modValue(freq) calculates the required MOD value for a given frequency, freq.
int modValue(int freq){
	//375000 is the new clock frequency after prescaling by 128
	return (375000/freq) - 1;
}

void generateSoundPWM0(int freq){
	TPM0->MOD = modValue(freq);
	TPM0_C0V = modValue(freq)/2;
}


void generateSoundPWM1(int freq){
	TPM1->MOD = modValue(freq);
	TPM1_C0V = modValue(freq)/2;
}


void generateSoundPWM2(int freq){
	TPM2->MOD = modValue(freq);
	TPM2_C0V = modValue(freq)/2;
}

void connecting_tone_thread (void *argument){
	//...
	for (;;){
		osEventFlagsWait(connecting_flag, 0x0000001, osFlagsWaitAny, osWaitForever);
		// 
		osMutexAcquire(buzzerMutex, osWaitForever);
		//262Hz (Note C)
		generateSoundPWM1(262);
		osDelay(2091752);

		//294Hz (Note D)
		generateSoundPWM1(294);
		osDelay(2091752);

		//330Hz (Note E)
		generateSoundPWM1(330);		
		osDelay(2091752);

		//349Hz (Note F)
		generateSoundPWM1(349);		
		osDelay(2091752);

		//392Hz (Note G)
		generateSoundPWM1(392);
		osDelay(2091752);

		//440Hz (Note A)
		generateSoundPWM1(440);
		osDelay(2091752);

		//494Hz (Note B)
		generateSoundPWM1(494);
		osDelay(2091752);		

		osMutexRelease(buzzerMutex);
		osEventFlagsSet(connected_flag, 0x0000001);
	}
}

void connected_tone_thread (void *argument){
	//...
	for (;;){
		// connected only after both connecting tone and green led flashed twice!
		osEventFlagsWait(connected_flag, 0x0000003, osFlagsWaitAny, osWaitForever);
		osMutexAcquire(buzzerMutex, osWaitForever);
		//262Hz (Note C)
		generateSoundPWM1(262);
		osDelay(2091752);

		//294Hz (Note D)
		generateSoundPWM1(294);	
		osDelay(2091752);

		//330Hz (Note E)
		generateSoundPWM1(330);
		osDelay(2091752);

		//349Hz (Note F)
		generateSoundPWM1(349);
		osDelay(2091752);

		//392Hz (Note G)
		generateSoundPWM1(392);
		osDelay(2091752);

		//440Hz (Note A)
		generateSoundPWM1(440);
		osDelay(2091752);

		//494Hz (Note B)
		generateSoundPWM1(494);
		osDelay(2091752);

		osMutexRelease(buzzerMutex);
	}
}

void disconnecting_tone_thread (void *argument){
	//...
	for (;;){
		osEventFlagsWait(disconnecting_flag, 0x0000001, osFlagsWaitAny, osWaitForever);
		osMutexAcquire(buzzerMutex, osWaitForever);
		//262Hz (Note C)
		generateSoundPWM1(262);
		osDelay(2091752);

		//294Hz (Note D)
		generateSoundPWM1(294);
		osDelay(2091752);

		//330Hz (Note E)
		generateSoundPWM1(330);
		osDelay(2091752);

		//349Hz (Note F)
		generateSoundPWM1(349);
		osDelay(2091752);

		//392Hz (Note G)
		generateSoundPWM1(392);
		osDelay(2091752);

		//440Hz (Note A)
		generateSoundPWM1(440);
		osDelay(2091752);

		//494Hz (Note B)
		generateSoundPWM1(494);
		osDelay(2091752);

		osMutexRelease(buzzerMutex);
		osEventFlagsSet(disconnecting_flag, NULL);
		osEventFlagsSet(disconnected_flag, 0x0000001);
	}
}

void connecting_flash_thread (void *argument){
	//...
	for (;;){
		osEventFlagsWait(connecting_flag, 0x0000001, osFlagsWaitAny, osWaitForever);
		osMutexAcquire(greenMutex, osWaitForever);
		// flash twice
		for (int i = 0; i < 2; i++) {
			led_control(Green, led_on);
			osDelay(1000);
			led_control(Green, led_off);
			osDelay(1000);
		}
		osMutexRelease(greenMutex);
		osEventFlagsSet(connected_flag, 0x0000002);
	}
}

void wheel_control_thread (void *argument){
	//...
	for (;;){
		osEventFlagsWait(connected_flag, 0x0000001, osFlagsWaitAny, osWaitForever);
		osSemaphoreAcquire(mySem_Wheels, osWaitForever);
	}
}


/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void app_main (void *argument) {

	// ...
	for (;;) {
		// wait for both buzzer and led to signal
		osEventFlagsWait(connected_flag, 0x0000003, osFlagsWaitAll, osWaitForever);
		osEventFlagsSet(connecting_flag, NULL);
	}
}

int main (void) {

	// System Initialization
	SystemCoreClockUpdate();
	initGPIO();
	initUART1(BAUD_RATE);
	initPWM();
	// ...

	osKernelInitialize();                 // Initialize CMSIS-RTOS
	// event flags
	disconnected_flag = osEventFlagsNew(NULL);
	osEventFlagsSet(disconnected_flag, 0x0000001); // by default disconnected
	connecting_flag = osEventFlagsNew(NULL);
	connected_flag = osEventFlagsNew(NULL);
	disconnecting_flag = osEventFlagsNew(NULL);
	
	// mutexes
	buzzerMutex = osMutexNew(NULL);
	greenMutex = osMutexNew(NULL);
	redMutex = osMutexNew(NULL);
	
	// semaphores
	mySem_Wheels = osSemaphoreNew(1,0,NULL);
	
	// threads
	// for buzzer
	osThreadNew(connecting_flash_thread, NULL, NULL);  
	osThreadNew(connecting_tone_thread, NULL, NULL); 
	osThreadNew(connected_tone_thread, NULL, NULL);  
	
	// for LED
	osThreadNew(disconnecting_tone_thread, NULL, NULL);
	
	// for wheels
	osThreadNew(wheel_control_thread, NULL, NULL); 
	
	// synchronize connection events
	osThreadNew(app_main, NULL, NULL);    // Create application main thread
	
	osKernelStart();                      // Start thread execution
	for (;;) {}
}
