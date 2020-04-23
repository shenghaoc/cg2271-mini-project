/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/

#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include <math.h>

#define MSG_COUNT 2

#define RED_LED_0 11 // Port C Pin 11
#define RED_LED_1 10 // Port C Pin 10
#define RED_LED_2 6 // Port C Pin 6
#define RED_LED_3 5 // Port C Pin 5
#define RED_LED_4 4 // Port C Pin 4
#define RED_LED_5 3 // Port C Pin 3
#define RED_LED_6 0 // Port C Pin 0
#define RED_LED_7 7 // Port C Pin 7

#define GREEN_LED_0 9 // Port C Pin 9
#define GREEN_LED_1 8 // Port C Pin 8
#define GREEN_LED_2 5 // Port A Pin 5
#define GREEN_LED_3 4 // Port A Pin 4
#define GREEN_LED_4 12 // Port A Pin 12
#define GREEN_LED_5 4 // Port D Pin 4
#define GREEN_LED_6 1 // Port A Pin 1
#define GREEN_LED_7 2 // Port A Pin 2

#define MASK(x) (1 << (x))

#define BUZZER_PIN 0 // Port B Pin 0 -> TPM1_CH0

#define LEFT_FRONT_WHEEL_PIN1 2 // Port B Pin 2 -> TPM2_CH0
#define LEFT__FRONT_WHEEL_PIN2 3 // Port B Pin 3 -> TPM2_CH1
#define RIGHT__FRONT_WHEEL_PIN1 29 // Port E Pin 29 -> TPM0_CH2
#define RIGHT__FRONT_WHEEL_PIN2 30 // Port E Pin 30 -> TPM0_CH3
#define LEFT_BACK_WHEEL_PIN1 31 // Port E Pin 31 -> TPM0_CH4
#define LEFT_BACK_WHEEL_PIN2 24 // Port E Pin 24 -> TPM0_CH0
#define RIGHT_BACK_WHEEL_PIN1 25 // Port E Pin 25 -> TPM0_CH1
#define RIGHT_BACK_WHEEL_PIN2 5 // Port D Pin 5 -> TPM0_CH5

#define BAUD_RATE 9600
#define UART_TX_PORTE0 0
#define UART_RX_PORTE1 1
#define UART1_INT_PRIO 128

#define c 261
#define d 294 
#define e 329 
#define f 349
#define g 392 
#define a 440 
#define b 493 
#define C 523 


// buzzer set LSB, LED set 2nd LSB
osEventFlagsId_t connecting_flag, connected_flag, moving_flag;

// no other action required at same time, so tied to thread
osThreadId_t finish_tone_flag;

// one device, different behavior for each connection state, working throughout
// also
osMutexId_t buzzerMutex, greenMutex;

// Car does not move throughout, therefore use semaphore
osSemaphoreId_t mySem_Wheels;

enum color_t{Red, Green};
enum state_t{led_on, led_off};

volatile uint8_t rx_data = 0;

// coordinates
typedef struct {
	uint8_t x;
	uint8_t y;
} myDataPkt;

osMessageQueueId_t coordMsg;

// can avoid two different functions for red LED since only delay changes
volatile uint32_t delay = 250;
double aux;

int melody_connecting[] = {a, b, c,  d,  e, f,  g, C};
int melody_connected[] = {C,  b,  g,  C,  b,   e,  C,  c,  g, a, C };
int melody_finish[] = {C,  b,  a,  g,  f,  e,  d,  c};

// CCAAADAA
GPIO_Type* green_LED_PT[] = {PTC, PTC, PTA, PTA, PTA, PTD, PTA, PTA};
PORT_Type* green_LED_PORT[] = {PORTC, PORTC, PORTA, PORTA, PORTA, PORTD, PORTA, PORTA};
int green_LED[] = {GREEN_LED_0, GREEN_LED_1, GREEN_LED_2, GREEN_LED_3, GREEN_LED_4, GREEN_LED_5, GREEN_LED_6, GREEN_LED_7};
int red_LED[] = {RED_LED_0, RED_LED_1, RED_LED_2, RED_LED_3, RED_LED_4, RED_LED_5, RED_LED_6, RED_LED_7};

void initGPIO(void) {
	// Enable Clock to PORTA, PORTC and PORTD
	SIM->SCGC5 |= ((SIM_SCGC5_PORTA_MASK) | (SIM_SCGC5_PORTC_MASK) | (SIM_SCGC5_PORTD_MASK));

	// Configure MUX settings to make all pins GPIO
	// Using the 2 rows at lower right

	// First row
	
	for (int i = 0; i < 7; i++) {
		PORTC->PCR[red_LED[i]] &= ~PORT_PCR_MUX_MASK;
		PORTC->PCR[red_LED[i]] |= PORT_PCR_MUX(1);
	}

	// Second row
	for (int i = 0; i < 7; i++) {
		green_LED_PORT[i]->PCR[green_LED[i]] &= ~PORT_PCR_MUX_MASK;
		green_LED_PORT[i]->PCR[green_LED[i]] |= PORT_PCR_MUX(1);
	}

	// Set Data Direction Registers for PortB and PortD

	// First row
	for (int i = 0; i < 7; i++) {
		PTC->PDDR |= MASK(red_LED[i]);
	}

	// Second row	
	for (int i = 0; i < 7; i++) {
		green_LED_PT[i]->PDDR |= MASK(green_LED[i]);
	}
}

void initPWM(void){

	//Enable clock for Ports A, B and E
	SIM_SCGC5 |= ((SIM_SCGC5_PORTD_MASK) | (SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTE_MASK));

	//Clear PTB0_Pin and PTB1_Pin settings and configure as PWM
	PORTB->PCR[BUZZER_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[BUZZER_PIN] |= PORT_PCR_MUX(3);

	// Same for left front wheel
	PORTB->PCR[LEFT_FRONT_WHEEL_PIN1] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[LEFT_FRONT_WHEEL_PIN1] |= PORT_PCR_MUX(3);
	PORTB->PCR[LEFT__FRONT_WHEEL_PIN2] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[LEFT__FRONT_WHEEL_PIN2] |= PORT_PCR_MUX(3);

	// Same for right front wheel
	PORTE->PCR[RIGHT__FRONT_WHEEL_PIN1] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[RIGHT__FRONT_WHEEL_PIN1] |= PORT_PCR_MUX(3);
	PORTE->PCR[RIGHT__FRONT_WHEEL_PIN2] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[RIGHT__FRONT_WHEEL_PIN2] |= PORT_PCR_MUX(3);

	// same for left back wheel
	PORTE->PCR[LEFT_BACK_WHEEL_PIN1] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[LEFT_BACK_WHEEL_PIN1] |= PORT_PCR_MUX(3);
	PORTE->PCR[LEFT_BACK_WHEEL_PIN2] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[LEFT_BACK_WHEEL_PIN2] |= PORT_PCR_MUX(3);

	// same for right back wheel
	 PORTE->PCR[RIGHT_BACK_WHEEL_PIN1] &= ~PORT_PCR_MUX_MASK;
	 PORTE->PCR[RIGHT_BACK_WHEEL_PIN1] |= PORT_PCR_MUX(3);
	 PORTD->PCR[RIGHT_BACK_WHEEL_PIN2] &= ~PORT_PCR_MUX_MASK;
	 PORTD->PCR[RIGHT_BACK_WHEEL_PIN2] |= PORT_PCR_MUX(3);

	//Enable clock for TPM0, TPM1 and TPM2
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
	NVIC_SetPriority(UART1_IRQn, UART1_INT_PRIO);
	NVIC_ClearPendingIRQ(UART1_IRQn);
	NVIC_EnableIRQ(UART1_IRQn);

}

void UART1_IRQHandler(void) {
	if ((UART1->S1 & UART_S1_RDRF_MASK)) {
		rx_data = UART1->D;

		if (rx_data == 0x01) {
			osEventFlagsSet(connecting_flag, 0x0000001);
		} else if (rx_data == 0x02) {
			// press music icon to play finish tone
			osThreadFlagsSet(finish_tone_flag, 0x0001);
		} else {
			myDataPkt myData;
			myData.x = rx_data;
			rx_data = UART1->D;
			myData.y = rx_data;
			osMessageQueuePut(coordMsg, &myData, NULL, 0);
			osSemaphoreRelease(mySem_Wheels);
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
			for (int i = 0; i < 7; i++) {
				PTC->PSOR = MASK(red_LED[i]);
			}
		}
		else if (color == Green) 
		{
			// CCAAADAA
			for (int i = 0; i < 7; i++) {
				green_LED_PT[i]->PSOR = MASK(green_LED[i]);
			}
		}
	}
	else
	{
		if (color == Red)
		{
			// all Cs
			for (int i = 0; i < 7; i++) {
				PTC->PCOR = MASK(red_LED[i]);
			}
		}
		else if (color == Green) 
		{
			// CCAAADAA
			for (int i = 0; i < 7; i++) {
				green_LED_PT[i]->PCOR = MASK(green_LED[i]);
			}
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

void offRGB(void) {
	led_control(Green, led_off);
	led_control(Red, led_off);
}


void connecting_tone_thread (void *argument){
	//...
	for (;;){
		osEventFlagsWait(connecting_flag, 0x0000001, osFlagsWaitAny, osWaitForever);
		osMutexAcquire(buzzerMutex, osWaitForever);

		for (int i = 0; i < 8; i++) {
			generateSoundPWM1(melody_connecting[i]);
			osDelay(1000);		
		}

		osMutexRelease(buzzerMutex);
		osEventFlagsSet(connected_flag, 0x0000001);
	}
}

void connected_tone_thread (void *argument){
	//...
	int i = 0;
	for (;;){
		// connected only after both connecting tone and green led flashed twice!
		osEventFlagsWait(connected_flag, 0x0000003, osFlagsWaitAll, osWaitForever);
		osMutexAcquire(buzzerMutex, osWaitForever);
		
		i = (i == 10) ? 0 : i + 1;
		generateSoundPWM1(melody_connected[i]);
		osDelay(1000);		

		osMutexRelease(buzzerMutex);
	}
}

void finish_tone_thread (void *argument){
	//...
	for (;;){
		osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
		osMutexAcquire(buzzerMutex, osWaitForever);
		
		for (int i = 0; i < 8; i++) {
			generateSoundPWM1(melody_finish[i]);
			osDelay(1000);		
		}

		osMutexRelease(buzzerMutex);
		osThreadFlagsSet(finish_tone_flag, NULL);
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

void running_green_thread (void *argument){
	//...
	int i = 0;
	for (;;){
		osEventFlagsWait(moving_flag, 0x0000001, osFlagsWaitAny, osWaitForever);
		osMutexAcquire(greenMutex, osWaitForever);
		
		i = (i == 7) ? 0 : i + 1;
		green_LED_PT[i]->PSOR = MASK(green_LED[i]);
		osDelay(1000);
		green_LED_PT[i]->PCOR = MASK(green_LED[i]);
		osDelay(1000);

		osMutexRelease(greenMutex);
	}
}

void constant_green_thread (void *argument){
	//...
	for (;;){
		osEventFlagsWait(connected_flag, 0x0000003, osFlagsWaitAll, osWaitForever);
		osMutexAcquire(greenMutex, osWaitForever);
		// always on
		led_control(Green, led_on);
		osMutexRelease(greenMutex);
	}
}

void flashing_red_thread (void *argument){
	// ...
	for (;;) {
		osEventFlagsWait(connected_flag, 0x0000003, osFlagsWaitAll, osWaitForever);
		led_control(Red, led_on);
		osDelay(delay);
		led_control(Red, led_off);
		osDelay(delay);
	}
}


void wheel_control_thread (void *argument){
	//...
	myDataPkt myRXData;
	uint32_t x;
	uint32_t y;
	for (;;){
		osSemaphoreAcquire(mySem_Wheels, osWaitForever);
		osMessageQueueGet(coordMsg, &myRXData, NULL, osWaitForever);
		x = myRXData.x;
		y = myRXData.y;
		osEventFlagsSet(moving_flag, 0x0000001);
		delay = 500;
		
		aux = pow(pow(x - 153, 2) + pow(153 - y, 2), 0.5);
		if (x == 255 || y == 255) {
			// stop
		} else if (y < 153 && aux < 120 && y > 10) {
			// turn somewhere?
		} else if (y > 153 && aux < 120) {
			// turn somewhere?
		} else if (y > 153 && aux < 120) {
			// turn somewhere?
		} else if (y < 153 && aux < 120 && y > 10) {
			// turn somewhere?
		}


		// Signal movement has finished
		delay = 250;
		osEventFlagsSet(moving_flag, NULL);
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

// Need to be higher priority than normal tone
osThreadAttr_t finish_attr = {
	.priority = osPriorityNormal1
};

// When car is moving, the other thread shouldn't run at all
osThreadAttr_t moving_attr = {
	.priority = osPriorityNormal1
};

// Everything else is secondary to car movement
osThreadAttr_t wheels_attr = {
	.priority = osPriorityHigh
};

int main (void) {

	// System Initialization
	SystemCoreClockUpdate();
	initGPIO();
	initUART1(BAUD_RATE);
  initPWM();
	offRGB();
	// ...

	osKernelInitialize();                 // Initialize CMSIS-RTOS
	// event flags
	connecting_flag = osEventFlagsNew(NULL);
	connected_flag = osEventFlagsNew(NULL);
	moving_flag = osEventFlagsNew(NULL);

	// mutexes
	buzzerMutex = osMutexNew(NULL);
	greenMutex = osMutexNew(NULL);

	// semaphores
	mySem_Wheels = osSemaphoreNew(MSG_COUNT,0,NULL);
	
	// messages
	
	coordMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);

	/*
	 *	threads
	 */

	// for buzzer
	finish_tone_flag = osThreadNew(finish_tone_thread, NULL, &finish_attr);
	osThreadNew(connecting_tone_thread, NULL, NULL); 
	osThreadNew(connected_tone_thread, NULL, NULL);  

	// for LED
	osThreadNew(connecting_flash_thread, NULL, NULL);  
	osThreadNew(running_green_thread, NULL, &moving_attr);  
	osThreadNew(constant_green_thread, NULL, NULL);  
	osThreadNew(flashing_red_thread, NULL, NULL);  

	// for wheels
	osThreadNew(wheel_control_thread, NULL, &wheels_attr); 

	// synchronize connection events
	osThreadNew(app_main, NULL, NULL);    // Create application main thread

	osKernelStart();                      // Start thread execution
	for (;;) {}
}
