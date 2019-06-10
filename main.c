//*****************************************************************************
//
// Codigo de partida Practica 1.
// Autores: Eva Gonzalez, Ignacio Herrero, Jose Manuel Cano
//
//*****************************************************************************

#include<stdbool.h>
#include<stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "drivers/buttons.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "utils/cpu_usage.h"
#include "event_groups.h"
#include "driverlib/udma.h"
#include "driverlib/comp.h"

#include "drivers/rgb.h"
#include "usb_dev_serial.h"
#include "protocol.h"
#include "remote.h"

#include"configADC.h"
#include "configUDMA.h"

#define LED1TASKPRIO 1
#define LED1TASKSTACKSIZE 128

//AJR - Variable para poner un punto de ruptura
static volatile uint16_t origen;

//AJR - Variable globales
uint32_t g_ui32CPUUsage;
uint32_t g_ulSystemClock;
uint8_t index;
//AJR - Semaforo/Mutex
xSemaphoreHandle Semaforo;
SemaphoreHandle_t xMutex;
//AJR - Flags de eventos
EventGroupHandle_t FlagsEventos, FlagsEventosDMA;

static uint8_t frame[MAX_FRAME_SIZE];
extern void vUARTTask( void *pvParameters );


//AJR - FLAGS PARA LAS ALARMAS
#define LED0_FLAG 0x0001
#define LED1_FLAG 0x0002
#define LED2_FLAG 0x0004
#define LED3_FLAG 0x0008
#define LED4_FLAG 0x0010
#define LED5_FLAG 0x0020
#define LED6_FLAG 0x0040
#define LED7_FLAG 0x0080

//AJR - FLAGS PARA LOS FLANCOS DE SUBIDA O BAJADA
#define ADC1_HIGH 0x0100
#define ADC2_HIGH 0x0200
#define ADC3_HIGH 0x0400
#define ADC4_HIGH 0x0800
#define ADC1_LOW  0x1000
#define ADC2_LOW  0x2000
#define ADC3_LOW  0x4000
#define ADC4_LOW  0x8000

//AJR - FLAG PARA EL ENVIO DE DATOS ANALÓGICOS
#define DMA0_FLAG 0x1000

//AJR - FLAGS PARA LA INTERRUPCION DEL COMPARADOR
#define COMP0 0x0001
#define COMP1 0x0002
#define COMP2 0x0004
#define COMP3 0x0008
#define COMP4 0x0010
#define COMP5 0x0020
#define COMP6 0x0040
#define COMP7 0x0080
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************

//Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
//
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
	//
	// This function can not return, so loop forever.  Interrupts are disabled
	// on entry to this function, so no processor interrupts will interrupt
	// this loop.
	//
	while(1)
	{
	}
}

//Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
void vApplicationTickHook( void )
{
	static unsigned char count = 0;

	if (++count == 10)
	{
		g_ui32CPUUsage = CPUUsageTick();
		count = 0;
	}
	//return;
}

//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationIdleHook (void)
{
	SysCtlSleep();
}


//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationMallocFailedHook (void)
{
	while(1);
}



//*****************************************************************************
//
// A continuacion van las tareas...
//
//*****************************************************************************

// El codigo de esta tarea esta definida en el fichero command.c, es la que se encarga de procesar los comandos del interprete a traves
// del terminal serie (puTTY)
//Aqui solo la declaramos para poderla referenciar en la funcion main
extern void vUARTTask( void *pvParameters );

//AJR - Tarea para el envio asincrono del estado de los botones
static portTASK_FUNCTION(BTNTask,pvParameters)
{

	PARAM_COMANDO_ESTADO_BTN parametro; //AJR - Aprovechamos el comando implementado anteriormente
	int32_t numdatos;
	uint8_t sw0, sw1;

	//
	// Bucle infinito, las tareas en FreeRTOS no pueden "acabar", deben "matarse" con la funcion xTaskDelete().
	//
	while(1){
			//AJR - Espera a que el semaforo este activo para leer el estado de los botones
			xSemaphoreTake(Semaforo, portMAX_DELAY);

			//AJR - Similar a la funcion implementada en remote.c para el envio del estado de los botones
			sw0 = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0);
			sw1 = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);

			parametro.swD = sw1;
			parametro.swI = sw0;

			numdatos = create_frame(frame,COMANDO_ESTADO_BTN,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
			if (numdatos>=0)
			{
				send_frame(frame,numdatos);
			}
	}
}

//AJR - Tarea que envia los datos del MICRO al PC
static portTASK_FUNCTION(ADCTask,pvParameters)
{

	int32_t numdatos;
	PARAM_COMANDO_MONITORIZAR parametro;
	EventBits_t status;
	//AJR - Inicializamos los valores del parametro para evitar enviar "basura"
	parametro.chan1 = 0; parametro.chan2 = 0; parametro.chan3 = 0; parametro.chan4 = 0;
	parametro.dig0 = 0; parametro.dig1 = 0; parametro.dig2 = 0; parametro.dig3 = 0;
	parametro.dig4 = 0; parametro.dig5 = 0; parametro.dig6 = 0; parametro.dig7 = 0;
	//
	// Bucle infinito, las tareas en FreeRTOS no pueden "acabar", deben "matarse" con la funcion xTaskDelete().
	//
	while(1)
	{
		//AJR - ESPERAMOS CUALQUIER TIPO DE EVENTO TANTO DEL COMPARADOR COMO DE LOS PINES DEL PUERTO B
		xEventGroupWaitBits(FlagsEventos, LED0_FLAG|LED1_FLAG|LED2_FLAG|LED3_FLAG|
							LED4_FLAG|LED5_FLAG|LED6_FLAG|LED7_FLAG|ADC1_HIGH|ADC2_HIGH|
							ADC3_HIGH|ADC4_HIGH|ADC1_LOW|ADC2_LOW|ADC3_LOW|ADC4_LOW,pdFALSE,pdFALSE,portMAX_DELAY);

		status = xEventGroupGetBits(FlagsEventos);

		//AJR - Limpiamos los flags correspondientes a los eventos que se han activado
		xEventGroupClearBits(FlagsEventos, status);

		//AJR - COMPROBAMOS SI HA SIDO ALGUNOS DE LOS PINES DEL PUERTO B
		if(status&LED0_FLAG)
			parametro.dig0 = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0);
		if(status&LED1_FLAG)
			parametro.dig1 = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_1);
		if(status&LED2_FLAG)
			parametro.dig2 = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_2);
		if(status&LED3_FLAG)
			parametro.dig3 = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_3);
		if(status&LED4_FLAG)
			parametro.dig4 = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_4);
		if(status&LED5_FLAG)
			parametro.dig5 = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_5);
		if(status&LED6_FLAG)
			parametro.dig6 = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_6);
		if(status&LED7_FLAG)
			parametro.dig7 = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_7);

		//AJR - COMPROBAMOS SI HA SIDO DE ALGUN COMPARADOR
		if(status&ADC1_HIGH) parametro.chan1 = 1;
		else if(status&ADC1_LOW)  parametro.chan1 = 0;
		if(status&ADC2_HIGH) parametro.chan2 = 1;
		else if(status&ADC2_LOW)  parametro.chan2 = 0;
		if(status&ADC3_HIGH) parametro.chan3 = 1;
		else if(status&ADC3_LOW)  parametro.chan3 = 0;
		if(status&ADC4_HIGH) parametro.chan4 = 1;
		else if(status&ADC4_LOW)  parametro.chan4 = 0;

		numdatos = create_frame(frame,COMANDO_MONITORIZAR,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
		if (numdatos>=0)
		{
			send_frame(frame,numdatos);
		}
	}
}

//AJR - Tarea que manda al PC los datos obtenidos de los pines analogicos
static portTASK_FUNCTION(DMATask,pvParameters)
{
	int32_t numdatos;
	PARAM_COMANDO_GRAFICA_REQUEST parametro;

	uint32_t muestras[4]; //AJR - Variable que utilizaremos para leer del secuenciador
	//
	// Bucle infinito, las tareas en FreeRTOS no pueden "acabar", deben "matarse" con la funcion xTaskDelete().
	//
	while(1)
	{
	//AJR - index controla que no leemos mas de 10 muestras por canal
	for(index = 0; index<10; index++){

		xEventGroupWaitBits(FlagsEventosDMA, DMA0_FLAG,pdTRUE,pdTRUE,portMAX_DELAY);

		ADCSequenceDataGet(ADC1_BASE,0,muestras);//AJR - COGEMOS LOS DATOS GUARDADOS

			//AJR - Pasamos de 32 bits a 16 (el conversor es de 12 bits, asi que solo son significativos los bits del 0 al 11)
			parametro.ch1[index]=muestras[0];
			parametro.ch2[index]=muestras[1];
			parametro.ch3[index]=muestras[2];
			parametro.ch4[index]=muestras[3];

			xEventGroupClearBits(FlagsEventosDMA, DMA0_FLAG);
	}
			numdatos = create_frame(frame,COMANDO_GRAFICA,&parametro,sizeof(parametro),MAX_FRAME_SIZE_ADC);
			if (numdatos>=0)
			{
				send_frame(frame,numdatos);
			}
	}
}

//*****************************************************************************
//
// Funcion main(), Inicializa los perifericos, crea las tareas, etc... y arranca el bucle del sistema
//
//*****************************************************************************
int main(void)
{

	//
	// Set the clocking to run at 40 MHz from the PLL.
	//
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
			SYSCTL_OSC_MAIN);	//Ponermos el reloj principal a 40 MHz (200 Mhz del Pll dividido por 5)


	// Get the system clock speed.
	g_ulSystemClock = SysCtlClockGet();


	//Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
	//                                                                        deben habilitarse con SysCtlPeripheralSleepEnable
	ROM_SysCtlPeripheralClockGating(true);

	// Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
	// Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
	// (y por tanto este no se deberia utilizar para otra cosa).
	CPUUsageInit(g_ulSystemClock, configTICK_RATE_HZ/10, 3);

	//
	// Inicializa la UARTy la configura a 115.200 bps, 8-N-1 .
	//se usa para mandar y recibir mensajes y comandos por el puerto serie
	// Mediante un programa terminal como gtkterm, putty, cutecom, etc...
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //AJR - Necesitaremos el puerto B

	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
	ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTStdioConfig(0,115200,SysCtlClockGet());

	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);	//La UART tiene que seguir funcionando aunque el micro este dormido
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);	//La UART tiene que seguir funcionando aunque el micro este dormido
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);
	ButtonsInit();
	
	//Inicializa los LEDs usando libreria RGB --> usa Timers 0 y 1 (eliminar si no se usa finalmente)
	RGBInit(1);
	SysCtlPeripheralSleepEnable(GREEN_TIMER_PERIPH);
	SysCtlPeripheralSleepEnable(BLUE_TIMER_PERIPH);
	SysCtlPeripheralSleepEnable(RED_TIMER_PERIPH);	//Redundante porque BLUE_TIMER_PERIPH y GREEN_TIMER_PERIPH son el mismo

	//AJR - Configura los pines del puerto GPIO B como entrada
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0);

	//AJR - Inicializa el semaforo y comprueba si ha habido error
	Semaforo = xSemaphoreCreateBinary();
	if (Semaforo == NULL) while(1);

	//AJR - Inicializa los eventos y comprueba si ha habido error
	FlagsEventos = xEventGroupCreate();
	if(FlagsEventos == NULL)
	{
		while(1);
	}
	//AJR - Inicializa los eventos y comprueba si ha habido error
		FlagsEventosDMA = xEventGroupCreate();
		if(FlagsEventosDMA == NULL) while(1);


	//AJR - Inicializa el semáforo/mútex y comprueba si ha habido error
    xMutex = xSemaphoreCreateMutex();
    if(xMutex == NULL)
    	while(1);

    //AJR - Llama a las funciones de configuración del ADC y del "DMA"(El nombre del último no es del todo correcto)
    //AJR - Estas funciones se encuentran en configADC.c y configDMA.c, respectivamente
	configADC_IniciaADC();
	DMA_IniciaDMA();

	//
	// Mensaje de bienvenida inicial.
	//
	UARTprintf("\n\nBienvenido a la aplicacion FreeRTOS-QT (curso 2016/17)!\n");
	UARTprintf("\nAutor: Adrian Juarez Rios");

	/**                                              Creacion de tareas 												**/

	// Crea la tarea que gestiona los comandos UART (definida en el fichero commands.c)
	//
	if((xTaskCreate(vUARTTask, (portCHAR *)"Uart", 512,NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE))
	{
		while(1);
	}

	//AJR - Crea la tarea que gestiona los eventos de los botones
	if((xTaskCreate(BTNTask, (portCHAR *)"Botones", 256,NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE))
	{
		while(1);
	}

	//AJR - Crea la tarea que gestiona el envío de las alarmas
	if((xTaskCreate(ADCTask, (portCHAR *)"ADC", 512,NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE))
	{
		while(1);
	}

	//AJR - Crea la tarea que gestiona el envío de los valores analógicos de los pines del ADC
	if((xTaskCreate(DMATask, (portCHAR *)"DMA", 256,NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE))
	{
		while(1);
	}

	UsbSerialInit(32,32);	//Inicializo el  sistema USB
	RemoteInit(); //Inicializo la aplicacion de comunicacion con el PC (Remote)

	//
	// Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
	//
	vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

	//De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.
	while(1)
	{
		//Si llego aqui es que algo raro ha pasado
	}
}

//AJR - Rutina de interrupcion de los botones
void RutinaISR(void){

	signed portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR(Semaforo, &pxHigherPriorityTaskWoken);

	GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0);
}

//AJR - Rutina de interrupcion de los pines del puerto B
void RutinaISR_B(void){

	BaseType_t xHigherPriorityTaskWoken=pdFALSE;
	uint32_t status = GPIOIntStatus(GPIO_PORTB_BASE, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|
			GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0);

	GPIOIntClear(GPIO_PORTB_BASE,status);

	if(status&GPIO_PIN_0)
		xEventGroupSetBitsFromISR(FlagsEventos, LED0_FLAG, &xHigherPriorityTaskWoken);
	if(status&GPIO_PIN_1)
		xEventGroupSetBitsFromISR(FlagsEventos, LED1_FLAG, &xHigherPriorityTaskWoken);
	if(status&GPIO_PIN_2)
		xEventGroupSetBitsFromISR(FlagsEventos, LED2_FLAG, &xHigherPriorityTaskWoken);
	if(status&GPIO_PIN_3)
		xEventGroupSetBitsFromISR(FlagsEventos, LED3_FLAG, &xHigherPriorityTaskWoken);
	if(status&GPIO_PIN_4)
		xEventGroupSetBitsFromISR(FlagsEventos, LED4_FLAG, &xHigherPriorityTaskWoken);
	if(status&GPIO_PIN_5)
		xEventGroupSetBitsFromISR(FlagsEventos, LED5_FLAG, &xHigherPriorityTaskWoken);
	if(status&GPIO_PIN_6)
		xEventGroupSetBitsFromISR(FlagsEventos, LED6_FLAG, &xHigherPriorityTaskWoken);
	if(status&GPIO_PIN_7)
		xEventGroupSetBitsFromISR(FlagsEventos, LED7_FLAG, &xHigherPriorityTaskWoken);

}

//AJR - Rutina de interrupcion del comparador ADC0
void RutinaISR_ADCComp(void){

	BaseType_t xHigherPriorityTaskWoken=pdFALSE;
	uint32_t status = ADCComparatorIntStatus(ADC0_BASE);
	ADCComparatorIntClear(ADC0_BASE, status);

	if((status&COMP0))
		xEventGroupSetBitsFromISR(FlagsEventos, ADC1_HIGH, &xHigherPriorityTaskWoken );
	if((status&COMP1))
		xEventGroupSetBitsFromISR(FlagsEventos, ADC2_HIGH, &xHigherPriorityTaskWoken );
	if((status&COMP2))
		xEventGroupSetBitsFromISR(FlagsEventos, ADC3_HIGH, &xHigherPriorityTaskWoken );
	if((status&COMP3))
		xEventGroupSetBitsFromISR(FlagsEventos, ADC4_HIGH, &xHigherPriorityTaskWoken );

	if((status&COMP4))
		xEventGroupSetBitsFromISR(FlagsEventos, ADC1_LOW, &xHigherPriorityTaskWoken );
	if((status&COMP5))
		xEventGroupSetBitsFromISR(FlagsEventos, ADC2_LOW, &xHigherPriorityTaskWoken );
	if((status&COMP6))
		xEventGroupSetBitsFromISR(FlagsEventos, ADC3_LOW, &xHigherPriorityTaskWoken );
	if((status&COMP7))
		xEventGroupSetBitsFromISR(FlagsEventos, ADC4_LOW, &xHigherPriorityTaskWoken );

}
//AJR - Rutina de interrupcion del ADC1
void RutinaISR_uDMA(void){
	BaseType_t xhigherPriorityTaskWoken=pdFALSE;

	ADCIntClear(ADC1_BASE, 0);

	xEventGroupSetBitsFromISR(FlagsEventosDMA, DMA0_FLAG , &xhigherPriorityTaskWoken );

	//AJR - NO ME HA DADO TIEMPO A IMPLEMENTAR EL uDMA
	//Desactivamos las interrupciones y deshabilitamos el canal y el sequencer
	/*ADCIntDisable(ADC1_BASE, 0);
	IntDisable(INT_ADC1SS0);
	ROM_uDMAChannelDisable(UDMA_CHANNEL_SSI1TX);
	ADCSequenceDisable(ADC1_BASE, 0);

	portEND_SWITCHING_ISR(xhigherPriorityTaskWoken);
*/
}
