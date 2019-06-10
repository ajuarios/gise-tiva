/*
 * remote.c
 *
 *  Created on: 1/4/2016
 *      Author: jcgar
 */

#include"remote.h"

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
#include "drivers/rgb.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "driverlib/udma.h"

static uint8_t frame[MAX_FRAME_SIZE];	//Usar una global permite ahorrar pila en la tarea, pero hay que tener cuidado!!!!
static uint32_t gRemoteProtocolErrors=0;

//Defino a un tipo que es un puntero a funcion con el prototipo que tienen que tener las funciones que definamos
typedef int32_t (*remote_fun)(uint32_t param_size, void *param);


//Funcion que se ejecuta cuando llega un paquete indicando comando rechazado
int32_t ComandoRechazadoFun(uint32_t param_size, void *param)
{
	//He recibido "Comando rechazado" desde el PC
	//TODO, por hacer: Tratar dicho error??
	gRemoteProtocolErrors++;
	return 0;
}


//Funcion que se ejecuta cuando llega un PING
int32_t ComandoPingFun(uint32_t param_size, void *param)
{
	int32_t numdatos;

	numdatos=create_frame(frame,COMANDO_PING,0,0,MAX_FRAME_SIZE);
	if (numdatos>=0)
	{
		send_frame(frame,numdatos);
	}

	return numdatos;
}


//AJR - Funcion que se ejecuta cuando llega el comando que configura los LEDS
int32_t ComandoLedsFun(uint32_t param_size, void *param)
{
	PARAM_COMANDO_LEDS parametro;

	if (check_and_extract_command_param(param, param_size, sizeof(parametro),&parametro)>0)
	{
		//AJR - Si alguno de los parametros está activo, enciende el led correspondiente
		if(parametro.leds.red)
			GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_PIN_1);
		else GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0);

		if(parametro.leds.green)
			GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,GPIO_PIN_3);
		else GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,0);

		if(parametro.leds.blue)
			GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,GPIO_PIN_2);
		else GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,0);

		return 0;	//Devuelve Ok (valor mayor no negativo)
	}
	else
	{
		return PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
	}
}

//AJR - Funcion que se ejecuta cuando llega el comando que configura el BRILLO
int32_t ComandoBrilloFun(uint32_t param_size, void *param)
{
	PARAM_COMANDO_BRILLO parametro;

	if (check_and_extract_command_param(param, param_size, sizeof(parametro),&parametro)>0)
	{
		RGBIntensitySet(parametro.rIntensity);
		return 0;	//Devuelve Ok (valor mayor no negativo)
	}
	else
	{
		return PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
	}
}

//AJR - Funcion que se ejecuta cuando llega el comando que configura los LEDS RGB
int32_t ComandoLedsRGBFun(uint32_t param_size, void *param)
{
	PARAM_COMANDO_LEDS_RGB parametro;
	uint32_t arrayRGB[3];
	if (check_and_extract_command_param(param, param_size, sizeof(parametro),&parametro)>0)
	{
		arrayRGB[0] = parametro.red<<8;
		arrayRGB[1] = parametro.green<<8;
		arrayRGB[2] = parametro.blue<<8;

		RGBColorSet(arrayRGB);

		return 0;	//Devuelve Ok (valor mayor no negativo)
	}
	else
	{
		return PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
	}
}

//AJR - Funcion que se ejecuta cuando llega el comando que pregunta el estado de los botones
int32_t ComandoEstadoBtnFun(uint32_t param_size, void *param)
{
	PARAM_COMANDO_ESTADO_BTN parametro;

	int32_t numdatos;
	uint32_t sw1, sw2;

	//AJR - Comprueba que boton(es) esta(n) pulsado(s)
	sw1 = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0);
	sw2 = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);

	parametro.swD = sw2;
	parametro.swI = sw1;

	numdatos = create_frame(frame,COMANDO_ESTADO_BTN,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
	if (numdatos>=0)
	{
		send_frame(frame,numdatos);
	}

	return numdatos;
}

//AJR - Funcion que se ejecuta cuando llega el comando que cambia al modo GPIO
int32_t ComandoCambioModoFun(uint32_t param_size, void *param)
{
	PARAM_COMANDO_CAMBIO_MODO parametro;

	if (check_and_extract_command_param(param, param_size, sizeof(parametro),&parametro)>0)
		{
			if(parametro.modo)
			{
				RGBDisable();
				ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
			}
			else
				RGBEnable();

			return 0;	//Devuelve Ok (valor mayor no negativo)
		}
		else
		{
			return PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
		}
}

//AJR - Funcion que se ejecuta cuando llega el comando que permite el envio asincrono del estado de los botones
int32_t ComandoAsincronoFun(uint32_t param_size, void *param)
{
	PARAM_COMANDO_ASINCRONO parametro;
		if (check_and_extract_command_param(param, param_size, sizeof(parametro),&parametro)>0)
		{
			if(parametro.asincrono)
			{
				GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0);
				GPIOIntEnable(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0);
				GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0,GPIO_BOTH_EDGES);
				IntEnable(INT_GPIOF);
			}
			else{
				IntDisable(INT_GPIOF);
			}
			return 0;	//Devuelve Ok (valor mayor no negativo)
		}
		else
		{
			return PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
		}
}

//AJR - Funcion que se ejecuta cuando llega el comando de Alarmas (MONITORIZAR)
int32_t ComandoMonitorizarFun(uint32_t param_size, void *param)
{
	PARAM_COMANDO_MONITORIZAR parametro;

		if (check_and_extract_command_param(param, param_size, sizeof(parametro),&parametro)>0)
		{
			if(parametro.mode)
			{
				//AJR - Digitales
				if(parametro.dig0)
				{
					GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_0);
					GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_0);
					GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0,GPIO_BOTH_EDGES);
				}
				else GPIOIntDisable(GPIO_PORTB_BASE, GPIO_PIN_0);
				if(parametro.dig1)
				{
					GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_1);
					GPIOIntEnable(GPIO_PORTB_BASE,GPIO_PIN_1);
					GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_1,GPIO_BOTH_EDGES);
				}
				else GPIOIntDisable(GPIO_PORTB_BASE, GPIO_PIN_1);
				if(parametro.dig2)
				{
					GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_2);
					GPIOIntEnable(GPIO_PORTB_BASE,GPIO_PIN_2);
					GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2,GPIO_BOTH_EDGES);
				}
				else GPIOIntDisable(GPIO_PORTB_BASE, GPIO_PIN_2);
				if(parametro.dig3)
				{
					GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_3);
					GPIOIntEnable(GPIO_PORTB_BASE,GPIO_PIN_3);
					GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_3,GPIO_BOTH_EDGES);
				}
				else GPIOIntDisable(GPIO_PORTB_BASE, GPIO_PIN_3);
				if(parametro.dig4)
				{
					GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_4);
					GPIOIntEnable(GPIO_PORTB_BASE,GPIO_PIN_4);
					GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_4,GPIO_BOTH_EDGES);
				}
				else GPIOIntDisable(GPIO_PORTB_BASE, GPIO_PIN_4);
				if(parametro.dig5)
				{
					GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_5);
					GPIOIntEnable(GPIO_PORTB_BASE,GPIO_PIN_5);
					GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_5,GPIO_BOTH_EDGES);
				}
				else GPIOIntDisable(GPIO_PORTB_BASE, GPIO_PIN_5);
				if(parametro.dig6)
				{
					GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_6);
					GPIOIntEnable(GPIO_PORTB_BASE,GPIO_PIN_6);
					GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_6,GPIO_BOTH_EDGES);
				}
				else GPIOIntDisable(GPIO_PORTB_BASE, GPIO_PIN_6);
				if(parametro.dig7)
				{
					GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_7);
					GPIOIntEnable(GPIO_PORTB_BASE,GPIO_PIN_7);
					GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_7,GPIO_BOTH_EDGES);
				}
				else GPIOIntDisable(GPIO_PORTB_BASE, GPIO_PIN_7);
				IntEnable(INT_GPIOB);

				//AJR - Analogicos

				ADCComparatorIntClear(ADC0_BASE, 0x0F);
				if(parametro.chan1)
				{
					ADCComparatorRegionSet(ADC0_BASE, 0, (parametro.chan1), (parametro.chan1));
					ADCComparatorRegionSet(ADC0_BASE, 4, (parametro.chan1), (parametro.chan1));
				}
				if(parametro.chan2)
				{
					ADCComparatorRegionSet(ADC0_BASE, 1, (parametro.chan2), (parametro.chan2));
					ADCComparatorRegionSet(ADC0_BASE, 5, (parametro.chan2), (parametro.chan2));
				}
				if(parametro.chan3)
				{
					ADCComparatorRegionSet(ADC0_BASE, 2, (parametro.chan3), (parametro.chan3));
					ADCComparatorRegionSet(ADC0_BASE, 6, (parametro.chan3), (parametro.chan3));
				}
				if(parametro.chan4)
				{
					ADCComparatorRegionSet(ADC0_BASE, 3, (parametro.chan4), (parametro.chan4));
					ADCComparatorRegionSet(ADC0_BASE, 7, (parametro.chan4), (parametro.chan4));
				}

				if(parametro.chan1 || parametro.chan2 || parametro.chan3 || parametro.chan4)
				{
					ADCSequenceEnable(ADC0_BASE,0);
					ADCIntEnable(ADC0_BASE, 0);
					IntEnable(INT_ADC0SS0);
					ADCComparatorIntEnable(ADC0_BASE, 0);
				}
			}
			else
			{
				ADCComparatorIntDisable(ADC0_BASE, 0);
				ADCIntDisable(ADC0_BASE, 0);
				IntDisable(INT_GPIOB);
			}
			return 0;	//Devuelve Ok (valor mayor no negativo)
		}
		else
		{
			return PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
		}
}

//AJR - Funcion que se ejecuta cuando llega el comando que permite la lectura de los pines analógicos
int32_t ComandoGraficaFun(uint32_t param_size, void *param)
{
	PARAM_COMANDO_GRAFICA parametro;
	float frec;
			if (check_and_extract_command_param(param, param_size, sizeof(parametro),&parametro)>0)
			{
				if(parametro.mode)
				{
				    frec = ((SysCtlClockGet()/parametro.frec)-1); //Conversion de la frecuencia de entrada
				    TimerLoadSet(TIMER2_BASE, TIMER_A, frec);
				    ADCIntEnable(ADC1_BASE, 0);
				    IntEnable(INT_ADC1SS0);
				    ADCSequenceEnable(ADC1_BASE, 0);
				    //ROM_uDMAChannelEnable(UDMA_CHANNEL_SSI1RX);
				}
				else
				{
					ADCIntDisable(ADC1_BASE, 0);
					IntDisable(INT_ADC1SS0);
				}
				return 0;	//Devuelve Ok (valor mayor no negativo)
			}
			else
			{
				return PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
			}
}

//Funcion que se ejecuta cuando recibimos un comando que no tenemos aun implementado
int32_t ComandoNoImplementadoFun(uint32_t param_size, void *param)
{
	return PROT_ERROR_UNIMPLEMENTED_COMMAND; /* Devuelve un error para que lo procese la tarea que recibe los comandos */
}


/* Array que contiene las funciones que se van a ejecutar en respuesta a cada comando */
static const remote_fun remote_fun_array[]=
{
		ComandoRechazadoFun, 	 /* Responde al paquete comando rechazado */
		ComandoPingFun, 	     /* Responde al comando ping */
		ComandoLedsFun, 	 	 /* Responde al comando LEDS */
		ComandoBrilloFun, 	 	 /* AJR - Responde al comando BRILLO*/
		ComandoLedsRGBFun,		 /* AJR - Responde al comando LEDS_RGB*/
		ComandoEstadoBtnFun,	 /* AJR - Responde al comando ESTADO_BTN*/
		ComandoCambioModoFun,	 /* AJR - Responde al comando CAMBIO_MODO*/
		ComandoAsincronoFun,	 /* AJR - Responde al comando ASINCRONO*/
		ComandoMonitorizarFun, 	 /* AJR - Responde al comando MONITORIZAR*/
		ComandoGraficaFun,		 /* AJR - Responde al comando GRAFICA*/
		ComandoNoImplementadoFun /* Responde si no esta implementado*/
};

// Codigo para procesar los comandos recibidos a traves del canal USB del micro ("conector lateral")

//Esta tarea decodifica los comandos y ejecuta la funcion que corresponda a cada uno de ellos (por posicion)
//Tambien gestiona posibles errores en la comunicacion
static portTASK_FUNCTION( CommandProcessingTask, pvParameters ){

	//Frame es global en este fichero, se reutiliza en las funciones que envian respuestas ---> CUIDADO!!!

	int32_t numdatos;
	uint8_t command;
	void *ptrtoparam;

	/* The parameters are not used. (elimina el warning)*/
	( void ) pvParameters;

	for(;;)
	{
		numdatos=receive_frame(frame,MAX_FRAME_SIZE);
		if (numdatos>0)
		{	//Si no hay error, proceso la trama que ha llegado.
			numdatos=destuff_and_check_checksum(frame,numdatos);
			if (numdatos<0)
			{
				//Error de checksum (PROT_ERROR_BAD_CHECKSUM), ignorar el paquete
				gRemoteProtocolErrors++;
				// Procesamiento del error (TODO, POR HACER!!)
			}
			else
			{
				//El paquete esta bien, luego procedo a tratarlo.
				command=decode_command_type(frame);
				numdatos=get_command_param_pointer(frame,numdatos,&ptrtoparam);

				if (command<(sizeof(remote_fun_array)/sizeof(remote_fun)))
				{
					switch(remote_fun_array[command](numdatos,ptrtoparam))
					{
						//La funcion puede devolver cï¿½digos de error.
					    //Se procesarï¿½an a continuaciï¿½n
						case PROT_ERROR_NOMEM:
						{
							// Procesamiento del error NO MEMORY (TODO, por hacer)
						}
						break;
						case PROT_ERROR_STUFFED_FRAME_TOO_LONG:
						{
							// Procesamiento del error STUFFED_FRAME_TOO_LONG (TODO, por hacer)
						}
						break;
						case PROT_ERROR_COMMAND_TOO_LONG:
						{
							// Procesamiento del error COMMAND TOO LONG (TODO, por hacer)
						}
						break;
						case PROT_ERROR_INCORRECT_PARAM_SIZE:
						{
							// Procesamiento del error INCORRECT PARAM SIZE (TODO, por hacer)
						}
						break;
						case PROT_ERROR_UNIMPLEMENTED_COMMAND:
						{
							PARAM_COMANDO_RECHAZADO parametro;

							parametro.command=command;
							//El comando esta bien pero no esta implementado
							numdatos=create_frame(frame,COMANDO_RECHAZADO,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
							if (numdatos>=0)
							{
									send_frame(frame,numdatos);
							}
						}
						break;
						//Aï¿½adir casos de error aqui...
						default:
							/* No hacer nada */
							break;
					}
				}
				else
				{
						/* El comando no es reconocido por el microcontrolador */
						ComandoNoImplementadoFun(numdatos,ptrtoparam);
						gRemoteProtocolErrors++;
				}
			}
		}
		else
		{ // if (numdatos >0)
				//Error de recepcion de trama(PROT_ERROR_RX_FRAME_TOO_LONG), ignorar el paquete
				gRemoteProtocolErrors++;
				// Procesamiento del error (TODO)
		}
	}
}


//Inicializa la tarea que recibe comandos (se debe llamar desde main())
void RemoteInit(void)
{
	//
	// Crea la tarea que gestiona los comandos USB (definidos en CommandProcessingTask)
	//
	if(xTaskCreate(CommandProcessingTask, (portCHAR *)"usbser",REMOTE_TASK_STACK, NULL, REMOTE_TASK_PRIORITY, NULL) != pdTRUE)
	{
		while(1);
	}

}
