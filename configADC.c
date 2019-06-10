#include<stdint.h>
#include<stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_adc.h"
#include "inc/hw_comp.h"
#include "inc/hw_udma.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "configADC.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "remote.h"


//Este fichero implementa la configuracion del ADC y la ISR asociada. Las tareas pueden llamar a la funcion configADC_LeeADC (bloqueante) para leer datos del ADC
//La funcion configADC_DisparaADC(...) (no bloqueante) realiza el disparo software del ADC
//La funcion configADC_IniciaADC realiza la configuraciï¿½n del ADC: Los Pines E0 a E3 se ponen como entradas analï¿½gicas (AIN3 a AIN0 respectivamente). Ademas crea la cola de mensajes necesaria para que la funcion configADC_LeeADC sea bloqueante


void configADC_IniciaADC(void)
{
				//AJR - Inicializacion del ADC0
			    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
			    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);

				//HABILITAMOS EL GPIOE
				SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
				SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);
				// Enable pin PE3 for ADC AIN0|AIN1|AIN2|AIN3
				GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0);

				//AJR - CONFIGURAR ADC0 SECUENCIADOR 0 (es el único secuenciador con 8 pasos)
				ADCSequenceDisable(ADC0_BASE,0);

				//Configuramos la velocidad de conversion al maximo (1MS/s)
				ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_FULL, 1);

				ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_ALWAYS,0);	//AJR - Disparo siempre

				//AJR - CONFIGURAMOS LOS PASOS DEL ADC0 SECUENCER 0
				/*AJR - INCISO--> Se ha intentado que los comparadores 0-3 controlaran los flancos de subida y los comparadores 4-7 los de bajada,
				 * se ha conseguido*/
				ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0|ADC_CTL_CMP0);
				ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH1|ADC_CTL_CMP1);
				ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH2|ADC_CTL_CMP2);
				ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH3|ADC_CTL_CMP3);

				ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH0|ADC_CTL_CMP4);
				ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_CH1|ADC_CTL_CMP5);
				ADCSequenceStepConfigure(ADC0_BASE, 0, 6, ADC_CTL_CH2|ADC_CTL_CMP6);
				ADCSequenceStepConfigure(ADC0_BASE, 0, 7, ADC_CTL_CH3|ADC_CTL_CMP7|ADC_CTL_END);

				//AJR - CONFIGURAMOS LA INTERRUPCION DEL COMPARADOR DEL ADC0
				//AJR - Interrupción cuando pase a la zona alta (supere el umbral)
				ADCComparatorConfigure(ADC0_BASE, 0, ADC_COMP_INT_HIGH_ONCE);
				ADCComparatorConfigure(ADC0_BASE, 1, ADC_COMP_INT_HIGH_ONCE);
				ADCComparatorConfigure(ADC0_BASE, 2, ADC_COMP_INT_HIGH_ONCE);
				ADCComparatorConfigure(ADC0_BASE, 3, ADC_COMP_INT_HIGH_ONCE);

				//AJR - Interrupción cuando pase a la zona baja (pase por debajo del umbral)
				ADCComparatorConfigure(ADC0_BASE, 4, ADC_COMP_INT_LOW_ONCE);
				ADCComparatorConfigure(ADC0_BASE, 5, ADC_COMP_INT_LOW_ONCE);
				ADCComparatorConfigure(ADC0_BASE, 6, ADC_COMP_INT_LOW_ONCE);
				ADCComparatorConfigure(ADC0_BASE, 7, ADC_COMP_INT_LOW_ONCE);

				//AJR - Establece los umbrales iniciales de los comparadores al maximo
				ADCComparatorRegionSet(ADC0_BASE, 0,4095,4095);
				ADCComparatorRegionSet(ADC0_BASE, 1,4095,4095);
				ADCComparatorRegionSet(ADC0_BASE, 2,4095,4095);
				ADCComparatorRegionSet(ADC0_BASE, 3,4095,4095);

				ADCComparatorRegionSet(ADC0_BASE, 4,4095,4095);
				ADCComparatorRegionSet(ADC0_BASE, 5,4095,4095);
				ADCComparatorRegionSet(ADC0_BASE, 6,4095,4095);
				ADCComparatorRegionSet(ADC0_BASE, 7,4095,4095);

				ADCComparatorIntClear(ADC0_BASE, 0xFF);

				IntPrioritySet(INT_ADC0SS0,configMAX_SYSCALL_INTERRUPT_PRIORITY);

				IntEnable(INT_ADC0SS0);

				ADCIntClear(ADC0_BASE, 0);

}


