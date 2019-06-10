#include <configUDMA.h>
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
#include "driverlib/udma.h"
#include "driverlib/comp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "remote.h"

//*****************************************************************************
//
// The control table used by the uDMA controller.  This table must be aligned
// to a 1024 byte boundary.
// Array con tabla de control para el DMA
// Por lo visto, se le puede poner 512 de tamanho si no se usa "ping-pong"
//
//*****************************************************************************
#if defined(ewarm)
#pragma data_alignment=1024
uint8_t ui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(ui8ControlTable, 1024)
uint8_t ui8ControlTable[1024];
#else
uint8_t ui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif



void DMA_IniciaDMA(void)
{
	//AJR - Inicializacion del ADC1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC1);

    //AJR - Configura un canal del DMA (canal 24)
	//AJR - Habilita el DMA y le asigna la zona de memoria de configuraciï¿½n
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);
	//ROM_uDMAEnable();
	//ROM_uDMAControlBaseSet(ui8ControlTable);

	//Configuramos la velocidad de conversion al maximo (1MS/s)
	ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_RATE_FULL, 1);

	//AJR - Inicializacion del timer 2
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER2);

	//AJR - Configuramos el timer 2
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);

	//AJR - CONFIGURAR ADC1 SECUENCIADOR 0
	ADCSequenceDisable(ADC1_BASE,0);

	//AJR - Asignamos el Timer A para disparar el ADC y lo habilitamos
	TimerControlTrigger(TIMER2_BASE, TIMER_A, true);
	TimerEnable(TIMER2_BASE, TIMER_A);

	//AJR - Secuencia que seguira el ADC1 Sequencer 0
	ADCSequenceConfigure(ADC1_BASE,0,ADC_TRIGGER_TIMER,0);	//AJR - Disparo hardware (timer2)
	ADCSequenceStepConfigure(ADC1_BASE,0,0,ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC1_BASE,0,1,ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC1_BASE,0,2,ADC_CTL_CH2);
	ADCSequenceStepConfigure(ADC1_BASE,0,3,ADC_CTL_CH3|ADC_CTL_IE|ADC_CTL_END );	//AJR - La ultima muestra provoca el disparo

	//Limpiamos interrupciones del ADC1 secuenciador 0
	ADCIntClear(ADC1_BASE,0);
	IntPrioritySet(INT_ADC1SS0,configMAX_SYSCALL_INTERRUPT_PRIORITY);

	/*******AJR - Posible implementación del uDMA******/
	/*uDMAChannelAssign(UDMA_CH24_ADC1_0);

	uDMAChannelAttributeDisable(UDMA_CHANNEL_SSI1RX,
										   UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
										   UDMA_ATTR_HIGH_PRIORITY |
										   UDMA_ATTR_REQMASK);

	uDMAChannelControlSet(UDMA_CHANNEL_SSI1RX | UDMA_PRI_SELECT,
									 UDMA_SIZE_16 | UDMA_SRC_INC_16 | UDMA_DST_INC_16 |
									 UDMA_ARB_32);

	uDMAChannelTransferSet(UDMA_CHANNEL_SSI1RX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG,
							   ((void *)ADC1_BASE + ADC_O_SSFIFO0),
								((void*)g_ui16DstBuf0), sizeof(g_ui16DstBuf0)/sizeof(uint16_t));

	ROM_uDMAChannelAttributeEnable(UDMA_CHANNEL_SSI1RX, UDMA_ATTR_USEBURST);

	ADCSequenceDMAEnable(ADC1_BASE, 0);
*/
}

