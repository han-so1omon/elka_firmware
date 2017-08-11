/**
  ******************************************************************************
  * @file    STM32F4-Discovery FreeRTOS demo\main.c
  * @author  T.O.M.A.S. Team
  * @version V1.1.0
  * @date    14-October-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "mpu6050.h"
#include "i2croutines.h"
#include <stdbool.h>
#include "imu.h"
#include "adc.h"




/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "FreeRTOSConfig.h"

#include "hw_config.h"
#include "motors.h"
#include "stabilizer.h"
#include "debug.h"
#include "nrf24l01.h"
#include "nRF24L01reg.h"

/** @addtogroup STM32F4-Discovery_Demo
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DELAY 10     /* msec */
#define queueSIZE	6
uint64_t u64Ticks=0;
uint64_t u64IdleTicks=0;    // Value of u64IdleTicksCnt is copied once per sec.
uint64_t u64IdleTicksCnt=0; // Counts when the OS has no task to execute.



/* Private macro -------------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Task functions declarations */
static void vLEDTask( void *pvParameters );
static void imuinit();
static void stabilizerTask(void* param);
portTASK_FUNCTION_PROTO( vDebugTask, pvParameters );
portTASK_FUNCTION_PROTO( vElkaRXAck, pvParameters ); // Task to wirelessly receive commands from base-station and transmit the values obtained from vGetData via wireless acknowledgement packet
portTASK_FUNCTION_PROTO( vGetData, pvParameters ); // Task to obtain gyro, accel and actuator data and transfer it to queue that can later be read by ElkaRxAck Task
portTASK_FUNCTION_PROTO( vSpektrumchannel_DSMX, pvParameters );

void nrfInitRXack(void);

//void USART2Init( void );



/* handlers to tasks to better control them */

xQueueHandle xDebugQueue;
xQueueHandle  tx1Queue,eulerqueue,spektrumqueue;
xQueueHandle xDebugQueue, uartData, adctransferQueue;
xSemaphoreHandle dataRdy, queuewritten;


Axis3f gyro; // Gyro axis data in deg/s
Axis3f acc;  // Accelerometer axis data in mG
Axis3i16   gyroMpu;
Axis3i16   accelMpu;
struct SpektrumStateStruct
{
	uint8_t LostFrame_cnt;
	uint8_t Sync;
	uint8_t ChannelCnt;
	uint8_t values[14];
};
typedef struct SpektrumStateStruct SpektrumStateType;
int SpektrumParser_DSMX(uint8_t c, SpektrumStateType* spektrum_state);
SpektrumStateType PrimarySpektrumState = {0,0,0,{0}};
int channeldata[7];
uint16_t spektrumchannel[7];

int channelnum;
int spektrum_check;

static const int LEDS[4][2] = {{LED3,DELAY*1},
							   {LED4,DELAY*2},
							   {LED5,DELAY*3},
							   {LED6,DELAY*4}};

/* semaphores, queues declarations */
xSemaphoreHandle xSemaphoreSW  = NULL;
xQueueHandle xQueue;

static void interruptCallback()
{
	portBASE_TYPE  xHigherPriorityTaskWoken = pdFALSE;
	//To unlock RadioTask
	xSemaphoreGiveFromISR(dataRdy, &xHigherPriorityTaskWoken);
	if(xHigherPriorityTaskWoken)
		vPortYieldFromISR();
}

/*Modify this to change packet structure to send*/
typedef struct
{
	uint8_t gyrobytes[6];
	//uint8_t accelbytes[6];
	uint8_t eulerangles[4];
	uint8_t commanded[16];
}imubytes;

imubytes imudata;

int16_t axi16, ayi16, azi16;
int16_t gxi16, gyi16, gzi16;

int main(void)
{

	/* initialize hardware... */
	prvSetupHardware();

	//vDebugInitQueue();
	stabilizerInit();
	USART1Init(115200);
	//USART2Init(38400);

	//init_tim();
	nrfInit(); //Initialize radio unit
	nrfInitRxAck(); //Setup radio with auto acknowledge - enhanced shockburst
	nrfSetInterruptCallback(interruptCallback);
	NVIC_Configuration();
	vSemaphoreCreateBinary(dataRdy); //create semaphore to release vElkaRxAck task. Serviced by external interrupt.
	vSemaphoreCreateBinary(queuewritten); // create semaphore to release vGetData task. Serviced by vElkaRxAck task.
	tx1Queue = xQueueCreate(3,sizeof(rxpacket)); //create queue to transfer packets received by radio to stabilizer task.
	eulerqueue = xQueueCreate(3,sizeof(eulerstruct)); //create queue to transfer imu data to be sent via radio.
	uartData = xQueueCreate(40, sizeof(uint8_t)); // create queue to read spektrum satellite receiver serial data.


    //adc_init_multi();

    bool i;
    uint16_t ratio;
    uint8_t data;
    uint8_t buffer[14];
    int j;


   // xTaskCreate(stabilizerTask, (const signed char * const)"STABILIZER",configMINIMAL_STACK_SIZE, NULL, /*Piority*/tskIDLE_PRIORITY, NULL);

    //xTaskCreate( vLEDTask, ( signed portCHAR * ) "LED3", configMINIMAL_STACK_SIZE, (void *)LEDS[0],tskIDLE_PRIORITY+1, NULL );
	xTaskCreate(vElkaRXAck, NULL,130, NULL, 2, NULL);
	xTaskCreate( vGetData, NULL, 130, NULL, 1, NULL );
	xTaskCreate (vSpektrumchannel_DSMX, NULL, 130, NULL, 3, NULL);
	//xTaskCreate( vDebugTask, (signed char *) "DEBUG", 130,NULL, 1, NULL );

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;
}


void vLEDTask( void *pvParameters )
{
    volatile int *LED;
    LED = (int *) pvParameters;
    bool i;
    uint16_t ratio;
    uint8_t data;
    uint8_t buffer[14];
    //imuinit();
    int16_t ax,ay,az,gx,gy,gz;


	for( ;; )
	{

		//nrfWrite1Reg(REG_CONFIG, 0x3f);
		//if (nrfRead1Reg(REG_CONFIG)>>5&1)
		{
			//STM_EVAL_LEDToggle(LED4);
			STM_EVAL_LEDToggle(LED3);
			//GPIOC->ODR ^= GPIO_Pin_2;
			//GPIOC->ODR ^= GPIO_Pin_3;
		}



	    //vTaskDelay(LED[1]/portTICK_RATE_MS);
        vTaskDelay(1000);
	}
}

portTASK_FUNCTION (vGetData, pvParameters)
{

	volatile unsigned long i;
	uint32_t xlastWakeTime1;
	portTickType xLastWakeTime1;
	xLastWakeTime1 = xTaskGetTickCount();

	static int led1_state = 0;
	rxpacket test;
	eulerstruct eulerdata;
	uint8_t ubyte, lbyte;
	adcstruct press_sens;

	xSemaphoreTake( queuewritten, 0 );

	for( ;; )
	{
		//vTaskDelayUntil(&xlastWakeTime1, 10);

		xSemaphoreTake(queuewritten, portMAX_DELAY);
		//xQueueReceive(adctransferQueue,&press_sens,0);
		xQueueReceive(eulerqueue,&eulerdata,0);
		gxi16 = (int)(100*gyro.x);
		gyi16 = (int)(100*gyro.y);
		gzi16 = (int)(100*gyro.z);
		axi16 = (int)(100*acc.x);
		ayi16 = (int)(100*acc.y);
		azi16 = (int)(100*acc.z);
		imudata.gyrobytes[0] = (gxi16>>8)&(0xff);
		imudata.gyrobytes[1] = (gxi16)&(0xff);
		imudata.gyrobytes[2] = (gyi16>>8)&(0xff);
		imudata.gyrobytes[3] = (gyi16)&(0xff);
		imudata.gyrobytes[4] = (gzi16>>8)&(0xff);
		imudata.gyrobytes[5] = (gzi16)&(0xff);
		imudata.eulerangles[0] = (eulerdata.data[0]>>8)&(0xff);
		imudata.eulerangles[1] = (eulerdata.data[0])&(0xff);
		imudata.eulerangles[2] = (eulerdata.data[1]>>8)&(0xff);
		imudata.eulerangles[3] = (eulerdata.data[1])&(0xff);

		imudata.commanded[0] = (eulerdata.data[3]>>8)&(0xff);
		imudata.commanded[1] = (eulerdata.data[3])&(0xff);
		imudata.commanded[2] = (eulerdata.data[4]>>8)&(0xff);
		imudata.commanded[3] = (eulerdata.data[4])&(0xff);
		imudata.commanded[4] = (eulerdata.data[5]>>8)&(0xff);
		imudata.commanded[5] = (eulerdata.data[5])&(0xff);
		imudata.commanded[6] = (eulerdata.data[6]>>8)&(0xff);
		imudata.commanded[7] = (eulerdata.data[6])&(0xff);
		/*imudata.commanded[8] = (press_sens.data[0]>>8)&(0xff);
	    imudata.commanded[9] = (press_sens.data[0])&(0xff);
		imudata.commanded[10] = (press_sens.data[1]>>8)&(0xff);
	    imudata.commanded[11] = (press_sens.data[1])&(0xff);
		imudata.commanded[12] = (press_sens.data[2]>>8)&(0xff);
	    imudata.commanded[13] = (press_sens.data[2])&(0xff);
		imudata.commanded[14] = (press_sens.data[3]>>8)&(0xff);
	    imudata.commanded[15] = (press_sens.data[3])&(0xff);*/
		imudata.commanded[8] = (eulerdata.data[9]>>8)&(0xff);
		imudata.commanded[9] = (eulerdata.data[9])&(0xff);
		imudata.commanded[10] = (eulerdata.data[10]>>8)&(0xff);
		imudata.commanded[11] = (eulerdata.data[10])&(0xff);
		imudata.commanded[12] = (eulerdata.data[11]>>8)&(0xff);
		imudata.commanded[13] = (eulerdata.data[11])&(0xff);
		imudata.commanded[14] = (eulerdata.data[12]>>8)&(0xff);
		imudata.commanded[15] = (eulerdata.data[12])&(0xff);

		//GPIO_WriteBit(GPIOB, GPIO_Pin_5, led1_state ? Bit_SET : Bit_RESET);
		//led1_state^=1;*/
	    //STM_EVAL_LEDToggle(LED4);
	    //vTaskDelay(100);
	}
}


portTASK_FUNCTION (vElkaRXAck, pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	uint8_t counter = 0;
	char *receive;
	unsigned char bytereceived[5];
	receive = &bytereceived[0];
	int led1_state = 0;
	int i, status;
	uint8_t status1, status2;
	char id1,id2,id3,id4, id5;
	static rxpacket pk, pknoreceive;
	unsigned char dataLen;
	for (i=0;i<TX_PAYLOAD_WIDTH; i++)
	{
		pknoreceive.data[i] = 0;
	}

	int *test;

	while(1)
	{
		xSemaphoreTake(dataRdy, portMAX_DELAY);
		//xSemaphoreTake(dataRdy, 1000);
		nrfSetEnable(false);
		if (nrfRead1Reg(REG_FIFO_STATUS)&2)
		{
			nrfFlushRx();
		}
		status1=nrfRead1Reg(REG_STATUS);
		if ((nrfRead1Reg(REG_FIFO_STATUS)>>5)&1)
		{
			nrfFlushTx();

		}

		// If the receive register flag is switched on, read the data into pk structure and
		// send it to tx1Queue which is used by stabilizer task to transfer gain and pilot inputs.
		// Simultaneously write an acknowledgement packet, stored in imudata, to be wirelessly transmitted to basestation.
		if ((nrfRead1Reg(REG_STATUS)>>6)&1)
		{

			dataLen = nrfRxLength(0);
			nrfReadRX((char *)pk.data,dataLen);
			status2 = nrfRead1Reg(REG_FIFO_STATUS);

			nrfWriteAck(0, &imudata,sizeof(imudata));

			xSemaphoreGive(queuewritten);

			xQueueSendToBack(tx1Queue,&pk,0);

		}
		else if ((nrfRead1Reg(REG_STATUS)>>6)&0)
		{
			xQueueSendToBack(tx1Queue,&pknoreceive,0);
			//STM_EVAL_LEDOff(LED3);
		}

		nrfWrite1Reg(REG_STATUS, 0x70); //clear interrupt bits
		nrfSetEnable(true);
		//vTaskDelay(7);
		STM_EVAL_LEDToggle(LED3);
	}
}

portTASK_FUNCTION (vSpektrumchannel_DSMX, pvParameters)
{
	volatile unsigned long i;
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	static int led2_state = 0;
	uint8_t c;
	int counter = 0;
	uint8_t check;
	//xSemaphoreTake(uartread, 0);

	for( ;; )
	{
		//xSemaphoreTake(uartread, portMAX_DELAY);
		if (xQueueReceive(uartData, &c, 10000) == pdTRUE)
		{

			check = SpektrumParser_DSMX(c, &PrimarySpektrumState);
			//STM_EVAL_LEDToggle(LED6);


		}

		if (check==3)
		{
			counter++;
			USART_SendData(USART3, check);
			if (counter>10)
			{
				STM_EVAL_LEDToggle(LED4);
				counter=0;
			}
			for(i=0;i<7;i++)
			{
				channeldata[i] = (PrimarySpektrumState.values[2*i]<<8)|(PrimarySpektrumState.values[2*i+1]);
			}

			for (i=0;i<7;i++)
			{
				channelnum = (channeldata[i] >> 11) & 0x0f;
				spektrumchannel[channelnum] = channeldata[i] & 0x7ff;
			}
			spektrumchannel[6] = 1;

			//xQueueSendToBack(spektrumqueue,&spektrumchannel,0);
		}
		else if (check !=3)
		{
			//spektrumchannel[6] = 0;
		}
		//vTaskDelayUntil( &xLastWakeTime, ( 20 / portTICK_RATE_MS  ) );
		//vTaskDelay(10);
	}
}

void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	for( ;; );
}
/*-----------------------------------------------------------*/

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

void vApplicationTickHook( void ) {
    ++u64Ticks;
}

// This FreeRTOS call-back function gets when no other task is ready to execute.
// On a completely unloaded system this is getting called at over 2.5MHz!
// ----------------------------------------------------------------------------
void vApplicationIdleHook( void ) {
    ++u64IdleTicksCnt;
}

/*All the initialization routines to setup nRF24L01+ as a PRX with Enhanced Shockburst. Details avaliable in datasheet */
void nrfInitRxAck()
{
	char radioAddress[5] = {0x34,0x43,0x10,0x10,0x01};

	//Set radio address
	nrfSetAddress(0, radioAddress);
	nrfWriteReg(REG_TX_ADDR, radioAddress,5);
	nrfWrite1Reg(REG_SETUP_AW,3);
	//nrfWrite1Reg(REG_RX_PW_P1,1);
	nrfWrite1Reg(REG_RX_PW_P0,TX_PAYLOAD_WIDTH);
	nrfWrite1Reg(REG_EN_AA,0x01); // enable auto acknowledge
	nrfWrite1Reg(REG_RF_CH,40); //channel 40
	nrfWrite1Reg(REG_EN_RXADDR,0x01);
	nrfWrite1Reg(REG_RF_SETUP,0x26);
	nrfWrite1Reg(REG_SETUP_RETR,0x7a); //retransmit packet after 1500 ms
	nrfSetEnable(true);
	// Enable the dynamic payload size and the ack payload for the pipe 0
	nrfWrite1Reg(REG_FEATURE, 0x06);
	nrfWrite1Reg(REG_DYNPD, 0x01);
	//Power the radio, Enable the DS interruption, set the radio in PRX mode
	nrfWrite1Reg(REG_CONFIG, 0x3f);
}

static portBASE_TYPE xHigherPriorityTaskWoken;
void USART3_IRQHandler(void)
{

	uint8_t b;
	int counter = 0;
	uint8_t k = 17;
	static int led1_state;
	xHigherPriorityTaskWoken = pdFALSE;
	if (USART_GetITStatus(USART3, USART_IT_RXNE)) // Received characters modify string
	{

		b = USART_ReceiveData(USART3);


		counter++;

		{
			//STM_EVAL_LEDToggle(LED6);
		}

		xQueueSendFromISR(uartData, &b, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );

	}


}

void USART1_IRQHandler(void)
{

	uint8_t b;
	int counter = 0;
	uint8_t k = 17;
	static int led1_state;
	xHigherPriorityTaskWoken = pdFALSE;
	if (USART_GetITStatus(USART1, USART_IT_RXNE)) // Received characters modify string
	{

		b = USART_ReceiveData(USART1);


		counter++;

		{
			//STM_EVAL_LEDToggle(LED6);
		}

		xQueueSendFromISR(uartData, &b, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );

	}


}

//decoding information for spektrum dx6i and dsm2 satellite receiver
int SpektrumParser_DSMX(uint8_t c, SpektrumStateType* spektrum_state)
{
  int k;
    //spektrum_state->Sync = 1;

	if (c==0xA2)
	{
		spektrum_state->LostFrame_cnt = c;
		spektrum_state->Sync = 1;
		spektrum_state->ChannelCnt = 0;
		k=3;
		return;
	}

	//else if (c)
	/*if (spektrum_state->Sync ==1 && c==0xA2)
	{
		spektrum_state->Sync = 2;
		spektrum_state->ChannelCnt = 0;
		return;
	}*/
	if (spektrum_state->Sync==1)// && spektrum_state->LostFrame_cnt == 1)
	{
		spektrum_state->values[spektrum_state->ChannelCnt] = c;
		spektrum_state->ChannelCnt++;

	}
	if (spektrum_state->ChannelCnt>=14 )
	{
		spektrum_state->ChannelCnt = 0;
		spektrum_state->Sync = 3;
		spektrum_state->LostFrame_cnt = 0;
	}
	//k = spektrum_state->Sync;
	return k;

}

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */
  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  /* Enable the USART3 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


