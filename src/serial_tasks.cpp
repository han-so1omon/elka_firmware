/*
 * serial_tasks.c
 *
 *  Created on: Apr 18, 2016
 *      Author: Vik
 */

#include <serial_tasks.h>
#include "stm32f4xx_conf.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "math.h"
#include "semphr.h"
#include "stm32f4_discovery.h"
#include "stabilizer.h"

static void vLEDTask( void *pvParameters );
static void vSerialWrite( void *pvParameters );
ROS_Type PrimaryROSState = {0,{0}};
//extern cb le_snap;


void rosSerialInit()
{
	//txQueue_xbee = xQueueCreate(1,sizeof(rx_xbee));
	xTaskCreate( vLEDTask, NULL, configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY+1, NULL );
	xTaskCreate( vSerialWrite, NULL, configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY+1, NULL );

}

void vLEDTask( void *pvParameters )
{

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	static int led2_state = 0;
	//uint8_t c[32];
	uint8_t check;
	//uint8_t dummy[5] = {1,2,3,4,5};
	uint8_t i=0;
	char j[20];
	int txlen;
	rx_xbee dummy;
	uint8_t snap_index = 0;

	uint8_t read_index;

	for( ;; )
	{
		//xSemaphoreTake(uartread, portMAX_DELAY);
		//if (xQueueReceive(uartData, &c, 10000) == pdTRUE)
		//txlen = rx_buf[0];
		//for (i=0;i<txlen-1;i++)
		//{
		//check = ROS_parser(rx_buf,&PrimaryROSState);


		//if(check==3)
		//	{
		//		if(rx_buf.data[1]==0)
		//		{
		//			if(rx_buf.data[2]==255)
		//			{
		//				for (i=0;i<4;i++)
		//				{
		//					roschannel[i] = (rx_buf.data[2*i+3]<<8)|(rx_buf.data[2*i+4]);
		//				}
		//			}
		//		}




		//	}

		//roschannel[0]=txlen;
		//roschannel[1]=rx_buf[2];
		//roschannel[3] = PrimaryROSState.Sync;
		//roschannel[2]=rx_buf[3];





		//xQueueSend(txQueue_xbee,&dummy,0);
//		for(snap_index = 0;snap_index<=BUFFER_SIZE1;snap_index++)
//		{
//			snap_temp[snap_index] = readBuffer(&le_snap);
//			if(snap_index>3 && snap_index<(BUFFER_SIZE1-3) && snap_temp[snap_index-2] == TRIM && snap_temp[snap_index-1] == 255 && snap_temp[snap_index]==255)
//			{
//				read_index = snap_index;
//			}
//		}

//		snapCh[0] = (snap_temp[read_index % BUFFER_SIZE1]<<8)+snap_temp[(read_index+1) % BUFFER_SIZE1];
//		snapCh[1] = (snap_temp[(read_index+2) % BUFFER_SIZE1]<<8)+snap_temp[(read_index+3) % BUFFER_SIZE1];
//		snapCh[2] = (snap_temp[(read_index+4) % BUFFER_SIZE1]<<8)+snap_temp[(read_index+5) % BUFFER_SIZE1];
//		snapCh[3] = (snap_temp[(read_index+6) % BUFFER_SIZE1]<<8)+snap_temp[(read_index+7) % BUFFER_SIZE1];



		vTaskDelay(40);

	}
}

void vSerialWrite( void *pvParameters )
{

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	static int led2_state = 0;
	uint8_t c;
	uint8_t check;
	uint8_t dummy[5] = {1,2,3,4,5};
	uint8_t i=0;
	char j[20], *pos = j;
	char buffer[128];
	char formattedints[128];
	int index = 0;


	for( ;; )
	{

		sprintf(j, "#,%d,%d,%d,",100,100,100);
		//sprintf(j,"Continuous data \r\n");
		//sprintf(buffer,"#,%s,", format_uint_array(formattedints,&imudata,26,",","%d%s"));
		// USART_SendData(USART1, '#');
		//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

		//USART_puts1(&imudata,26);

		//		        while( !(USART1->SR & 0x00000040) );
		//		        USART_SendData(USART1, '#');
		//		        USART_puts1(USART1, &imudata,10);
		//		        while( !(USART1->SR & 0x00000040) );
		//		        USART_SendData(USART1, '\r');
		//		        while( !(USART1->SR & 0x00000040) );
		//		        USART_SendData(USART1, '\n');

		while( !(USART1->SR & 0x00000040) );
		USART_SendData(USART1, '#');
		USART_puts1(USART1, &imudata,30);
		//USART_puts1(USART1, &snap_buf.data,30);
		while( !(USART1->SR & 0x00000040) );
		USART_SendData(USART1, '\r');
		while( !(USART1->SR & 0x00000040) );
		USART_SendData(USART1, '\n');
		//while( !(USART3->SR & 0x00000040) );
		//USART_SendData(USART3, '\n');


		//index +=sprintf(&buffer[index],"%d",imudata[i]);
		i++;
		if (i>100)
		{
			i=0;
		}
		// USART_puts(j);




		STM_EVAL_LEDToggle(LED4);
		vTaskDelay(40);

	}
}

void USART1_IRQHandler(void)
{

	uint8_t b;
	static int counter=0;
	static int rx_buffer = 0;
	static int tx_buffer = 0;
	static portBASE_TYPE xHigherPriorityTaskWoken;
	//uint8_t tx_buf[32];

	int txlen;

	int i =0;
	xHigherPriorityTaskWoken = pdFALSE;
	uint8_t tx_buf[5]={1,2,3,4,5};

	if (USART_GetITStatus(USART1, USART_IT_RXNE)) // Received characters modify string
	{

		rx_buf.data[rx_buffer++] = USART_ReceiveData(USART1);

		//b = USART_ReceiveData(USART1);
		txlen = rx_buf.data[0];
		if (rx_buffer>txlen)
		{
			rx_buffer = 0;
			//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		}

//				b = USART_ReceiveData(USART1);
//				writeBuffer(&le_snap,b);


		//xQueueSendFromISR(txQueue_xbee, &rx_buf, &xHigherPriorityTaskWoken);

		portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );

	}
}

void USART3_IRQHandler(void)
{

	uint8_t b;
	static int counter=0;
	static int rx_buffer = 0;
	static int tx_buffer = 0;
	static portBASE_TYPE xHigherPriorityTaskWoken;
	rx_xbee snap_temp;
	//uint8_t tx_buf[32];

	int txlen;

	int i =0;
	xHigherPriorityTaskWoken = pdFALSE;
	uint8_t tx_buf[5]={1,2,3,4,5};

	if (USART_GetITStatus(USART3, USART_IT_RXNE)) // Received characters modify string
	{

				snap_buf.data[rx_buffer++] = USART_ReceiveData(USART3);

				//b = USART_ReceiveData(USART1);
				txlen = snap_buf.data[0];
				if (rx_buffer>txlen)
				{
					rx_buffer = 0;
					//memset(snap_temp.data,0,32);

					//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
				}

		//b = USART_ReceiveData(USART3);
		//writeBuffer(&le_snap,b);


		//xQueueSendFromISR(txQueue_xbee, &rx_buf, &xHigherPriorityTaskWoken);

		portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );

	}
}

void USART_puts(USART_TypeDef* USARTx, volatile char *s){

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, *s);
		*s++;
	}
}

void USART_puts1(USART_TypeDef* USARTx, uint8_t *data, int length){

	int i;
	for (i=0;i<length;i++){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, data[i]);

	}
}

char* format_uint_array(char *b, uint8_t* data, int length, char* delim, char* fmt)
{
	int i;
	char s[50];
	b[0] = 0;
	for( i = 0; i < length; i++ )
	{
		s[0]=0;
		sprintf( s, fmt, data[i], (i<length-1)?delim : "");
		strcat(b, s);
	}
	return b;
}

int ROS_parser(uint8_t* c, ROS_Type* ROS_state)
{
	int k;
	int txlen;
	int i;
	txlen = c[0];
	if (c[1]==0)
	{
		ROS_state->Sync = 1;
	}
	else if((c[2]==255) &  (ROS_state->Sync==1))
	{
		ROS_state->Sync = 2;
	}
	//	else if(c==0xFF & ROS_state->Sync==2)
	//	{
	//		ROS_state->Sync = 3;
	//	}
	else if(ROS_state->Sync==2)
	{
		for (i=0;i<txlen-3;i++)
		{

			ROS_state->values[ROS_state->ChannelCnt] = c[i+3];
			ROS_state->ChannelCnt++;
		}
		if(ROS_state->ChannelCnt==8)
		{
			ROS_state->Sync = 3;
			ROS_state->ChannelCnt = 0;
		}
	}
	k = ROS_state->Sync;
	return k;
}

