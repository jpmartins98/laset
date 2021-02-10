/**
  ******************************************************************************
  * @file    main.c
  * @author  3S0 FreeRTOs
  * @version V1.0
  * @date    24/10/2017
  * @brief   FreeRTOS Example project.
  ******************************************************************************
*/


/*
 *
 * Messages Queues
 * 2017-2018
 *
 */

/* Standard includes. */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <lcd.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lcd.h"

#include "queue.h"

/* Task priorities. */
#define mainFLASH_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)
#define mainUSART_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)
#define mainLCD_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)

/* The rate at which the flash task toggles the LED. */
#define mainFLASH_DELAY			( ( TickType_t ) 1000 / portTICK_RATE_MS )
/* The rate at which the temperature is read. */
#define mainTEMP_DELAY			( ( TickType_t ) 100 / portTICK_RATE_MS )


typedef struct strConfig
{
	int8_t comand[4];         // 1-atualizar_hora 2-report_config 3-submeter_configuracao 4-configurar modo  5-configurar  7-ports 8-leds 9-frequencia// [comando, porto, valor ,aplicar]
	int8_t modo;			//1-continuo 2-condicional 3-sleep
	int8_t start_data;
	int8_t end_data;
	int8_t start_time;
	int8_t end_time;
	int8_t freq_datalog;	// frequencia em vezes por segundo
	int8_t sensores_now[10];  //
	int8_t sesores_cond[6];		//sensores que teram trubleshoot
	int8_t sensores_cond_value[6]; //volar do trubleshot so sensor da mesma posicao do vetor sensores_cond
	int8_t leds;			//0 - desligado 1 -ligado
	int8_t freq_leds;		//frequencia em vezes por dia
	int8_t time_on_off;		//max .5 e significa q passa tanto tempo ligado como desligado
	char port1[10];
	char port2[10];
	char port3[10];
	char port4[10];
	char port5[10];
	int armazenar;   // 1-enviar dados pela uart destina 	2 - armazenar no sd card
	char config[200];
}strConfig;

uint16_t ADC1_XYZ[3];
int Gx,Gy,Gz;

typedef struct temptime
{
	int32_t temp;
	int32_t time;
}temptime_t;

char RxData;

/* Configure RCC clocks */
//static void prvSetupRCC( void );

static void prvSetupRCCHSI( void );

/* Configure GPIO. */
static void prvSetupGPIO( void );

/* Configure the ADC */
//static void prvSetupADC( void );
void DMA_Config(void);
void ADC_Config(void);
void read_acel(void);

/* Simple LED toggle task. */
static void prvFlashTask1( void *pvParameters );

/* LCD activity task. */
static void prvLcdTask( void *pvParameters );

/* ADC temperature read task. */
static void prvTempTask( void *pvParameters );

static void prvSetupUSART_INT( void );

static void prvSetupEXTI1( void );

static void prvTickTask( void *pvParameters);

static void prvSendTemp( void *pvParameters);


/********** Useful functions **********/
/* USART2 configuration. */
static void prvSetupUSART2( void );

/* USART2 send message. */
static void prvSendMessageUSART2(char *message);

/***************************************/


/* Task 1 handle variable. */
TaskHandle_t HandleTask1;

/* Task 2 handle variable. */
TaskHandle_t HandleTask2;

/* Task 3 handle variable. */
TaskHandle_t HandleTask3, HandleTask4, HandleTask5;

QueueHandle_t xQueue,xTickQueue,xTemptimeQueue; /* Global variable. */



int main( void )
{
	/*Setup the hardware, RCC, GPIO, etc...*/
    //prvSetupRCC();
    prvSetupRCCHSI();
	prvSetupUSART2();
    prvSetupUSART_INT();
    prvSetupEXTI1();
    prvSetupGPIO();
    //prvSetupADC();
    DMA_Config();
    ADC_Config();

    xQueue = xQueueCreate( 10, sizeof( char ) );

    if( xQueue == 0 )
    {
    /* Queue was not created and must not be used. */
    }
    else
    {
    /* Queue created successfully. */
    }

    xTickQueue = xQueueCreate( 10, sizeof( TickType_t ) );

    if( xTickQueue == 0 )
	{
	/* Queue was not created and must not be used. */
	}
	else
	{
	/* Queue created successfully. */
	}

    xTemptimeQueue = xQueueCreate( 10, sizeof( temptime_t ) );
    if( xTemptimeQueue == 0 )
	{
	/* Queue was not created and must not be used. */
	}
	else
	{
	/* Queue created successfully. */
	}

	/* Create the tasks */
 	//xTaskCreate( prvLcdTask, "Lcd", configMINIMAL_STACK_SIZE, NULL, mainLCD_TASK_PRIORITY, &HandleTask1 );
 	xTaskCreate( prvFlashTask1, "Flash1", configMINIMAL_STACK_SIZE, NULL, mainFLASH_TASK_PRIORITY, &HandleTask2 );
    //xTaskCreate( prvTempTask, "Temp", configMINIMAL_STACK_SIZE+100, NULL, mainFLASH_TASK_PRIORITY, &HandleTask3 );
 	//xTaskCreate( prvTickTask, "TickTask", configMINIMAL_STACK_SIZE, NULL, mainFLASH_TASK_PRIORITY, &HandleTask4 );
 	xTaskCreate( prvSendTemp, "SendTemp", configMINIMAL_STACK_SIZE+200, NULL, mainFLASH_TASK_PRIORITY, &HandleTask5 );

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;
}
/*-----------------------------------------------------------*/
static void prvSendTemp( void *pvParameters)
{
	//temptime_t temp;
	//char buf[50],timebuf[50];
	//char str[6];
	//int i=0;

	strConfig MyConfig;
	char buf[100];
	char receveid;
	char str[50];
	int i=0;
	int k;
	int b;
	char comparar[50];
	MyConfig.comand[1]=0;MyConfig.comand[2]=0;MyConfig.comand[3]=0;MyConfig.comand[4]=0;



	for(;;)
	{
		//xQueueReceive(xTemptimeQueue, &temp, ( TickType_t ) portMAX_DELAY);

		//sprintf(buf, "Temperatura = %ld",temp.temp);
		//sprintf(timebuf, " || Tempo = %ld \r\n",(temp.time/100));  //taking out 01 to the temp value
		//lcd_draw_fillrect(85,10,28,50,0x0000);
		//lcd_draw_string(5,10, buf,0xFFFF,1);
		//strcat(buf,timebuf);
		//char c;



		//if( xQueue != 0){
			//char c;
			//xQueueReceive(xQueue, &c, ( TickType_t ) portMAX_DELAY);
			//if (i<=5){
			//	str[i]=c;
			//	i++;
			//}

			//prvSendMessageUSART2(str);
			//prvSendMessageUSART2('\n');

		//}
		//if (i>5){
			//i=0;

//		}

//	}
//}

			if( uxQueueMessagesWaiting(xQueue) > 0){
				xQueueReceive(xQueue, &receveid, ( TickType_t ) portMAX_DELAY);
				str[i]=receveid;


				i++;
				if (str[i-1]=='.'){
					sprintf(buf,'%d',i);


					k=0;
					switch(i){

						case 4:
							b=0;
							sprintf(comparar,"imu");
							for (k=0;k<3;k++){
								if (comparar[k]==str[k]){b++;}else{b=100;break;}
							}
							if(b==3){
								read_acel();
								sprintf(buf, "imu_reconhecido, %d,%i,x->%d mg,y->%d mg,z->%d mg \r\n",b,i,Gx,Gy,Gz);//ADC1_XYZ[0],ADC1_XYZ[1],ADC1_XYZ[2]
								prvSendMessageUSART2(buf);
								//enquanto a configurar ctd esta parado ou maior
							}
							break;

						case 7:
							b=0;
							sprintf(comparar,"config \r\n");
							for (k=0;k<6;k++){
								if (comparar[k]==str[k]){b++;}else{b=100;break;}
							}
							if(b==6){
								sprintf(buf, "config_reconhecido. \r\n");
								prvSendMessageUSART2(buf);
								MyConfig.comand[1]=5;MyConfig.comand[2]=0;MyConfig.comand[3]=0;MyConfig.comand[4]=0;		//enquanto a configurar ctd esta parado ou maior
							}
							break;

						case 12:
							b=0;
							sprintf(comparar,"update_time");
							for (k=0;k<11;k++){
								if (comparar[k]==str[k]){b++;}else{b=100;break;}
							}
							if(b==11){
								sprintf(buf, "update_time_reconhecido. \r\n");
								MyConfig.comand[1]=1;MyConfig.comand[2]=0;MyConfig.comand[3]=0;MyConfig.comand[4]=1;
								prvSendMessageUSART2(buf);
							}
							break;

						case 14:
								b=0;
								sprintf(comparar,"report_config \r\n");
								for (k=0;k<13;k++){
									if (comparar[k]==str[k]){b++;}else{b=100;break;}
								}
								if(b==13){
									sprintf(buf, "report_config_reconhecido. \r\n");
									MyConfig.comand[1]=2;MyConfig.comand[2]=0;MyConfig.comand[3]=0;MyConfig.comand[4]=1;
									prvSendMessageUSART2(buf);
								}
								break;

						default:
								break;
						}

					if (MyConfig.comand[1]==5){
						switch(i){

							case 15:
								b=0;
								sprintf(comparar,"continuos_mode \r\n");
								for (k=0;k<14;k++){
									if (comparar[k]==str[k]){b++;}else{b=100;break;}
								}
								if(b==14){
									sprintf(buf, "continuos_mode, %d,%i \r\n",b,i);
									prvSendMessageUSART2(buf);
									MyConfig.comand[1]=4;MyConfig.comand[2]=100;MyConfig.comand[3]=1;MyConfig.comand[4]=1;
									}
								break;

							case 17:
								b=0;
								sprintf(comparar,"conditional_mode \r\n");
								for (k=0;k<16;k++){
									if (comparar[k]==str[k]){b++;}else{b=100;break;}
								}
								if(b==16){
									sprintf(buf, "conditional_mode_reconhecido, %d,%i \r\n",b,i);
									prvSendMessageUSART2(buf);
									MyConfig.comand[4]=4;MyConfig.comand[2]=100;MyConfig.comand[3]=2;MyConfig.comand[4]=1;
								}
								break;

							case 11:
								b=0;
								sprintf(comparar,"sleep_mode \r\n");
								for (k=0;k<10;k++){
									if (comparar[k]==str[k]){b++;}else{b=100;break;}
								}
								if(b==10){
									sprintf(buf, "sleep_mode_reconhecido, %d,%i \r\n",b,i);
									prvSendMessageUSART2(buf);
									MyConfig.comand[4]=4;MyConfig.comand[2]=100;MyConfig.comand[3]=3;MyConfig.comand[4]=1;
								}
								break;

								case 6:
									b=0;
									sprintf(comparar,"port");
									for (k=0;k<4;k++){
										if (comparar[k]==str[k]){b++;}else{b=100;break;}
									}
									if(b==4){
										switch(str[4]){

											case '1':
												sprintf(buf, "port1_reconhecido, %d,%i \r\n",b,i);
												prvSendMessageUSART2(buf);
												MyConfig.comand[1]=7;MyConfig.comand[2]=1;MyConfig.comand[3]=0;MyConfig.comand[4]=0;
												break;

											case '2':
												sprintf(buf, "port2_reconhecido, %d,%i \r\n",b,i);
												prvSendMessageUSART2(buf);
												MyConfig.comand[1]=7;MyConfig.comand[2]=2;MyConfig.comand[3]=0;MyConfig.comand[4]=0;
												break;

											case '3':
												sprintf(buf, "port3_reconhecido, %d,%i ",b,i);
												prvSendMessageUSART2(buf);
												MyConfig.comand[1]=7;MyConfig.comand[2]=3;MyConfig.comand[3]=0;MyConfig.comand[4]=0;
												break;

											case '4':
												sprintf(buf, "port4_reconhecido, %d,%i \r\n",b,i);
												prvSendMessageUSART2(buf);
												MyConfig.comand[1]=7;MyConfig.comand[2]=4;MyConfig.comand[3]=0;MyConfig.comand[4]=0;
												break;

											case '5':
												sprintf(buf, "port5_reconhecido, %d,%i \r\n",b,i);
												prvSendMessageUSART2(buf);
												MyConfig.comand[1]=7;MyConfig.comand[2]=5;MyConfig.comand[3]=0;MyConfig.comand[4]=0;
												break;

											default:
												break;
										}
									}
									break;

								case 4:
									b=0;
									sprintf(comparar,"uvc");
									for (k=0;k<3;k++){
										if (comparar[k]==str[k]){b++;}else{b=100;break;}
									}
									if(b==3){
										sprintf(buf, "uvc_reconhecido, %d,%i \r\n",b,i);
										prvSendMessageUSART2(buf);
										MyConfig.comand[1]=8;MyConfig.comand[2]=0;MyConfig.comand[3]=0;MyConfig.comand[4]=0;
									}
									break;

								case 5:
									b=0;
									sprintf(comparar,"freq");
									for (k=0;k<4;k++){
										if (comparar[k]==str[k]){b++;}else{b=100;break;}
									}
									if(b==4){
										sprintf(buf, "freq_reconhecido, %d,%i \r\n",b,i);
										prvSendMessageUSART2(buf);
										MyConfig.comand[1]=9;MyConfig.comand[2]=0;MyConfig.comand[3]=0;MyConfig.comand[4]=0;
									}
									break;

								case 14:
									b=0;
									sprintf(comparar,"add_condition");
									for (k=0;k<13;k++){
										if (comparar[k]==str[k]){b++;}else{b=100;break;}
									}
									if(b==13){
										sprintf(buf, "add_condiction_reconhecido, %d,%i \r\n",b,i);
										prvSendMessageUSART2(buf);
										MyConfig.comand[1]=10;MyConfig.comand[2]=0;MyConfig.comand[3]=0;MyConfig.comand[4]=0;
									}
									b=0;
									sprintf(comparar,"del_condition");
									for (k=0;k<13;k++){
										if (comparar[k]==str[k]){b++;}else{b=100;break;}
									}
									if(b==13){
										sprintf(buf, "del_condition_reconhecido, %d,%i \r\n",b,i);
										prvSendMessageUSART2(buf);
										MyConfig.comand[1]=10;MyConfig.comand[2]=0;MyConfig.comand[3]=0;MyConfig.comand[4]=0;
									}
									b=0;
									sprintf(comparar,"submit_config");
									for (k=0;k<13;k++){
										if (comparar[k]==str[k]){b++;}else{b=100;break;}
									}
									if(b==13){
										sprintf(buf, "submit_config_reconhecido, %d,%i \r\n",b,i);
										prvSendMessageUSART2(buf);
										MyConfig.comand[3]=10;MyConfig.comand[2]=0;MyConfig.comand[3]=0;MyConfig.comand[4]=1;
									}
									break;

								 default:
									 break;
								}
							}
								if (MyConfig.comand[1]==7){

									switch(i){

										case 3:
											b=0;
											sprintf(comparar,"on");
											for (k=0;k<2;k++){
													if (comparar[k]==str[k]){b++;}else{b=100;break;}
											}
											if(b==2){
												sprintf(buf, "on_reconhecido, %d,%i ",b,i);
												prvSendMessageUSART2(buf);
												MyConfig.comand[1]=7;MyConfig.comand[3]=1;MyConfig.comand[4]=1;
											}
											break;

										case 4:
											b=0;
											sprintf(comparar,"off");
											for (k=0;k<3;k++){
													if (comparar[k]==str[k]){b++;}else{b=100;break;}
											}
											if(b==3){
												sprintf(buf, "off_reconhecido, %d,%i ",b,i);
												prvSendMessageUSART2(buf);
												MyConfig.comand[1]=7;MyConfig.comand[3]=0;MyConfig.comand[4]=1;
											}
											break;

										case 5:
											b=0;
											sprintf(comparar,"info");
											for (k=0;k<4;k++){
													if (comparar[k]==str[k]){b++;}else{b=100;break;}
											}
											if(b==4){
												sprintf(buf, "info_reconhecido, %d,%i ",b,i);
												prvSendMessageUSART2(buf);
												MyConfig.comand[1]=70;MyConfig.comand[4]=1;
											}
											break;

										default:
											break;
									}

							}

							i=0;
							str[0]='.';
						}
						if (i>49){
							i=0;
							sprintf(buf, "limite char por comando");
							prvSendMessageUSART2(buf);
							str[0]='.';
						}

		}
	}
}







static void prvTickTask( void *pvParameters)
{
	TickType_t tick, l_tick=0;
	uint8_t cnt=0;
	char click;
	for(;;)
	{
		xQueueReceive(xTickQueue, &tick, ( TickType_t ) 250);
		if(cnt==0)
			l_tick=tick;
		else
		{
			if( (tick - l_tick) > 250)
			{
				click='1';
				xQueueSendToBack(xQueue, &click,  ( TickType_t ) 0);
			}
			if((tick - l_tick) < 250 && (tick - l_tick) > 5)
			{
				click='2';
				xQueueSendToBack(xQueue, &click,  ( TickType_t ) 0);
			}
			l_tick=tick;
			cnt=1;
		}
		cnt++;
	}
}

static void prvFlashTask1( void *pvParameters )
{
    TickType_t xLastExecutionTime;

    xLastExecutionTime = xTaskGetTickCount();

    for( ;; )
	{
    	vTaskDelayUntil( &xLastExecutionTime, mainFLASH_DELAY );
		GPIO_WriteBit(GPIOB, GPIO_Pin_0, (1-GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_0)));
	}
}

/*-----------------------------------------------------------*/


/* Example task to present characteres in ther display. */
static void prvLcdTask( void *pvParameters )
{
	char c;
	lcd_init ( );

	for( ;; )
	{
		if( xQueue != 0)
			xQueueReceive(xQueue, &c, ( TickType_t ) portMAX_DELAY);
        lcd_draw_char( 63-(5*10)/2, 79-(7*10)/2, c, 0xFFFF, 10 );
        vTaskDelay( ( TickType_t ) 2000 / portTICK_RATE_MS);
	}
}
/*-----------------------------------------------------------*/


/* Temperature task - demo to read the ADC and get the temperature. */
/* Change this task accordingly. */
static void prvTempTask( void *pvParameters )
{
    TickType_t xLastExecutionTime;
    int ADC1ConvertedValue=0;

    temptime_t temptime;

    xLastExecutionTime = xTaskGetTickCount();
    for( ;; )
	{
    	vTaskDelayUntil ( &xLastExecutionTime, mainTEMP_DELAY);
        /* Read Sensor */
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        while( ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) != SET );

        ADC1ConvertedValue = ADC_GetConversionValue(ADC1);
        ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

        /*work with x 1000 to avoid float*/
        temptime.temp = (int32_t) ADC1ConvertedValue;
        temptime.temp = ( temptime.temp * 3300 ) / 4096; // finds mv
        temptime.temp = ((( 1400 - temptime.temp ) * 1000 ) / 430 ) + 250;

        temptime.time=xTaskGetTickCount();
        /*The temp variable has the temperature value times 10 */
		xQueueSendToBack(xTemptimeQueue, &temptime,  ( TickType_t ) 0);

    }
}
/*-----------------------------------------------------------*/



static void prvSetupRCC( void )
{
    /* RCC configuration - 72 MHz */
    ErrorStatus HSEStartUpStatus;

    RCC_DeInit();
    /*Enable the HSE*/
    RCC_HSEConfig(RCC_HSE_ON);
    /* Wait untill HSE is ready or time out */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    if(HSEStartUpStatus == SUCCESS)
    {
        /* Enable The Prefetch Buffer */
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        /* 72 MHZ - 2 wait states */
        FLASH_SetLatency(FLASH_Latency_2);

        /* No division HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        /* PCLK1 = HCLK/2 (36MHz) */
        RCC_PCLK1Config(RCC_HCLK_Div2);
        /* PCLK2 = HCLK (72MHz)*/
        RCC_PCLK2Config(RCC_HCLK_Div1);

        /* Use PLL with HSE=12MHz */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_6);
        /* Enable the PLL */
        RCC_PLLCmd(ENABLE);
        /* Wait for PLL ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET );

        /* Select the PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        /* Wait until PLL is used as system clock */
        while( RCC_GetSYSCLKSource() != 0x08 );
    }
    else
    {
        while(1);
    }
}
/*-----------------------------------------------------------*/




static void prvSetupRCCHSI( void )
{
	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	RCC_PCLK1Config(RCC_HCLK_Div1);
	RCC_PCLK2Config(RCC_HCLK_Div1);

	RCC_HSICmd(ENABLE);
	FLASH_SetLatency(FLASH_Latency_2);

	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);
	RCC_PLLCmd(ENABLE);
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	while(RCC_GetSYSCLKSource() != 0x08);
}
/*-----------------------------------------------------------*/



static void prvSetupGPIO( void )
{
    /* GPIO configuration - GREEN LED*/
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIOB clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //Bot\E3o SW5
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

}
/*-----------------------------------------------------------*/
static void prvSetupEXTI1( void )
{
    /*NVIC configuration*/
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the EXTI0 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /*Configure Key Button EXTI Line to generate an interrupt on falling edge*/
    EXTI_InitTypeDef EXTI_InitStructure;

    /*Connect Key Button EXTI Line to Key Button GPIO Pin*/
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);

    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}

static void prvSetupUSART_INT( void )
{
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Configura o Priority Group com 1 bit */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	/* Interrup\E7\E3o global do USART2 com prioridade 0 sub-prioridade 2 */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

void prvSetupUSART2( void )
{
USART_InitTypeDef USART_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;

    /* USART2 is configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - 1 Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled */

    /* Enable GPIOA clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE );

    /* USART Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    /* Configure the USART2 */
    USART_Init(USART2, &USART_InitStructure);
    /* Enable the USART2 */
    USART_Cmd(USART2, ENABLE);
 }

/*-----------------------------------------------------------*/



static void prvSendMessageUSART2(char *message)
{
uint16_t cont_aux=0;

    while(cont_aux != strlen(message))
    {
        USART_SendData(USART2, (uint8_t) message[cont_aux]);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
        {
        }
        cont_aux++;
    }
}
/*-----------------------------------------------------------*/


static void prvSetupADC( void )
{
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_16,1, ADC_SampleTime_239Cycles5);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

}
/*-----------------------------------------------------------*/

void DMA_Config(void)
{
		 //DMA_1
		 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
		 DMA_DeInit(DMA1_Channel1);

		 DMA_InitTypeDef DMA_InitStructure;
		 DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
		 DMA_InitStructure.DMA_MemoryBaseAddr =  (uint32_t)ADC1_XYZ;
		 DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
		 DMA_InitStructure.DMA_BufferSize = 3;
		 DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		 DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		 DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
		 DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
		 DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //Better for SCAN and CONTINOUS ADC modes
		 DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		 DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

		 DMA_Init(DMA1_Channel1, &DMA_InitStructure);
		 DMA_Cmd(DMA1_Channel1, ENABLE);
}
void ADC_Config(void){

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
		/* Initialize the ADC1 according to the ADC_InitStructure members */
		ADC_InitTypeDef ADC_InitStructure;
		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_InitStructure.ADC_ScanConvMode = ENABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfChannel = 3; /* xyz */
		ADC_Init(ADC1, &ADC_InitStructure);

		ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_7Cycles5); //Z
		ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_7Cycles5); //Y
		ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_7Cycles5); //X

		ADC_DMACmd(ADC1, ENABLE);
		ADC_Cmd(ADC1, ENABLE);

		/* Calibrar */
		ADC_ResetCalibration(ADC1);
		while(ADC_GetResetCalibrationStatus(ADC1));
		ADC_StartCalibration(ADC1);
		while(ADC_GetCalibrationStatus(ADC1));
		ADC_SoftwareStartConvCmd ( ADC1 , ENABLE );
}

void read_acel(void){
		while(DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET);
		DMA_ClearFlag(DMA1_FLAG_TC1);
		//while(ADC_GetFlagStatus(ADC_FLAG_STRT) == RESET);
		//ADC_ClearFlag(ADC_FLAG_STRT);
		Gx=((ADC1_XYZ[2]*(12.21/4095))-6.2)*1000;
		Gy=((ADC1_XYZ[1]*(12.21/4095))-6.2)*1000;
		Gz=-(((ADC1_XYZ[0]*(12.21/4095))-6.2))*1000;
}
