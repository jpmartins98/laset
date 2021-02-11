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

/*I2C*/
//#define I2Cx_RCC        RCC_APB1Periph_I2C1
//#define I2Cx            I2C1
//#define I2C_GPIO_RCC    RCC_APB2Periph_GPIOB
//#define I2C_GPIO        GPIOB
//#define I2C_PIN_SDA     GPIO_Pin_7
//#define I2C_PIN_SCL     GPIO_Pin_8


/* The rate at which the flash task toggles the LED. */
#define mainFLASH_DELAY			( ( TickType_t ) 1000 / portTICK_RATE_MS )
/* The rate at which the temperature is read. */
#define mainTEMP_DELAY			( ( TickType_t ) 100 / portTICK_RATE_MS )


typedef struct strConfig
{
	int8_t comand[5];         // 1-atualizar_hora 2-report_config 3-submeter_configuracao 4-configurar modo  5-configurar  7-ports 8-leds 9-frequencia// [comando, porto, valor ,aplicar]
	int8_t modo;			//1-continuo 2-condicional 3-sleep
	float freq_datalog;	// frequencia em vezes por segundo (Hz)
	int8_t start_data;
	int8_t end_data;
	int8_t start_time;
	int8_t end_time;
	int8_t sensores_now[10];  	//
	int8_t sesores_cond[6];		//sensores que teram trubleshoot
	int8_t sensores_cond_value[6]; //volar do trubleshot so sensor da mesma posicao do vetor sensores_cond
	int8_t leds;			//0 - desligado 1 -ligado
	int8_t freq_leds;		//frequencia em vezes por dia
	int8_t time_on_off_leds;		//max .5 e significa q passa tanto tempo ligado como desligado
	char port1[10];
	char port2[10];
	char port3[10];
	char port4[10];
	char port5[10];
	//int armazenar;   // 1-enviar dados pela uart destina 	2 - armazenar no sd card
	char config[200];
	int I2C_clock_speed;
	int I2C_Slave_adress[4];
	int I2C_Master_adress;
}strConfig;
strConfig MyConfig;
strConfig Runing_Config;

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
void verificar_comando(char comparar[50] ,char str[50], int tamanho_palavra, int comando[5]);
void verificar_comando_port(char str[50]);
void i2c_init(void);

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
	Runing_Config.I2C_clock_speed=100000;
	Runing_Config.I2C_Slave_adress[1]=0x08;Runing_Config.I2C_Slave_adress[1]=0;Runing_Config.I2C_Slave_adress[2]=0;Runing_Config.I2C_Slave_adress[3]=0;
	Runing_Config.I2C_Master_adress=0x00;
	Runing_Config.modo=3;									//ctd em sleep
	Runing_Config.freq_datalog=1/60;						//quando em modo continuo ou condicional freq minuto a minuto
	Runing_Config.leds=0;									//led off
	Runing_Config.freq_leds=1/60/60/2; 						// quando led on freq uma vez de 2 em duas horas
	Runing_Config.time_on_off_leds=1/60;					//quando led on 1 min on por 60 min off
    prvSetupRCCHSI();
	prvSetupUSART2();
    prvSetupUSART_INT();
    prvSetupEXTI1();
    prvSetupGPIO();
    //prvSetupADC();
    DMA_Config();
    ADC_Config();
    i2c_init();

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

	int comando[5];
	char buf[100];
	char receveid;
	char str[50];
	int i=0;
	MyConfig.comand[1]=0;MyConfig.comand[2]=0;MyConfig.comand[3]=0;MyConfig.comand[4]=0;



	for(;;)
	{
		if( uxQueueMessagesWaiting(xQueue) > 0){
			xQueueReceive(xQueue, &receveid, ( TickType_t ) portMAX_DELAY);
			str[i]=receveid;
			i++;
			if (str[i-1]=='\r'){ //menu_principal
				switch(i){
					case 4:{
						comando[0]=110; comando[1]=0; comando[2]=0; comando[3]=1;
						verificar_comando("imu" , str, 3, comando);
						if (MyConfig.comand[0]==110){read_acel();}
						break;
					}
					case 7:{
						comando[0]=5; comando[1]=0; comando[2]=0; comando[3]=0;
						verificar_comando("config" , str, 6, comando);
						break;
					}
					case 12:{
						comando[0]=1; comando[1]=0; comando[2]=0; comando[3]=0;
						verificar_comando("update_time" , str, 11, comando);
						break;
					}
					case 14:{
						comando[0]=2; comando[1]=0; comando[2]=0; comando[3]=0;
						verificar_comando("report_config" , str, 11, comando);
						break;
					}
					default:{break;}

				}

				if (MyConfig.comand[0]>4){			//menu_configuração
					switch(i){
						case 4:{
							comando[0]=21; comando[1]=0; comando[2]=0; comando[3]=0;
							verificar_comando("uvc" , str, 3, comando);
							break;}

						case 6:{
							verificar_comando_port(str); //port
							break;}
						case 11:{
							comando[0]=20; comando[1]=25; comando[2]=3; comando[3]=0;
							verificar_comando("sleep_mode" , str, 10, comando);
						break;}

						case 14:{
							comando[0]=3; comando[1]=0; comando[2]=0; comando[3]=1;
							verificar_comando("submit_config" , str, 13, comando);
							break;}

						case 15:{
							comando[0]=20; comando[1]=25; comando[2]=1; comando[3]=0;
							verificar_comando("continuos_mode" , str, 14, comando);
							break;}

						case 16:{
							comando[0]=22; comando[1]=100; comando[2]=0; comando[3]=0;
							verificar_comando("freq_amostr_all" , str, 15, comando);
							break;}

						case 17:{
							comando[0]=20; comando[1]=25; comando[2]=2; comando[3]=0;
							verificar_comando("conditional_mode" , str, 16, comando);
							break;}

						 default:{break;}
						}
				}

				if (MyConfig.comand[0]==7 || (MyConfig.comand[0]<74 && MyConfig.comand[0]>69 ) ){ //port menu
					switch(i){

						case 3:{
							comando[0]=70; comando[1]=MyConfig.comand[1]; comando[2]=1; comando[3]=1;
							verificar_comando("on" , str, 2, comando);
							break;}

						case 4:{
							comando[0]=70; comando[1]=MyConfig.comand[1]; comando[2]=0; comando[3]=1;
							verificar_comando("off" , str, 3, comando);
							break;}

						case 5:{
							comando[0]=71; comando[1]=MyConfig.comand[1]; comando[2]=0; comando[3]=1;
							verificar_comando("info" , str, 4, comando);
							comando[0]=72; comando[1]=MyConfig.comand[1]; comando[2]=0; comando[3]=0;
							verificar_comando("name" , str, 4, comando);
							break;}

						case 12:{
							comando[0]=73; comando[1]=MyConfig.comand[1]; comando[2]=0; comando[3]=0;
							verificar_comando("freq_amostr" , str, 11, comando);
							break;}

						default:{break;}
					}

				}
				if ((MyConfig.comand[0]==20 || (MyConfig.comand[0]<56 && MyConfig.comand[0]>49 )) && MyConfig.comand[2]==2){  //continuos mode menu
					switch(i){
						case 18:{
							comando[0]=50; comando[1]=0; comando[2]=MyConfig.comand[2]; comando[3]=0;
							verificar_comando("add_condition_max" , str, 17, comando);
							comando[0]=51; comando[1]=0; comando[2]=MyConfig.comand[2]; comando[3]=0;
							verificar_comando("add_condition_min" , str, 17, comando);
						break;}

						case 14:{
							comando[0]=52; comando[1]=0; comando[2]=MyConfig.comand[2]; comando[3]=0;
							verificar_comando("del_condition" , str, 13, comando);
						break;}

						case 11:{
								comando[0]=54; comando[1]=0; comando[2]=MyConfig.comand[2]; comando[3]=0;
								verificar_comando("start_time" , str, 10, comando);
								break;}

						case 9:{
									comando[0]=55; comando[1]=0; comando[2]=MyConfig.comand[2]; comando[3]=0;
									verificar_comando("end_time" , str, 8, comando);
									break;}

						case 7:{
							comando[0]=53; comando[1]=0; comando[2]=MyConfig.comand[2]; comando[3]=1;
							verificar_comando("report" , str, 6, comando);
							break;}

						default:{break;}
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
	#define abb NVIC_InitStructure
	NVIC_InitTypeDef abb;
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
		char buf[50];
		while(DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET);
		DMA_ClearFlag(DMA1_FLAG_TC1);
		//while(ADC_GetFlagStatus(ADC_FLAG_STRT) == RESET);
		//ADC_ClearFlag(ADC_FLAG_STRT);
		Gx=((ADC1_XYZ[2]*(12.21/4095))-6.2)*1000;
		Gy=((ADC1_XYZ[1]*(12.21/4095))-6.2)*1000;
		Gz=-(((ADC1_XYZ[0]*(12.21/4095))-6.2))*1000;
		sprintf(buf, "x-> %d mg, y-> %d mg,z-> %d mg \r\n",Gx,Gy,Gz);
		prvSendMessageUSART2(buf);
}


void verificar_comando(char comparar[50] ,char str[50], int tamanho_palavra, int comando[5]){
	char buf[50];
	int k=0;
	int b=0;
	for (k=0;k<tamanho_palavra;k++){
		if (comparar[k]==str[k]){b++;}else{b=100;break;}
	}
	if(b==tamanho_palavra){
		sprintf(buf, "%s.\r\n",comparar);
		prvSendMessageUSART2(buf);
		MyConfig.comand[0]=comando[0];MyConfig.comand[1]=comando[1];MyConfig.comand[2]=comando[2];MyConfig.comand[3]=comando[3];MyConfig.comand[4]=comando[4];
	}
}



void verificar_comando_port(char str[50]){
	char buf[50];
	int k=0;
	int b=0;
	char comparar[10];
	sprintf(comparar,"port");
	for (k=0;k<4;k++){
			if (comparar[k]==str[k]){b++;}else{b=100;break;}
		}
	if(b==4){
		switch(str[4]){
			case '1':
				sprintf(buf, "port1.\r\n");
				prvSendMessageUSART2(buf);
				MyConfig.comand[0]=7;MyConfig.comand[1]=1;MyConfig.comand[2]=0;MyConfig.comand[3]=0;
				break;

			case '2':
				sprintf(buf, "port2.\r\n");
				prvSendMessageUSART2(buf);
				MyConfig.comand[0]=7;MyConfig.comand[1]=2;MyConfig.comand[2]=0;MyConfig.comand[3]=0;
				break;

			case '3':
				sprintf(buf, "port3.\r\n");
				prvSendMessageUSART2(buf);
				MyConfig.comand[0]=7;MyConfig.comand[1]=3;MyConfig.comand[2]=0;MyConfig.comand[3]=0;
				break;

			case '4':
				sprintf(buf, "port4.\r\n");
				prvSendMessageUSART2(buf);
				MyConfig.comand[0]=7;MyConfig.comand[1]=4;MyConfig.comand[2]=0;MyConfig.comand[3]=0;
				break;

			case '5':
				sprintf(buf, "port5.\r\n");
				prvSendMessageUSART2(buf);
				MyConfig.comand[0]=7;MyConfig.comand[1]=5;MyConfig.comand[2]=0;MyConfig.comand[3]=0;
				break;

			default:
				break;
		}
		return;
	}
}


void i2c_init()
{
    // Initialization struct

    I2C_InitTypeDef I2C_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;

    // Step 1: Initialize I2C
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    I2C_InitStruct.I2C_ClockSpeed = Runing_Config.I2C_clock_speed;//100000
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = Runing_Config.I2C_Master_adress;//0x00
    I2C_InitStruct.I2C_Ack = I2C_Ack_Disable; //I2C_Ack_Enable; // disable acknowledge when reading (can be changed later on)
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStruct);
    I2C_Cmd(I2C1, ENABLE);
    // Step 2: Initialize GPIO as open drain alternate function
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
}
