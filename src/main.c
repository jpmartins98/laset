
/**
  **************************************************************************
  * @file    main.c
  * @author  3S0 FreeRTOS nnd@isep.ipp.pt
  * @version V1.1
  * @date    28/11/2018
  * @brief   SISTR/SOTER FreeRTOS Example project
  **************************************************************************
*/


/*
 *
 * LED blink
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


#define main_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)
char RxData;
//bsd struct///////////////////////////////////////
typedef struct bsdconfig
{
	uint16_t C0,C1,C2,C3,C4,C5,C6,A0,A1,A2; 							//coeficietes
	uint8_t address,reset,CMD_PROM_RD, convert_d1,convert_d2, read_adc;
}bsdconfig;
bsdconfig bsd;
////////////////////////////////////////////////////
// configuração struct ////////////////////////////
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
//////////////////////////////////////////////////
///////////////////////////////////////////////////////////////comented struct and defines///////////////////////////////////////////////////////////
/* 	The rate at which the flash task toggles the LED. */
//#define mainFLASH_DELAY			( ( TickType_t ) 1000 / portTICK_RATE_MS )
/* 	The rate at which the temperature is read. */
//#define mainTEMP_DELAY			( ( TickType_t ) 100 / portTICK_RATE_MS )


//                 spi test defines
/*#define CMD_SET_READ_BIT 0x80
#define CMD_SET_WRITE_BIT 0x7F
#define REG_WHO_AM_I 0x0f //3A
#define ENABLE_ALL_AXES 0x87
#define REG_OFFSET_X 0x16
#define REG_OFFSET_Y 0x17
#define REG_OFFSET_Z 0x18
#define REG_GAIN_X 0x19
#define REG_GAIN_Y 0x1A
#define REG_GAIN_Z 0x1B
#define REG_CTRL_REG1 0x20
#define REG_CTRL_REG2 0x21
#define REG_CTRL_REG3 0x22
#define REG_HP_FILTER_RESET 0x23
#define REG_STATUS_REG 0x27
#define REG_OUTX_L 0x28
#define REG_OUTX_H 0x29
#define REG_OUTY_L 0x2a
#define REG_OUTY_H 0x2b
#define REG_OUTZ_L 0x2c
#define REG_OUTZ_H 0x2d
#define REG_FF_WU_CFG 0x30
#define REG_FF_WU_SRC 0x31
#define REG_FF_WU_ACK 0x32
#define REG_FF_WU_THS_L 0x34
#define REG_FF_WU_THS_H 0x35
#define REG_FF_WU_DURATION 0x36
#define REG_DD_CFG 0x38
#define REG_DD_SRC 0x39
#define REG_DD_ACK 0x3a
#define REG_DD_THSI_L 0x3c
#define REG_DD_THSI_H 0x3d
#define REG_DD_THSE_L 0x3e
#define REG_DD_THSE_H 0x3f
/*


/*I2C  config variables for our pcb
//#define I2Cx_RCC        RCC_APB1Periph_I2C1
//#define I2Cx            I2C1
//#define I2C_GPIO_RCC    RCC_APB2Periph_GPIOB
//#define I2C_GPIO        GPIOB
//#define I2C_PIN_SDA     GPIO_Pin_7
//#define I2C_PIN_SCL     GPIO_Pin_8
*/

/*
typedef struct temptime
{
	int32_t temp;
	int32_t time;
}temptime_t;

//adc imu
uint16_t ADC1_XYZ[3];
int Gx,Gy,Gz;

//spi imu
typedef struct tridimensional_value
{
	float x_value;
	float y_value;
	float z_value;

}tridimensional_value_t;
tridimensional_value_t acel_value;*/
///////////////////////////////////////////////////////////////end comented struct///////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////declare functions//////////////////////////////////////////////////////////////////

/********** tasks**********/
static void prvFlashTask1( void *pvParameters );
static void prvSendTemp( void *pvParameters);

/********** gpio init**********/
static void prvSetupGPIO( void );

/********** rtc e clock**********/
static void prvSetupRCC( void );		//72hz
static void prvSetupRCCHSI( void );  	//60hz
static void prvSetupRTC(void);			//rtc

/*********USART2 send message***********/
static void prvSetupUSART2( void );
static void prvSendMessageUSART2(char *message);
static void prvSetupUSART_INT( void );

/*********functions comandos***********/
void verificar_comando_port(char str[50]);
void verificar_comando(char comparar[50] ,char str[50], int tamanho_palavra, int comando[5]);

/*********functions i2c***********/
void i2c_init();
int i2c_received_int_8(uint8_t address);
uint16_t i2c_received_int_16(uint8_t address, int size);
uint32_t i2c_received_int_32(uint8_t address, int size);
void i2c_send(uint8_t address, uint8_t byte);
void i2c_end_com();

//i2c-bsd
void bsd_init( void );
uint16_t bsd_FourteenBitConversion( uint16_t Value);
uint16_t bsd_TenBitConvertion( uint16_t Value);
uint32_t bsd_d1_d2(int d);
unsigned long bsd_calc_temperature(uint32_t d2);
unsigned long bsd_calc_pressure(uint32_t d1,uint32_t d2);

//i2c imu
//void start_i2c_imu();
//void ler_condutividade();

/*********adc functions***********/
//void DMA_Config(void);
//void ADC_Config(void);
//void read_acel(void);

/*********spi functions***********/

//static void prvSetupSPI(void);
//uint8_t transfer_8b_SPI2_Master(uint8_t outByte)
//static void prvGetAcelValuesTask( )


/*********extern interrput 1***********/
//static void prvSetupEXTI1( void )
////////////////////////////////////////////////////////////end declare functions//////////////////////////////////////////////////////////////////




/* Task handle variable. */
TaskHandle_t HandleTask1;TaskHandle_t HandleTask2;
/* Queue handle variable. */
QueueHandle_t xQueue; /* Global variable. */

int main( void )
{
	/*Setup the hardware, RCC, GPIO, etc...*/
    //prvSetupRCC();
	bsd.address=0x77,bsd.reset=0x1E,bsd.CMD_PROM_RD = 0xA0, bsd.convert_d1=0x48,bsd.convert_d2=0x58, bsd.read_adc=0x00; //bsd comands
	Runing_Config.I2C_clock_speed=100000;
	Runing_Config.I2C_Slave_adress[1]=0x08;Runing_Config.I2C_Slave_adress[1]=0;Runing_Config.I2C_Slave_adress[2]=0;Runing_Config.I2C_Slave_adress[3]=0;
	Runing_Config.I2C_Master_adress=0x00;
	Runing_Config.modo=3;									//ctd em sleep
	Runing_Config.freq_datalog=1/60;						//quando em modo continuo ou condicional freq minuto a minuto
	Runing_Config.leds=0;									//led off
	Runing_Config.freq_leds=1/60/60/2; 						// quando led on freq uma vez de 2 em duas horas
	Runing_Config.time_on_off_leds=1/60;					//quando led on 1 min on por 60 min off
    prvSetupRCC();		//relogio 72mhz
    //prvSetupRCCHSI();  		//relogio do stm 64mhz
    prvSetupRTC();
	prvSetupUSART2();
    prvSetupUSART_INT();
    //prvSetupEXTI1();
    prvSetupGPIO();
    RTC_SetCounter(1);

    i2c_init();

    /* queue */
    xQueue = xQueueCreate( 100, sizeof( char ) );
    if( xQueue == 0 )  {    }
    else { }
	/* Create the tasks */
 	xTaskCreate( prvFlashTask1, "Flash1", configMINIMAL_STACK_SIZE+500, NULL, main_TASK_PRIORITY, &HandleTask1 );
 	xTaskCreate( prvSendTemp, "SendTemp", configMINIMAL_STACK_SIZE+200, NULL, main_TASK_PRIORITY, &HandleTask2 );

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;
}
/*-----------------------------------------------------------*/





static void prvFlashTask1( void *pvParameters )
{
	char buff[50];
	//size_t RTC_Counter;
    for( ;; )
	{
    	vTaskSuspend(NULL);
		/* Block 1 second. */
    	//vTaskDelayUntil( &xLastExecutionTime, 1000 / portTICK_RATE_MS ); // ciclo de 1 s em 1s
		vTaskDelay( ( TickType_t ) 1000 / portTICK_PERIOD_MS  );	// ciclo de 1 s + ciclo da task em 1s + ciclo da task
		//sprintf(buff,"Teste");
		//prvSendMessageUSART2(buff);
		RCC_ClocksTypeDef ClksFreq;
		RCC_GetClocksFreq(&ClksFreq);
		size_t RTC_Counter = RTC_GetCounter();
		sprintf(buff,"%zu",RTC_Counter);
		prvSendMessageUSART2(buff);


        /* Toggle the LED */
		//GPIO_WriteBit(GPIOB, GPIO_Pin_0, ( 1-GPIO_ReadOutputDataBit( GPIOB, GPIO_Pin_0 ) ) );


		//sprintf(buf,"ADCCLK_Frequency: %zu  ",ClksFreq.ADCCLK_Frequency);
		//prvSendMessageUSART2(buf);

		//sprintf(,"HCLK_Frequency: %zu  ",ClksFreq.HCLK_Frequency);
		//prvSendMessageUSART2(buf);

		//sprintf(buff,"PCLK1_Frequency: %zu  ",ClksFreq.PCLK1_Frequency);
		//prvSendMessageUSART2(buff);

		//sprintf(buf,"PCLK2_Frequency: %zu  ",ClksFreq.PCLK2_Frequency);
		//prvSendMessageUSART2(buf);

		//sprintf(buf,"SYSCLK_Frequency: %zu  ",ClksFreq.SYSCLK_Frequency);
		//prvSendMessageUSART2(buf);
	}
}





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
	int beta=0;
	MyConfig.comand[1]=0;MyConfig.comand[2]=0;MyConfig.comand[3]=0;MyConfig.comand[4]=0;
	sprintf(buf, "linha de comando");prvSendMessageUSART2(buf);
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
						if (MyConfig.comand[0]==110){
							if (beta==0){i2c_send(0x3A, 0x2A);i2c_end_com();}else{beta=1;}																//inicializar
							i2c_send(0x3A, 0x01);int valor = i2c_received_int_8(0x3A);i2c_end_com();if(valor>128){valor=valor-256;}valor=valor*1000/64;//obter valor x
							size_t RTC_Counter = RTC_GetCounter();
							sprintf(buf,"x -> %d mg, time -> %zu \r\n",valor,RTC_Counter);
							prvSendMessageUSART2(buf);
						}	//read_acel();     prvGetAcelValuesTask(); //ler_adc(0x3A, 0x01);
						/*if (beta==0){start_imu();}else{beta=1;}*/
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
							vTaskResume(HandleTask1);
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
/*-------------------------------comandos_functions----------------------------*/

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
///////////////////////////////////////____________________i2c_functions_____________////////////////////////


void i2c_send(uint8_t address, uint8_t byte){
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C2, ENABLE);
	while(!(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)));
	I2C_Send7bitAddress(I2C2, address, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2C2,byte);
	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

uint32_t i2c_received_int_32(uint8_t address, int size){
	//max size=4 considera-se o primeiro byte recebido como msb e o ultimo como lsb
	if (size==3 ||size==4){
		uint32_t i2C_response=0;
		uint32_t multiplicador=1;
		int k=0;
		I2C_GenerateSTART(I2C2,ENABLE);
		while(!(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)));
		I2C_Send7bitAddress(I2C2, address, I2C_Direction_Receiver);
		while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
		for (k=0;k<size;k++){
			while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
			i2C_response=I2C_ReceiveData(I2C2)*multiplicador+i2C_response;
			multiplicador=multiplicador*256;
		}
		return i2C_response;
	}
	return -1;
}


uint16_t i2c_received_int_16(uint8_t address, int size){
	//max size=4 considera-se o primeiro byte recebido como msb e o ultimo como lsb
	if (size==2||size==1){
		uint16_t i2C_response=0;
		uint16_t multiplicador=1;
		int k=0;
		I2C_GenerateSTART(I2C2,ENABLE);
		while(!(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)));
		I2C_Send7bitAddress(I2C2, address, I2C_Direction_Receiver);
		while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
		for (k=0;k<size;k++){
			while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
			i2C_response=I2C_ReceiveData(I2C2)*multiplicador+i2C_response;
			multiplicador=multiplicador*256;
		}
		return i2C_response;
	}
	return -1;
}

int i2c_received_int_8(uint8_t address){
	uint8_t i2C_response=0;
	I2C_GenerateSTART(I2C2,ENABLE);
	while(!(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)));
	I2C_Send7bitAddress(I2C2, address, I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
	i2C_response=I2C_ReceiveData(I2C2);
	return i2C_response;
}

void i2c_end_com(){
	I2C_AcknowledgeConfig(I2C2, DISABLE);
	I2C_GenerateSTOP(I2C2, ENABLE);
	while(I2C_GetFlagStatus(I2C2,I2C_FLAG_STOPF));
}
/////////////////////////////////////////////____________________________//////////////////////_____________--bsd_aproax misto entre--https://www.ccsinfo.com/forum/viewtopic.php?t=54676__ e https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=89BSD_Calculation_Method&DocType=SS&DocLang=EN
void bsd_init( void ){
	uint16_t coeficientes[9];
	int i;
	i2c_send(bsd.address, bsd.reset);	//reset//ele mete no adress 0x77 << 1
	i2c_end_com();
	vTaskDelay( ( TickType_t ) 3 / portTICK_PERIOD_MS  );//ele faz delay 3ms
	for(i=0;i<8;i++){
		i2c_send(bsd.address, bsd.CMD_PROM_RD + 2*i); 				//2*i porque os bit que estao a ser alterados sao os bits 234 menos significatios e n os 123
		coeficientes[i]=i2c_received_int_16(bsd.address,2);			//8 coeficientes
		void i2c_end_com();
	}
	//A explicação da necessidade de fazer este passo encontra-se na tabela  de mapa de moria q se encontra na pag 6  de https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=89BSD_Calculation_Method&DocType=SS&DocLang=EN	onde temos 10 volares codificados em 7 endereços de memoria
	bsd.C0 = (coeficientes[1] & 0xFFFC) >> 2; //ignora ultimos dois bit (considera 14) e swift o numero -2 bit
	bsd.C0 = bsd_FourteenBitConversion(bsd.C0);
	bsd.C1 = ((coeficientes[1] & 0x03) << 12) + ((coeficientes[2] & 0xFFF0) >> 4);  //considera os ultimos dois bits anterios mente ignorados e swifta os +12bit e do novo array ignora os ultimos 4 bit e swifta os restantes 12 -4bit
	bsd.C1 = bsd_FourteenBitConversion(bsd.C1);
	bsd.C2 = (((coeficientes[2] & 0xF) << 6) + (coeficientes[3] >> 10)) & 0x3FF;
	bsd.C2 = bsd_TenBitConvertion(bsd.C2);
	bsd.C3 = coeficientes[3] & 0x3FF;
	bsd.C3 = bsd_TenBitConvertion(bsd.C3);
	bsd.C4 = (coeficientes[4] >> 6) & 0x3FF;
	bsd.C4 = bsd_TenBitConvertion(bsd.C4);
	bsd.C5 =  (((coeficientes[4] & 0x3F) << 4) + (coeficientes[5] >> 12)) & 0x3FF;
	bsd.C5 = bsd_TenBitConvertion(bsd.C5);
	bsd.C6 = (coeficientes[5] >> 2) & 0x3FF;
	bsd.C6 = bsd_TenBitConvertion(bsd.C6);
	bsd.A0 = (((coeficientes[5] & 0x3) << 8) + (coeficientes[6] >> 8)) & 0x3FF;
	bsd.A0 = bsd_TenBitConvertion(bsd.A0);
	bsd.A1 = (((coeficientes[6] & 0xFF) << 2) + (coeficientes[7] >> 14)) & 0x3FF;
	bsd.A1 = bsd_TenBitConvertion(bsd.A1);
	bsd.A2 = (coeficientes[7] >> 4) & 0x3FF;
	bsd.A2 = bsd_TenBitConvertion(bsd.A2);
}



uint16_t bsd_FourteenBitConversion( uint16_t Value){
	uint16_t Converted = Value;
	if(Value > 8192){
		//Converted = ((Value - 8192 -1)^8191)*-1; //o q estava feito
		Converted=Value-16384;
	}
	return Converted;
}
//Convert from twos complement if required
uint16_t bsd_TenBitConvertion( uint16_t Value){
	uint16_t Converted = Value;
	if(Value > 512){
		//Converted = ((Value - 512 -1)^511)*-1; //o q estava feito
		Converted=Value-1024;
	}
	return Converted;
}

uint32_t bsd_d1_d2(int d){
	uint32_t d_val;
	//d=1-> d1		d=2	->d2
	if (d==1||d==2){
		i2c_send(bsd.address, 0x38+(0x10*d) );			// bsd convert_d1 ou convert_d2
		i2c_end_com();
		vTaskDelay( ( TickType_t ) 10 / portTICK_PERIOD_MS  );
		i2c_send(bsd.address, bsd.read_adc);
		d_val=i2c_received_int_32(bsd.address, 3);
		i2c_end_com();
		//if (d_val<0){d_val=d_val+16777216;}
		return d_val;
	}
	return -1;
}


unsigned long bsd_calc_pressure(uint32_t d1,uint32_t d2){
	unsigned long y,pressure,p;
	int Pmin=0,Pmax=28;
	int q[7];q[0]=9;q[1]=11;q[2]=9;q[3]=15;q[4]=15;q[5]=16;q[6]=16;
	y=(d1+bsd.C0*(2^q[0])+bsd.C3*(2^q[3])*(d2/16777216)+bsd.C4*(2^q[4])*((d2/16777216)^2))/(bsd.C1*(2^q[1])+bsd.C5*(2^q[5])*(d2/16777216)+bsd.C6*(2^q[6])*((d2/16777216)^2));
	p=y*(1-bsd.C2*(2^q[2])/16777216)+(bsd.C2*(2^q[2])/16777216)*(y^2);
	pressure=(p-0.1)/0.8*(Pmax-Pmin)+Pmin;
	return pressure;
}


unsigned long bsd_calc_temperature(uint32_t d2){
	unsigned long temperature;
	temperature=bsd.A0/3+bsd.A1*2*(d2/16777216)+bsd.A2*2*((d2/16777216)^2);
	return temperature;
}





//_______END_________///////////////////////////////////////////____________________________//////////////////////_____________--bsd_aproax--https://www.ccsinfo.com/forum/viewtopic.php?t=54676__

//////////////////other function////////////////////////////imu/spi/i2c/adc//////////////////////////////////////

//spi imu/////////////////
/*
uint8_t transfer_8b_SPI2_Master(uint8_t outByte)
{
    while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
    SPI_I2S_SendData(SPI2, outByte); // send
    while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
    return SPI_I2S_ReceiveData(SPI2); // read received
}


static void prvGetAcelValuesTask( )
{

	char buff[20];
	uint16_t x_value_raw, y_value_raw, z_value_raw;
	uint8_t l = 0, h = 0;

			GPIO_ResetBits(GPIOD, GPIO_Pin_2); // slave select (low)
			transfer_8b_SPI2_Master(CMD_SET_READ_BIT | REG_OUTX_L);
			l = transfer_8b_SPI2_Master(0xFF);
			GPIO_SetBits(GPIOD, GPIO_Pin_2); // slave deselect (high)

			GPIO_ResetBits(GPIOD, GPIO_Pin_2); // slave select (low)
			transfer_8b_SPI2_Master(CMD_SET_READ_BIT | REG_OUTX_H);
			h = transfer_8b_SPI2_Master(0xFF);
			GPIO_SetBits(GPIOD, GPIO_Pin_2); // slave deselect (high)

			x_value_raw = h << 8 | l;

			GPIO_ResetBits(GPIOD, GPIO_Pin_2); // slave select (low)
			transfer_8b_SPI2_Master(CMD_SET_READ_BIT | REG_OUTY_L);
			l = transfer_8b_SPI2_Master(0xFF); // value WHO_AM_I
			GPIO_SetBits(GPIOD, GPIO_Pin_2); // slave deselect (high)

			GPIO_ResetBits(GPIOD, GPIO_Pin_2); // slave select (low)
			transfer_8b_SPI2_Master(CMD_SET_READ_BIT | REG_OUTY_H);
			h = transfer_8b_SPI2_Master(0xFF); // dummy data
			GPIO_SetBits(GPIOD, GPIO_Pin_2); // slave deselect (high)

			y_value_raw = h << 8 | l;

			GPIO_ResetBits(GPIOD, GPIO_Pin_2); // slave select (low)
			transfer_8b_SPI2_Master(CMD_SET_READ_BIT | REG_OUTZ_L);
			l = transfer_8b_SPI2_Master(0xFF); // dummy data
			GPIO_SetBits(GPIOD, GPIO_Pin_2); // slave deselect (high)

			GPIO_ResetBits(GPIOD, GPIO_Pin_2); // slave select (low)
			transfer_8b_SPI2_Master(CMD_SET_READ_BIT | REG_OUTZ_H);
			h = transfer_8b_SPI2_Master(0xFF); // dummy data
			GPIO_SetBits(GPIOD, GPIO_Pin_2); // slave deselect (high)

			z_value_raw = h << 8 | l;

			//VALUE CONVERTION

			acel_value.x_value = x_value_raw * 0.0009765625;
			acel_value.y_value = y_value_raw * 0.0009765625;
			acel_value.z_value = z_value_raw * 0.0009765625;

			sprintf(buff, "x = %d\r\n", x_value_raw);
			prvSendMessageUSART2(buff);

			sprintf(buff, "y = %d\r\n", y_value_raw);
			prvSendMessageUSART2(buff);

			sprintf(buff, "z = %d\r\n", z_value_raw);
			prvSendMessageUSART2(buff);

}

*/
//end spi imu///////////////////

//adc imu

/*void read_acel(void){
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
}*/

//end adc imu

//start i2c imu/////////////////
/*void ler_adc(uint8_t address, uint8_t byte){
	int i2C_response;
	char buf[50];
	//send_data
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C2, ENABLE);
	while(!(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)));
	I2C_Send7bitAddress(I2C2, address, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2C2,byte);
	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	//received_data
	I2C_GenerateSTART(I2C2,ENABLE);
	while(!(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)));
	I2C_Send7bitAddress(I2C2, address, I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
	i2C_response=I2C_ReceiveData(I2C2);
	I2C_AcknowledgeConfig(I2C2, DISABLE);
	I2C_GenerateSTOP(I2C2, ENABLE);
	while(I2C_GetFlagStatus(I2C2,I2C_FLAG_STOPF));
	//tratamento de dados
	if (i2C_response>128){i2C_response=i2C_response-256;}
	i2C_response=i2C_response*1000/64;
	sprintf(buf, "%d mg",i2C_response);
	prvSendMessageUSART2(buf);
}*/


/*
void start_i2c_imu(){
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C2, ENABLE);
	while(!(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)));
	I2C_Send7bitAddress(I2C2, 0x3A, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2C2,0x2A);
	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_SendData(I2C2, 0x01);
	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTOP(I2C2, ENABLE);
	while(I2C_GetFlagStatus(I2C2,I2C_FLAG_STOPF));
}
*/
//end i2c imu
////////////////////////////////////////////////////////////////////////////////////

//Convert from twos complement if required
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

        /* No division HCLK = SYSCLK = 72 MHz*/
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        /* PCLK1 = HCLK/2 (36MHz) */
        RCC_PCLK1Config(RCC_HCLK_Div2);
        /* PCLK2 = HCLK (72MHz)*/
        RCC_PCLK2Config(RCC_HCLK_Div1);

        /* Use PLL with HSE = 12 MHz (12 MHz * 6 = 72 MHz) */
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
    	/* HSE error? No further action */
        while(1);
    }
}

//init i2c
void i2c_init()
{
    // Initialization struct
    I2C_InitTypeDef I2C_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;

    // Step 1: Initialize I2C
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);//I2Cx_RCC->RCC_APB1Periph_I2C2
    I2C_InitStruct.I2C_ClockSpeed = 800;	//adcmax=400
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 1;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C2, &I2C_InitStruct);//I2Cx->I2C2
    I2C_Cmd(I2C2, ENABLE);//I2Cx->I2C2

    // Step 2: Initialize GPIO as open drain alternate function
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//RCC_APB2Periph_GPIOB
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;//pb10/pb11
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//2MHz
    GPIO_Init(GPIOB, &GPIO_InitStruct);//gpiob
}
////////////////////////i2c init/////////////////////////////////////////
/*void i2c_init()
{
    // Initialization struct
    I2C_InitTypeDef I2C_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;

    // Step 1: Initialize I2C
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    I2C_InitStruct.I2C_ClockSpeed = Runing_Config.I2C_clock_speed;			//100000
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = Runing_Config.I2C_Master_adress;		//0x00
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
}*/

////////////////////////i2c init end/////////////////////////////////////////

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
    /* GPIO configuration */
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIOB clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE );

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

}
/*-----------------------------------------------------------*/



static void prvSetupUSART2( void )
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


static void prvSetupRTC(void)
{
	//
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	//
	PWR_BackupAccessCmd(ENABLE);
	//
	if ((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN)
	{
		//
		RCC_BackupResetCmd(ENABLE);
		RCC_BackupResetCmd(DISABLE);
		//
		RCC_LSEConfig(RCC_LSE_ON);
		while ((RCC->BDCR & RCC_BDCR_LSERDY) != RCC_BDCR_LSERDY) {}
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
		RTC_SetPrescaler(0x7FFF); //
		//
		RCC_RTCCLKCmd(ENABLE);
		//
		RTC_WaitForSynchro();
	}
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
/*-----------------------------------------------------------*/

/* This is a blocking send USART function */
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
///////////////////////////////////////////EXTI1///////////////////////////////////
/*static void prvSetupEXTI1( void )
{
    //NVIC configurationstatic void prvSetupUSART_INT( void )
    NVIC_InitTypeDef NVIC_InitStructure;

    // Enable the EXTI0 Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //Configure Key Button EXTI Line to generate an interrupt on falling edge
    EXTI_InitTypeDef EXTI_InitStructure;

    //Connect Key Button EXTI Line to Key Button GPIO Pin
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);

    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}*/


///////////////////////////////////////////////////////////adc//////////////////////////////////////////////////
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
/*void DMA_Config(void)
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
		// Initialize the ADC1 according to the ADC_InitStructure members
		ADC_InitTypeDef ADC_InitStructure;
		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_InitStructure.ADC_ScanConvMode = ENABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfChannel = 3; // xyz
		ADC_Init(ADC1, &ADC_InitStructure);

		ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_7Cycles5); //Z
		ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_7Cycles5); //Y
		ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_7Cycles5); //X

		ADC_DMACmd(ADC1, ENABLE);
		ADC_Cmd(ADC1, ENABLE);

		// Calibrar
		ADC_ResetCalibration(ADC1);
		while(ADC_GetResetCalibrationStatus(ADC1));
		ADC_StartCalibration(ADC1);
		while(ADC_GetCalibrationStatus(ADC1));
		ADC_SoftwareStartConvCmd ( ADC1 , ENABLE );
}*/
//////////////////////////////////////////////////////////end adc /////////////////////////////////////////////////////
//////////////////////////////////////////////////////////spi /////////////////////////////////////////////////////
/*
static void prvSetupSPI(void)
{

	uint8_t numRead = 0;

	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE);

    // GPIOB (PB13)SPC clock
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // GPIOB (PB14)MISO serial data output
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // GPIOB (PB15)MOSI serial data input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

	//GPIOD (PD2)SPI CS chip select
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	GPIO_ResetBits(GPIOD, GPIO_Pin_2); // slave select (low)
	transfer_8b_SPI2_Master(CMD_SET_READ_BIT | REG_WHO_AM_I);
	numRead = transfer_8b_SPI2_Master(0xFF); // value WHO_AM_I
	GPIO_SetBits(GPIOD, GPIO_Pin_2); // slave deselect (high)

	GPIO_ResetBits(GPIOD, GPIO_Pin_2); // slave select (low)
	transfer_8b_SPI2_Master(CMD_SET_WRITE_BIT & REG_CTRL_REG1);
	transfer_8b_SPI2_Master(0x87); // set accel output x,y,z
	GPIO_SetBits(GPIOD, GPIO_Pin_2); // slave deselect (high)

}
*/
//////////////////////////////////////////////////////////end spi /////////////////////////////////////////////////////
//////////////////////////////////////////////////////////coindutividade i2c /////////////////////////////////////////////////////
/*
void ler_condutividade()
{
	char c[10];
	int i;
	I2C_AcknowlegeConfig(I2C2,ENABLE);
	I2C_GenerateSTART(I2C2,ENABLE);
	 //Wait for I2C EV5.	// It means that the start condition has been correctly released	// on the I2C bus (the bus is free, no other devices is communicating))
	while(!(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT)));
	// Send slave address
	I2C_Send7bitAddress(I2C2,0x1C,I2C_Direction_Transmitter);
	delay(1);
	if(I2C_checkEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED )==1){
		I2C_SendData(I2C2,'R');
		while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTING));
		I2C_GetFlagStatus(I2C2,ENABLE);
		delay(900);
		I2C_GenerateSTART(I2C2,ENABLE);
		while(!(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT)));
		I2C_Send7bitAddress(I2C2,0x1C,I2C_Direction_Receiver);//I2C_Direction_Receiver
		while(!(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)));
		while(!(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED)));
		for (i=0;i<=10;i++){
			while(!(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED)));
			c[i]=(char*)I2C_ReceivedData(I2C2);
		}
		I2C_AcknowledgeConfig(I2C2,DISABLE);
		I2C_GenerateSTOP(I2C2,ENABLE);
		while(I2C_GetFlagStatus(I2C2,I2C_FLAG_STOPF));
	}
	else{
		c[0]='x';
	}
}
*/
//////////////////////////////////////////////////////////coindutividade i2c end /////////////////////////////////////////////////////



//////////////////////////////////////////////////////////exemplo task /////////////////////////////////////////////////////
/*static void prvTickTask( void *pvParameters)
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
*/

/*
static void prvFlashTask1( void *pvParameters )
{
    TickType_t xLastExecutionTime;

    xLastExecutionTime = xTaskGetTickCount();

    for( ;; )
	{
    	vTaskDelayUntil( &xLastExecutionTime, mainFLASH_DELAY );
		GPIO_WriteBit(GPIOB, GPIO_Pin_0, (1-GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_0)));
	}
}*/

/*-----------------------------------------------------------*/


/* Example task to present characteres in ther display. */
/*static void prvLcdTask( void *pvParameters )
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
}*/
/*-----------------------------------------------------------*/


/* Temperature task - demo to read the ADC and get the temperature. */
/* Change this task accordingly. */
/*static void prvTempTask( void *pvParameters )
{
    TickType_t xLastExecutionTime;
    int ADC1ConvertedValue=0;

    temptime_t temptime;

    xLastExecutionTime = xTaskGetTickCount();
    for( ;; )
	{
    	vTaskDelayUntil ( &xLastExecutionTime, mainTEMP_DELAY);
        // Read Sensor
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        while( ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) != SET );

        ADC1ConvertedValue = ADC_GetConversionValue(ADC1);
        ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

        //work with x 1000 to avoid float
        temptime.temp = (int32_t) ADC1ConvertedValue;
        temptime.temp = ( temptime.temp * 3300 ) / 4096; 						// finds mv
        temptime.temp = ((( 1400 - temptime.temp ) * 1000 ) / 430 ) + 250;

        temptime.time=xTaskGetTickCount();
        //The temp variable has the temperature value times 10
		xQueueSendToBack(xTemptimeQueue, &temptime,  ( TickType_t ) 0);

    }
}*/
