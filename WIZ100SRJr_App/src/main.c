/**
  ******************************************************************************
  * @file    main.c
  * @author  IOP Team
  * @version V1.0.0
  * @date    01-May-2015
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, WIZnet SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2015 WIZnet Co.,Ltd.</center></h2>
  ******************************************************************************
  */ 
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "W7500x_crg.h"
#include "W7500x_wztoe.h"
#include "W7500x_miim.h"
#include "W7500x_i2c.h"
#include "W7500x_adc.h"
#include "common.h"
#include "uartHandler.h"
#include "flashHandler.h"
#include "storageHandler.h"
#include "gpioHandler.h"
#include "timerHandler.h"
#include "tftp.h"
#include "ConfigData.h"
#include "ConfigMessage.h"
#include "extiHandler.h"
#include "DHCP/dhcp.h"
#include "DNS/dns.h"
#include "S2E.h"
#include "dhcp_cb.h"
#include "atcmd.h"

/* Private typedef -----------------------------------------------------------*/
UART_InitTypeDef UART_InitStructure;
I2C_ConfigStruct conf;

/* Private define ------------------------------------------------------------*/
#define __DEF_USED_MDIO__
#define __W7500P__ // for W7500P
#ifndef __W7500P__ // for W7500
	#define __DEF_USED_IC101AG__ //for W7500 Test main Board V001
#endif

///////////////////////////////////////
// Debugging Message Printout enable //
///////////////////////////////////////
#define _MAIN_DEBUG_
//#define F_APP_DHCP
//#define F_APP_DNS

//#define ENABLE_MQTT
//#define ENABLE_NTP
//#define ENABLE_TCPClient
//#define ENABLE_ADXL345
//#define ENABLE_LM92
//#define ENABLE_WCS6800

///////////////////////////
// Demo Firmware Version //
///////////////////////////
#define VER_H		1
#define VER_L		00

/* Private function prototypes -----------------------------------------------*/
void delay(__IO uint32_t milliseconds); //Notice: used ioLibray
void TimingDelay_Decrement(void);

/* Private variables ---------------------------------------------------------*/
/* Transmit and receive buffers */
static __IO uint32_t TimingDelay;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
uint8_t g_send_buf[WORK_BUF_SIZE];
uint8_t g_recv_buf[WORK_BUF_SIZE];

uint8_t run_dns = 1;
uint8_t op_mode;
uint8_t factory_flag = 0;

uint8_t socket_buf[2048];
uint8_t g_op_mode = NORMAL_MODE;

volatile uint8_t g_check_temp = 0;

#if defined(ENABLE_LM92)
uint8_t cmd[2];
uint8_t dat[2];
int tempcode;
float temp;
char b;
#endif

#if defined(ENABLE_ADXL345)
int readings[3] = {0,0,0};
float x,y,z;
uint8_t buf[6];
uint8_t reg;
#endif

#if defined(ENABLE_WCS6800)
int cvalue;
float cout;

int ADC_Read(ADC_CH num)
{
    ADC_ChannelSelect (num); ///< Select ADC channel to CH0
    ADC_Start(); ///< Start ADC
    while(ADC_IsEOC()); ///< Wait until End of Conversion
    return ((uint16_t)ADC_ReadData()); ///< read ADC Data
}
#endif

char str[256];

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main()
{
    //uint8_t tx_size[8] = { 2, 2, 2, 2, 2, 2, 2, 2 };
    //uint8_t rx_size[8] = { 2, 2, 2, 2, 2, 2, 2, 2 };
    //uint8_t mac_addr[6] = {0x00, 0x08, 0xDC, 0x11, 0x22, 0x33};
    //uint8_t src_addr[4] = {192, 168,  0,  80};
    //uint8_t gw_addr[4]  = {192, 168,  0,  1};
    //uint8_t sub_addr[4] = {255, 255, 255,  0};
    //uint8_t dns_server[4] = {8, 8, 8, 8};           // for Example domain name server
    //uint8_t tmp[8];
	//int ret;
#if defined(F_APP_DHCP) || defined(F_APP_DNS)
	S2E_Packet *value = get_S2E_Packet_pointer();
#endif
#if defined(F_APP_DNS)
	uint8_t dns_server_ip[4];
#endif

    /* External Clock */
    CRG_PLL_InputFrequencySelect(CRG_OCLK);

    /* Clock */
    *(volatile uint32_t *)(0x41001014) = 0x00060100; // 48MHz
    //*(volatile uint32_t *)(0x41001014) = 0x000C0200; // 48MHz
    //*(volatile uint32_t *)(0x41001014) = 0x00050200; // 20MHz, Default
    //*(volatile uint32_t *)(0x41001014) = 0x00040200; // 16MHz

    /* Set System init */
    SystemInit();

	//__enable_irq();

    /* UART2 Init */
    S_UART_Init(115200);

    /* UART Init */
    //UART_StructInit(&UART_InitStructure);
    //UART_Init(UART_DEBUG,&UART_InitStructure);

    /* SysTick_Config */
    SysTick_Config((GetSystemClock()/1000));

    /* Set WZ_100US Register */
    setTIC100US((GetSystemClock()/10000));
    //getTIC100US();	
    //printf(" GetSystemClock: %X, getTIC100US: %X, (%X) \r\n", 
    //      GetSystemClock, getTIC100US(), *(uint32_t *)TIC100US);        

	LED_Init(LED1);
	LED_Init(LED2);
	LED_Init(LED3);

	LED_Off(LED1);
	LED_Off(LED2);
	LED_Off(LED3);

	//BOOT_Pin_Init();
	Board_factory_Init();
	EXTI_Configuration();

#if defined(EEPROM_ENABLE)
    I2C1_Init();
#endif
    /* Configure I2C0 */
    conf.scl = I2C_PA_9;
    conf.sda = I2C_PA_10;
    I2C_Init(&conf);

    // ADC initialize
    ADC_Init();

#if defined(ENABLE_LM92)
	cmd[0] = 0x01;
	I2C_Write(&conf, 0x96, cmd, 1);
	I2C_Read(&conf, 0x97, dat, 1);
    b = dat[0];
    b |= 0x10;
    //b &= ~0x10;
    //printf("[DB]1 dat[0]:0x%X b:0x%X\r\n", dat[0], b);
    cmd[0] = 0x01;
    cmd[1] = b;
    I2C_Write(&conf, 0x96, cmd, 2);
    //cmd[0] = 0x01;
    //I2C_Write(&conf, 0x96, cmd, 1);
    //I2C_Read(&conf, 0x97, dat, 1);
    //printf("[DB]2 dat[0]:0x%X\r\n", dat[0]);
#endif

#if defined(ENABLE_ADXL345)
	cmd[0] = 0x2d;
	cmd[1] = 0x00;
	I2C_Write(&conf, 0xa6, cmd, 2);

	cmd[0] = 0x31;
	cmd[1] = 0x0b;
	I2C_Write(&conf, 0xa6, cmd, 2);

	cmd[0] = 0x31;
	cmd[1] = 0x0b;
	I2C_Write(&conf, 0xa6, cmd, 2);

	cmd[0] = 0x2c;
	I2C_Write(&conf, 0xa6, cmd, 1);
	I2C_Read(&conf, 0xa7, buf, 1);
	reg = buf[0];
	reg &= 0x10;
	reg |= 0x0f;
	cmd[0] = 0x2c;
	cmd[1] = reg;
	I2C_Write(&conf, 0xa6, cmd, 2);

	cmd[0] = 0x2d;
	cmd[1] = 0x08;
	I2C_Write(&conf, 0xa6, cmd, 2);
#endif

	/* Load Configure Information */
	load_S2E_Packet_from_storage();
	UART_Configuration();

	/* Check MAC Address */
	check_mac_address();

	Timer0_Configuration();

#ifdef _MAIN_DEBUG_
	uint8_t tmpstr[6] = {0,};

	ctlwizchip(CW_GET_ID,(void*)tmpstr);
    printf("\r\n============================================\r\n");
	printf(" WIZnet %s EVB Demo v%d.%.2d\r\n", tmpstr, VER_H, VER_L);
	printf("============================================\r\n");
	printf(" WIZ100SRJr Platform based S2EApp Example\r\n");
	printf("============================================\r\n");
#endif

#ifdef __DEF_USED_IC101AG__ //For using IC+101AG
    *(volatile uint32_t *)(0x41003068) = 0x64; //TXD0 - set PAD strengh and pull-up
    *(volatile uint32_t *)(0x4100306C) = 0x64; //TXD1 - set PAD strengh and pull-up
    *(volatile uint32_t *)(0x41003070) = 0x64; //TXD2 - set PAD strengh and pull-up
    *(volatile uint32_t *)(0x41003074) = 0x64; //TXD3 - set PAD strengh and pull-up
    *(volatile uint32_t *)(0x41003050) = 0x64; //TXE  - set PAD strengh and pull-up
#endif	
#ifdef __W7500P__
	*(volatile uint32_t *)(0x41003070) = 0x61;
	*(volatile uint32_t *)(0x41003054) = 0x61;
#endif

#ifdef __DEF_USED_MDIO__ 
    /* mdio Init */
    mdio_init(GPIOB, MDC, MDIO);
    
//    printf("%d \r\n", PHY_ADDR);
    /* PHY Link Check via gpio mdio */
    while( link() == 0x0)
    {
        printf(".");  
        delay(500);
    }
    printf("PHY is linked. \r\n");  
#else
    delay(1000);
    delay(1000);
#endif

	Mac_Conf();
#if defined(F_APP_DHCP)
	DHCP_init(SOCK_DHCP, g_send_buf);

	/* Initialize Network Information */
	if(value->options.dhcp_use) {		// DHCP
		uint32_t ret;
		uint8_t dhcp_retry = 0;

		//printf("Start DHCP...\r\n");
		while(1) {
			ret = DHCP_run();

			if(ret == DHCP_IP_LEASED)
				break;
			else if(ret == DHCP_FAILED)
				dhcp_retry++;

			if(dhcp_retry > 3) {
				Net_Conf();
				break;
			}
			do_udp_config(SOCK_CONFIG);
		}
	} else 								// Static
		Net_Conf();
#else
	Net_Conf();
#endif

#if defined(F_APP_DNS)
	DNS_init(SOCK_DNS, g_send_buf);
	if(value->options.dns_use) {
		uint8_t dns_retry = 0;

		memcpy(dns_server_ip, value->options.dns_server_ip, sizeof(dns_server_ip));

		while(1) {
			if(DNS_run(dns_server_ip, (uint8_t *)value->options.dns_domain_name, value->network_info[0].remote_ip) == 1)
				break;
			else
				dns_retry++;

			if(dns_retry > 3) {
				break;
			}

			do_udp_config(SOCK_CONFIG);

			if(value->options.dhcp_use)
				DHCP_run();
		}
	}
#endif

	display_Net_Info();

	atc_init(&rxring, &txring);

	op_mode = OP_DATA;
	while (1) {
		if(op_mode == OP_COMMAND) {			// Command Mode
			atc_run();
			sockwatch_run();
		} else {							// DATA Mode
			s2e_run(SOCK_DATA);
		}

		do_udp_config(SOCK_CONFIG);

		if (g_check_temp == 1)
		{
#if defined(ENABLE_LM92)
			cmd[0] = 0x00;
			I2C_Write(&conf, 0x96, cmd, 1);
			I2C_Read(&conf, 0x97, dat, 2);
			dat[0] = (uint8_t)dat[0];
			dat[1] = (uint8_t)dat[1];
			tempcode = ((dat[0]<<8)|dat[1]) >> 3;
			temp = tempcode * 0.0625;

			//printf("[DB] check temp : 0x%X 0x%X %d\r\n", dat[0], dat[1], tempcode);
			printf("[DB] check temp : %.2f\r\n", temp);
#endif

#if defined(ENABLE_ADXL345)
			cmd[0] = 0x32;
			I2C_Write(&conf, 0xa6, cmd, 1);
			I2C_Read(&conf, 0xa7, buf, 6);
		    readings[0] = (int16_t)(buf[1] << 8 | buf[0]);
		    readings[1] = (int16_t)(buf[3] << 8 | buf[2]);
		    readings[2] = (int16_t)(buf[5] << 8 | buf[4]);
	        x=(int16_t)readings[0];
	        x=x*4e-3;
	        //x=x*0.00376390;
	        y=(int16_t)readings[1];
	        y=y*4e-3;
	        //y=y*0.00376009;
	        z=(int16_t)readings[2];
	        z=z*4e-3;
	        //z=z*0.00349265;
			//printf("[DB] check buf : 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\r\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
			//printf("[DB] check readings : %d %d %d\r\n", readings[0], readings[1], readings[2]);
			printf("[DB] check accelerometer : %.3f %.3f %.3f\r\n", x, y, z);
#endif

#if defined(ENABLE_WCS6800)
	        cvalue = ADC_Read(ADC_CH0);
	        //cout = (((2048-cvalue)*3.3/0.055)/4095.0);
	        //printf("Current sensor reading: %3.2f \r\n", cout);
	        //printf("Current sensor reading: %d \r\n", cvalue);

	        float Sensitivity    = 0.055; // mV/A
	        float InternalVcc    = 3.3;
	        float ZeroCurrentVcc = InternalVcc / 2;
	        float SensedVoltage  = (cvalue * InternalVcc) / 4096;
	        float Difference     = SensedVoltage - ZeroCurrentVcc;
	        float SensedCurrent  = Difference / Sensitivity;
	        cout = SensedCurrent;

	        //printf("Sensitivity: %.3f \r\n", Sensitivity);
	        //printf("InternalVcc: %.1f \r\n", InternalVcc);
	        //printf("ZeroCurrentVcc: %.3f \r\n", ZeroCurrentVcc);
	        //printf("SensedVoltage: %.3f \r\n", SensedVoltage);
	        //printf("Difference: %.3f \r\n", Difference);
	        //printf("SensedCurrent: %.2f \r\n", SensedCurrent);
			printf("[DB] check current : %.2f\r\n", cout);
#endif

#if defined(ENABLE_TCPClient)
			sprintf(str, "%.2f, %.3f, %.3f, %.3f, %.2f\r\n", temp, x, y, z, cout);
			struct __network_info *network = (struct __network_info *)get_S2E_Packet_pointer()->network_info;
			if(network->state == net_connect)
			{
				RingBuffer_InsertMult(&rxring, &str, strlen(str));
			}
#endif

			g_check_temp = 0;
		}

#if defined(F_APP_DHCP)
		if(value->options.dhcp_use)
			DHCP_run();
#endif

#if defined(F_APP_DNS)
		if(value->options.dns_use && run_dns == 1) {
			memcpy(dns_server_ip, value->options.dns_server_ip, sizeof(dns_server_ip));

			if(DNS_run(dns_server_ip, (uint8_t *)value->options.dns_domain_name, value->network_info[0].remote_ip) == 1) {
				run_dns = 0;
			}
		}
#endif
	}

    return 0;
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void delay(__IO uint32_t milliseconds)
{
  TimingDelay = milliseconds;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}
