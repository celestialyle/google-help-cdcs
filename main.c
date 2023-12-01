/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "oled.h"
#include "utils.h"
#include "nbiot.h"
#include "sht3x.h"
#ifdef KY_005_IR_ENISSION
#include "irsnd.h"
#endif

#ifdef KY_022_IR_RECEIVER
#include "irmp.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern unsigned char key1, key2;
char acDevInfo[128] = {0}, acHexBuf[256] = {0}, acAtBuf[512] = {0}, acUserCmd[64] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	int val = 0;

	const char *pcVersion = "V1.0.0";
	float fTemp = 0.0, fHumi = 0.0;
	unsigned int atLen = 0;
	int iRet = -1;
#ifndef KE1_EXT_EXAMPLE
	unsigned short usLight = 0, usSound = 0, usVoltage = 0;
	int iUserCase = 0, tryCnt = 120, iSigVal = 0;
	char netFlag = 0, cmdLen = 0, acSigVal[3] = {0};
	unsigned int dLen = 0, timeout = 1000, upDevFreq = 0, upNetFreq = 0;
	int nbSP = 0, nbCCID = 0, nbSNR = 0;
	unsigned char nbECL = 0;
	CTMessage stCtMsg;
#else
	unsigned short usExtAdc1 = 0, usExtAdc2 = 0;
	char acOledStr[18] = {0};
#endif

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); //B
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //R
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2); //G

	Beep_Switch(1);
	HAL_Delay(500);
	Beep_Switch(0);

	UART_Enable_Receive_IT();// 使能串口接收中断，开始接收数�??????????????????????

	OLED_Init(); // 初始化OLDE

	//KE1_I2C_SHT31_Init();
	SHT3X_SetI2cAdr(0x44);
	OLED_DrawLogo(); // 显示bmp单色logo图片
	HAL_Delay(1500);

	OLED_ShowKE1(); // 显示 小熊座KE1
	OLED_ShowString(0, 3, (uint8_t *)pcVersion, 6);
	HAL_Delay(1000);
	printf("Hello! i am KE1 south demo %s\r\n",pcVersion);
	OLED_ShowString(0, 3, (uint8_t *)"Checking...", 8);
	do{
		HAL_Delay(1000);

		//KE1_I2C_SHT31(&fTemp, &fHumi);
		SHT3X_GetTempAndHumi(&fTemp, &fHumi, REPEATAB_HIGH, MODE_CLKSTRETCH, 6);// GPIO-I2C
		printf("T:%0.2f,H:%0.2f\r\n", fTemp, fHumi);

		KE1_Send_AT("AT\r\n"); atLen = sizeof(acAtBuf);
		iRet = KE1_Recv_AT(acAtBuf, &atLen, 1000);
		if(0 == fHumi){    OLED_ShowString(0, 3, (uint8_t *)"SHT3X read error", 8); HAL_Delay(1000); NVIC_SystemReset();}
		else if(0 == iRet){OLED_ShowString(0, 3, (uint8_t *)"NB AT error     ", 8); HAL_Delay(1000); NVIC_SystemReset();}
	}while(0 == fHumi || 0 == iRet);

//	if(0 == fHumi || 0 == iRet){
//		printf("I2C or AT error , reboot MCU\n");
//		NVIC_SystemReset();//无法读取I2C或�?�AT没有响应则MCU 重启
//	}

	OLED_Clear();

#if defined KY_005_IR_ENISSION
	printf("----- Init & Start KY_005_IR_ENISSION\r\n");
	IRMP_DATA irmp_sent_data;
	irsnd_init();
	KE1_Ext_IR_Start();
	printf("----- Init IR Done\r\n");
#elif defined KY_022_IR_RECEIVER
	printf("----- Init & Start KY_022_IR_RECEIVER\r\n");
	IRMP_DATA irmp_recv_data;
	HAL_TIM_Base_Start_IT(&htim15);
	irmp_init();
#endif


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		for(val = 1000; val > 0; val--){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, val); //R红色灯渐暗
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000-val); //B蓝色灯渐亮
		if(val <= 500)
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 500-val); //G绿色灯渐亮
		else
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0); //红色在val大于500熄灭
		HAL_Delay(5); }
		for(val = 0; val < 1000; val++){ //反之可实现呼吸效果
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, val); //R
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000-val); //B
		if(val <= 500)
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 500-val); //G
		else
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0); //G
		HAL_Delay(5); }

#ifdef KE1_EXT_EXAMPLE // test extend example
		/* KE1 EXTEND EXAMPLE BEGIN*/

	#if defined KY_022_IR_RECEIVER
		if (irmp_get_data (&irmp_recv_data)) {
		  printf("Name->[%s],Protocol->%02X, Address->%02X, Command->%02X, Flags->%02X\r\n",
				  irmp_protocol_names[irmp_recv_data.protocol],
				  irmp_recv_data.protocol,
				  irmp_recv_data.address,
				  irmp_recv_data.command,
				  irmp_recv_data.flags);
		  snprintf(acOledStr, sizeof(acOledStr), "IR-Cmd:%04X", irmp_recv_data.command);
		  OLED_ShowString(0, 2, (uint8_t *)acOledStr, strlen(acOledStr));memset(acOledStr, 0, sizeof(acOledStr));
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		}
	#else
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET){// 扩展例程：KY-018,
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		}else{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
		}
	#endif

	#if defined KY_005_IR_ENISSION
		if(1 == key1){
			key1 = 0;
			irmp_sent_data.protocol = IRMP_NEC_PROTOCOL;                             // use NEC protocol
			irmp_sent_data.address  = 0x00FC;                                        // set address to 0x00FF
			irmp_sent_data.command  = 0x0016;                                        // set command to 0x0001
			irmp_sent_data.flags    = 0;                                             // don't repeat frame

			irsnd_send_data (&irmp_sent_data, TRUE);                                 // send frame, wait for completion
			printf("Sent data\r\n");
		}
	#else
		#if 1
		KE1_Ext_PWM_Start(3700, 50);// test beep
		HAL_Delay(100);
		KE1_Ext_PWM_Stop();
		HAL_Delay(100);
		#else
		KE1_Ext_PWM_Breathing_LED(5);
		#endif
	#endif

		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);

		KE1_ADC_Extend_Get(&usExtAdc1, &usExtAdc2);// 扩展例程：KY-018,
		snprintf(acOledStr, sizeof(acOledStr), "PA5 AD:%04d", usExtAdc1);
		OLED_ShowString(0, 0, (uint8_t *)acOledStr, strlen(acOledStr));memset(acOledStr, 0, sizeof(acOledStr));
		snprintf(acOledStr, sizeof(acOledStr), "PA7 AD:%04d", usExtAdc2);
		OLED_ShowString(0, 2, (uint8_t *)acOledStr, strlen(acOledStr));memset(acOledStr, 0, sizeof(acOledStr));
		//printf("usExtAdc1:%d, usExtAdc2:%d\n", usExtAdc1, usExtAdc2);

		/* KE1 EXTEND EXAMPLE END*/

#else
		if(0 == netFlag){
			OLED_Show_Note(NULL, 1, tryCnt);
			if(0 != tryCnt) {
				tryCnt--;
			}else{
				OLED_Show_Note("reg err", 0, 0);
				if(NB_OK == nbiot_reboot(10)){
					iUserCase = NB_STEP_BUFF_CLEAR;tryCnt = 120;
				}else{
					NVIC_SystemReset();
				}
			}
		}else{
			if(0 < iSigVal && 32 > iSigVal){
				OLED_Show_Note(NULL, 2, iSigVal);
			}else if(99 == iSigVal){
				OLED_Show_Note("no net", 0, 0);
			}
		}

		//KE1_I2C_SHT31(&fTemp, &fHumi); /** 采集温湿�??????  **/
		SHT3X_GetTempAndHumi(&fTemp, &fHumi, REPEATAB_HIGH, MODE_CLKSTRETCH, 6);// GPIO-I2C
		KE1_ADC_Senser_Get(&usLight, &usSound, &usVoltage);/** 采集光强和噪�??????  **/
		if(0 < fHumi){
			OLED_ShowT_H(fTemp, fHumi);
		}

		if(1 == key1){
			key1 = 0;
			printf("key1 touch!\r\n");
		}
		if(1 == key2){
			key2 = 0;
			printf("key2 touch!\r\n");
		}
		timeout = 500;

		//printf("case %d - %d - %d - %d\r\n", iUserCase,timeout, upDevFreq, upNetFreq);
		switch(iUserCase){
			case NB_STEP_BUFF_CLEAR:
				KE1_Clear_AT_Buf();
				iUserCase = NB_STEP_CHECK_AT;
				continue;
			case NB_STEP_CHECK_AT:
				KE1_Send_AT("ATE1\r\n");/** �??????启模块AT命令回显功能 **/
				break;
			case NB_STEP_CHECK_REG:
				if(NB_OK == nbiot_check_reg(3)){/** �??????查模块是否已经联�?????? **/
					iUserCase = NB_STEP_UP_REG_INFO;
					netFlag = 1;
				}else{
					iUserCase = NB_STEP_STOP_MODULE;
				}
				break;
			case NB_STEP_STOP_MODULE:
				KE1_Send_AT("AT+CFUN=0\r\n");/** 关闭模块，设备模块参数前�??????要关闭模�??????  **/
				timeout = 10000;
				break;
			case NB_STEP_SET_COAP:
				KE1_Send_AT("AT+NCDP=180.101.147.115,5683\r\n");/** 设备电信物联网南向接口地�?????? **/
				break;
			case NB_STEP_START_MODULE:
				KE1_Send_AT("AT+CFUN=1\r\n");/** 启动通信模块**/
				timeout = 10000;
				break;
			case NB_STEP_SET_PDP:
				KE1_Send_AT("AT+CGDCONT=1,\"IP\",\"CTNB\"\r\n");/** 设置PDP **/
				break;
			case NB_STEP_SIM_CHECK:/** �??????查SIM是否存在 **/
				KE1_Send_AT("AT+CIMI\r\n");
				break;
			case NB_STEP_START_REG:
				KE1_Send_AT("AT+CGATT=1\r\n");/** 启动网络附着  **/
				break;
			case NB_STEP_SET_AUTO_REG:
				KE1_Send_AT("AT+QREGSWT=1\r\n");/** 设置网络自动注册 **/
				break;
			case NB_STEP_WAITING_REG_OK:
				HAL_Delay(1000);
				KE1_Send_AT("AT+CGATT?\r\n");/** �??????查网络是否联网成�?????? **/
				break;
			case NB_STEP_UP_REG_INFO:
				if(1 == netFlag && 0 == upNetFreq){
					nbiot_get_signl_val(acSigVal);iSigVal = atoi(acSigVal); printf("Signal:%d\r\n", iSigVal);

					tryCnt = 0;
					OLED_Show_UP_Flag(1);
					/*
					 ** 上报的无线参数�?�必须在数据范围内才算有效数据，数据范围要求
					 ** 1. 信号强度，上报范围应�?????? -140 ~ -40之间
					 ** 2. 覆盖等级，上报范围应�?????? 0~2之间
					 ** 3. 信噪比，上报范围应在 -20~30之间
					 **	4. 小区ID，上报范围应�?????? 0~2147483647之间
					 ** AT样例: AT+NMGS=11,03FFFFFFA608F651550E01
					 ** 平台JSON数据格式: {"SignalPower":-90,"CellID":150360405,"SNR":14,"ECL":1}
					 * */
					memset(acAtBuf, 0, sizeof(acAtBuf));
					nbiot_get_nuestats(&nbSP, &nbSNR, &nbCCID, &nbECL);

					printf("Signal:%d, %d, %d, %d\r\n", nbSP, nbCCID, nbSNR, nbECL);

					snprintf(acAtBuf, sizeof(acAtBuf), "AT+NMGS=14,03%08X%08X%08X%02X\r\n", nbSP, nbCCID, nbSNR, nbECL);// 打包模组信号强度参数
					KE1_Send_AT(acAtBuf);/** AT命令发�?�数�??????  **/
					upNetFreq = (60*60)*2;
				}

				break;
			case NB_STEP_UP_DEV_INFO:
				tryCnt++;
				if(10 == tryCnt || (1 == netFlag && 0 == upDevFreq)){
					tryCnt = 0;
					nbiot_get_signl_val(acSigVal);iSigVal = atoi(acSigVal); printf("Signal:%d\r\n", iSigVal);
				}

				if(1 == netFlag && 0 == upDevFreq){
					memset(acDevInfo, 0, sizeof(acDevInfo));
					memset(acAtBuf, 0, sizeof(acAtBuf));

					dLen = snprintf(acDevInfo, sizeof(acDevInfo), "{\"T\":\"%0.2f\",\"H\":\"%0.2f\",\"L\":\"%d\",\"S\":\"%d\",\"V\":\"%d\",\"NB\":\"%d\"}", fTemp, fHumi, usLight,usSound,usVoltage,iSigVal);// 打包设备传感器数�??????????????????????
					printf("%s\r\n", acDevInfo);
					if(0 < fHumi){
						OLED_Show_UP_Flag(1);
						vdAscii2hex(acDevInfo, acHexBuf);
						snprintf(acAtBuf, sizeof(acAtBuf), "AT+NMGS=%d,00%04X%s\r\n", (dLen+3), dLen, acHexBuf);// 打包COAP数据包AT命令
						//printf("%s\r\n", acAtBuf);
						KE1_Send_AT(acAtBuf);
					}
					upDevFreq = (60*60);
				}
				break;
		}

		atLen = sizeof(acAtBuf);
		iRet = KE1_Recv_AT(acAtBuf, &atLen, timeout);
		//printf("RAT:%d-%d\r\n",iRet, timeout);

		if(0 == iRet){//AT命令响应超时
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
		}else if(1 == iRet){//AT命令接收到OK响应
			if(NB_STEP_WAITING_REG_OK > iUserCase){
				iUserCase++;
				//HAL_Delay(1000);
			}
		}else if(2 == iRet){//AT命令接收到ERROR响应
			printf("AT error !\r\n");
			if(NB_STEP_START_MODULE == iUserCase || NB_STEP_SIM_CHECK == iUserCase){
				OLED_Show_Note("NO SIM", 0, 0);
			}
			if(NB_STEP_UP_REG_INFO == iUserCase){
				OLED_Show_Note("NO Reg", 0, 0);
			}
			do{ // 报警
				HAL_Delay(200);
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
			}while(1);

		}else if(3 == iRet){//接收到网络已注册提示
			printf("Net ready !\r\n");
			netFlag = 1;
			OLED_Show_Note("Net OK", 0, 0);
			HAL_Delay(3000);
			iUserCase = NB_STEP_UP_REG_INFO;
		}else if(4 == iRet){//AT命令接收到电信物联网平台下发数据
			printf("%s", acAtBuf);
			memset(acUserCmd, 0, sizeof(acUserCmd));
			cmdLen = nbiot_parse_NNMI(acAtBuf, acUserCmd);

			if(strstr(acUserCmd, "AAAA0000")){
				printf("device info upload successfully\r\n");
				OLED_Show_UP_Flag(0);
			}else if(strstr(acUserCmd, "CCCC0000")){
				printf("Module connectivity upload successfully\r\n");
				iUserCase = NB_STEP_UP_DEV_INFO;
				OLED_Show_UP_Flag(0);
			}else{

				printf("user data[%d]:%s\r\n", cmdLen, acUserCmd);
				memset(&stCtMsg, 0, sizeof(CTMessage));
				nbiot_parse_ct_msg(acUserCmd, &stCtMsg);
				printf("CMD[%02X]:%s\r\n", stCtMsg.userCmd, stCtMsg.userVal);
				/*
				 * 解析用户命令执行对应操作
				 * TO-DO
				 */

				if(CTL_LED == stCtMsg.userCmd){// Three color LED --- +NNMI:6,01008F010000
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
				}else if(CTL_LCD == stCtMsg.userCmd){// OLED --- +NNMI:12,01008D020006313233343536
					OLED_Clear();
					OLED_ShowString(0, 2, stCtMsg.userVal, 16);
				}else if(CTL_SWITCH_1 == stCtMsg.userCmd){// Relay 1 --- +NNMI:6,010090030000
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
				}else if(CTL_SWITCH_2 == stCtMsg.userCmd){// Relay 2 --- +NNMI:6,010091040000
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
				}
				/* 向平台发送命令响�????
				 */
				acUserCmd[1] = '2';
				acUserCmd[6] = '0';acUserCmd[7] = '0';acUserCmd[8] = '0';acUserCmd[9] = '0';acUserCmd[10] = 0;
				printf("CmdResp:%s\r\n", acUserCmd);
				snprintf(acAtBuf, sizeof(acAtBuf), "AT+NMGS=%d,%s\r\n", 5, acUserCmd);// 打包COAP数据包AT命令
				KE1_Send_AT(acAtBuf);
				if(CTL_LCD == stCtMsg.userCmd){
					HAL_Delay(3000);
				}
			}

		}else{
			if(-2 == iRet){// UART error
				NVIC_SystemReset();
			}
		}
		if(0 != upNetFreq) {
			upNetFreq--;
			if(0 == upNetFreq){iUserCase = NB_STEP_UP_REG_INFO;}
		}
		if(0 != upDevFreq) upDevFreq--;

#endif
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
