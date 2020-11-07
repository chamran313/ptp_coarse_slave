
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2020 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "lwip.h"

/* USER CODE BEGIN Includes */
#include "udp.h"
#include "ptp.c"


#define LWIP_PTP
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define ptp_port	1234

ip_addr_t local_ip;
ip_addr_t remote_ip;
extern struct netif gnetif;
//struct udp_pcb *udpc;
char tx_buf[12];
uint16_t udp_rcv_cnt;
int32_t  bf_sytl, bf_syth, af_sytl, af_syth;
uint8_t i=0;
uint8_t synq_state;
int32_t rcv_buf[3], ptphdr;
//extern struct ETH_TimeStamp tx_ts;
extern ETH_TimeStamp tx_ts;
extern ETH_TimeStamp rx_ts;
ETH_TimeStamp t1,t2,t3,t4, offset, pr_delay, bw1,bw2, bw3, bw4,bw11,bw22, target_time;

uint16_t udp_320_cnt, udpn320_cnt, port_copy, s3cnt;
uint32_t etpttssr;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void ptp_start(void);
void udp_receive_callback(void *arg, struct udp_pcb *udpc, struct pbuf *p, const ip_addr_t *addr, u16_t port);
void ip_asign(void);
void udp_client_init(void);
void udp_slave_init(void);
void udp_send1(void);
void load_target_time(ETH_TimeStamp *tg_time);
//ETH_TimeStamp  minus_calc(ETH_TimeStamp* t1, ETH_TimeStamp* t2 );
ETH_TimeStamp  minus_plus_calc(ETH_TimeStamp* t1, ETH_TimeStamp* t2, uint8_t opr );
ETH_TimeStamp nsec_minus(ETH_TimeStamp* t);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if(htim->Instance == TIM2)
	{
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		etpttssr = ETH->PTPTSSR;
	}

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM2_Init();
  MX_LWIP_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	ptp_start();
	ip_asign();
  udp_slave_init();
	
  //HAL_TIM_Base_Start_IT(&htim2);
	target_time.TimeStampHigh = 30;
	target_time.TimeStampLow = 0;
	load_target_time(&target_time);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
   MX_LWIP_Process();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIMEx_RemapConfig(&htim2, TIM_TIM2_ETH_PTP) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB14 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PG2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void ptp_start(void)
{
	
	//assert_param(IS_ETH_PTP_UPDATE(UpdateMethod));
	ETH->MACIMR |=  0x200;  // disable timestamp trigger interrupt generation
  ETH-> PTPTSCR |= 0x00000001;   // enable ptp time stamping
	ETH-> PTPSSIR = 10;     // sub_second increment register  0XA:FOR 96MHZ
													// : 5 FOR 216MHZ
	
	ETH->PTPTSCR |= 0x00000200; // sub_second reg overflow when recieve 999 999 999
	
	//ETH->PTPTSCR |= 0x00000c00;
  ETH->PTPTSCR |= 0x00000100; //set TSSARFE bit -> timestamp enable for all rcv frames
	
	ETH->PTPTSHUR = 0;
	ETH->PTPTSLUR = 0;
	ETH-> PTPTSCR |= 0x00000004;  // set bit2 = tssti  time stamp system time initialize
	while(ETH->PTPTSCR & 0x4){};
	
	//Enable enhanced descriptor structure 
    ETH->DMABMR |= ETH_DMABMR_EDE;
		
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ip_asign(void)
{
local_ip = gnetif.ip_addr;
 ip4addr_aton("192.168.1.110", &remote_ip); // .100 for stm    .50 for pc 
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++

/*void udp_client_init(void)
{
	
	err_t err;
	
	 // Create a new UDP control block  
  udpc = udp_new();
  
  if (udpc!=NULL)
  {
    //assign destination IP address 
    //IP4_ADDR( &DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3 );
    
    // configure destination IP address and port
    err= udp_connect(udpc, &remote_ip, 320);
    
    if (err == ERR_OK)
    {
      // Set a receive callback for the upcb 
      udp_recv(udpc, udp_receive_callback, NULL);  
    }
		else udp_remove(udpc);
  }
}*/


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void udp_slave_init(void)
{
   struct udp_pcb *udpc;
   err_t err;
   
   /* Create a new UDP control block  */
   udpc = udp_new();
   
   if (udpc)
   {
     /* Bind the upcb to the UDP_PORT port */
     /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
      err = udp_bind(udpc, &local_ip, ptp_port);
      
      if(err == ERR_OK)
      {
        /* Set a receive callback for the upcb */
        udp_recv(udpc, udp_receive_callback, NULL);
      }
      else
      {
        udp_remove(udpc);
      }
   }
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*void udp_send1(void)
{
  struct pbuf *pb;
	uint16_t len;
	err_t err;
	
	//len = sprintf(buf, "a");
	buf[0] = i;
	buf[1] = 0x57;
	len = 2;
	
	pb = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
	err = pbuf_take(pb,buf,len);
	if(err == ERR_OK )
	 {
		udp_connect(udpc, &remote_ip, 1025+i);
		 
		udp_send(udpc, pb);
		//tx_tsl = heth.TxDesc->TimeStampLow;
		//pbuf_free(pb);
		 
		 udp_disconnect(udpc);
		 
		 pbuf_free(pb);
		 //udp_remove(udpc);
	 }
}*/
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void udp_receive_callback(void *arg, struct udp_pcb *udpc, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{ 
	err_t  err1;
	
	port_copy = port;
	udpn320_cnt++;
	pbuf_copy_partial( p, rcv_buf, p->len, 0); 
	
	if(port == ptp_port )  // port 319,320 : for ptp messages
		{
		udp_320_cnt++;
		synq_state++;
		//pbuf_copy_partial( p, rcv_buf, p->len, 0); 
		ptphdr = rcv_buf[2];
		if(synq_state!= rcv_buf[2])
			{
				tx_buf[0] = 0x66;
				synq_state = 0;
			
				err1 = pbuf_take(p, tx_buf, 1);
				if(err1== ERR_OK) 
						{
						//Connect to the remote client 
						udp_connect(udpc, addr, port);
						udp_send(udpc, p);
						}
				 // free the UDP connection, so we can accept new clients 
				//udp_disconnect(udpc);
			}
		else if(synq_state==1)	{t2 = rx_ts;}
		else if(synq_state==2)	
			{
			 //pbuf_copy_partial( p, rcv_buf, p->len, 0); 
			 t1.TimeStampHigh = rcv_buf[1];
			 t1.TimeStampLow  = rcv_buf[0];
			 tx_buf[0] = 0x12;
			 err1 = pbuf_take(p, tx_buf, 1);
			 if(err1== ERR_OK) 
					{
					//Connect to the remote client 
					udp_connect(udpc, addr, port);
					udp_send(udpc, p);
					}
			 /* free the UDP connection, so we can accept new clients */
			 //udp_disconnect(udpc);
			 //udp_send1();
			 t3 = tx_ts;
			}
		else if(synq_state==3)
			{
			 //pbuf_copy_partial( p, rcv_buf, p->len, 0); 
			 t4.TimeStampHigh = rcv_buf[1];
			 t4.TimeStampLow  = rcv_buf[0];
			 synq_state = 0;
				bw1 =  minus_plus_calc(&t2, &t1, minus );
				bw2 =  minus_plus_calc(&t4, &t3, minus );
				bw11 =  nsec_minus( &bw1 ); // if sec is neg, also apply neg in nsec 
				bw22 =  nsec_minus( &bw2 );
				
				bw4 =  minus_plus_calc( &bw22, &bw11, minus); 
				bf_syth = ETH->PTPTSHR;
				bf_sytl = ETH->PTPTSLR;
				
				offset.TimeStampHigh = bw4.TimeStampHigh /2;
				offset.TimeStampLow = bw4.TimeStampLow /2;
				
				ETH_PTPTime_UpdateOffset( offset.TimeStampHigh  , offset.TimeStampLow);
				
				bw3 = minus_plus_calc( &bw11, &bw22, plus);
				pr_delay.TimeStampHigh = bw3.TimeStampHigh / 2 ;
				pr_delay.TimeStampLow = bw3.TimeStampLow / 2;
				af_syth = ETH->PTPTSHR;
				af_sytl = ETH->PTPTSLR;
				//delay = ((t2-t1) + (t4-t3))/2;
				//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
				
				/*target_time.TimeStampHigh = ETH->PTPTSHR;
				target_time.TimeStampLow = (ETH->PTPTSLR) + 68000 ; //first 30000-- 40 * -  100 *
				load_target_time(&target_time);*/
				s3cnt++;
			}
		}
	
 
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
	
	//rxtsh = rx_ts.TimeStampHigh;
	//rxtsl = rx_ts.TimeStampLow;
	
	//rsyth = ETH->PTPTSHR;
	//rsytl = ETH->PTPTSLR;
	
  /* Free receive pbuf */
  pbuf_free(p);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
ETH_TimeStamp nsec_minus(ETH_TimeStamp* t)
{
  ETH_TimeStamp time;
	
	if(t->TimeStampHigh <0 )	{ t->TimeStampLow = -(t->TimeStampLow); }
	time.TimeStampHigh = t->TimeStampHigh;
	time.TimeStampLow  = t->TimeStampLow;
	return time;
	//return *t;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 ETH_TimeStamp  minus_plus_calc(ETH_TimeStamp* t1, ETH_TimeStamp* t2, uint8_t opr )
{
//int32_t  secdiff, nsecdiff;
 ETH_TimeStamp time;

	if(opr==1) // minus
		{
		 time.TimeStampHigh =  t1->TimeStampHigh - t2->TimeStampHigh;
		 time.TimeStampLow =   t1->TimeStampLow - t2->TimeStampLow;
		}
	else   // plus
		{
     time.TimeStampHigh =  t1->TimeStampHigh + t2->TimeStampHigh;
		 time.TimeStampLow =   t1->TimeStampLow + t2->TimeStampLow;
		}
	
	
	
	if( time.TimeStampHigh<0 && time.TimeStampLow>0 )
	{
	 time.TimeStampHigh +=1;
   time.TimeStampLow  = biliard - time.TimeStampLow;	
   if(time.TimeStampHigh==0) { time.TimeStampLow = -(time.TimeStampLow); }		
	}
	else if( time.TimeStampHigh>0 && time.TimeStampLow<0 ) 
	{
	 time.TimeStampHigh = time.TimeStampHigh - 1; 
	 time.TimeStampLow = time.TimeStampLow + biliard;
	}
	
	return time;
	
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void load_target_time(ETH_TimeStamp *tg_time)
{
	ETH->PTPTTHR = tg_time->TimeStampHigh;
	ETH->PTPTTLR = tg_time->TimeStampLow;
	
	ETH->MACIMR &= 0XFFFFFDFF; // unmask timestamp trigger interrupt
	
	ETH->PTPTSCR |= 0x00000010 ; //Time stamp interrupt trigger enable

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
