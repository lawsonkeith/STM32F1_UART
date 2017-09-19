/**
  ******************************************************************************
  * @file    UART/UART_TwoBoards_ComIT/Src/main.c 
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    14-April-2017
  * @brief   This sample code shows how to use UART HAL API to transmit
  *          and receive a data buffer with a communication process based on
  *          IT transfer. 
  *          The communication is done using 2 Boards.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include "stm32f1xx_hal_gpio.h"
// KL - msg framing
#define HEADER_SOF = 0x12345678;
#define HEADER_SOF1  'a'
#define HEADER_SOF2  'b'
#define HEADER_SOF3  'c'
#define HEADER_SOF4  'd'

#define __GPIOC_CLK_ENABLE __HAL_RCC_GPIOC_CLK_ENABLE

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* UART handler declaration */
UART_HandleTypeDef UartHandle;
__IO ITStatus UartReady = RESET;
__IO uint32_t UserButtonStatus = 0;  /* set to 1 after User Button interrupt  */

/* Buffer used for transmission */
uint8_t aTxBuffer[100];
uint8_t aRxByte;
struct UARTFrame{ // 14 bytes
	//uint8_t		SOF1;
	//uint8_t		SOF2;
	//uint8_t		SOF3;
	//uint8_t		SOF4;
	uint8_t		ID;
	uint8_t		Msg[8];
	uint8_t		csum;
}UART_rx_frame,UART_tx_frame;
// 115200 / 10 / 14 = 1.2ms Message Tx / Rx time.

int lock_UART_rx_frame=0; // locks global struct - locked by IRQ  unlocked by main()
int lock_UART_tx_frame=0;  // locks global struct - locked by main   unlocked by IRQ
extern __IO uint32_t uwTick;
/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(int i);
void KL_GPIO_init();
/* Private functions ---------------------------------------------------------*/

int RxCANMsg(){
	return 1;

}
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	static uint32_t timer;
  /* STM32F103xB HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 64 MHz */
  SystemClock_Config();
  
  // debug msg
  strcpy((char*)UART_tx_frame.Msg," -HI!- ");


  /*##-1- Configure the UART peripheral ######################################*/
  UartHandle.Instance        = USARTx;
  UartHandle.Init.BaudRate   = 115200;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
  {
    Error_Handler(1);
  }  
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler(2);
  }
  
  //KL @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  KL_GPIO_init();
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,0);

  // Start comms Rx process.
  if(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)&aRxByte, 1) != HAL_OK)
   {
     Error_Handler(3);
   }

  do{
	  static int Count;

	  Count++;

	  // Get data from IB8000
	  if(lock_UART_rx_frame ){
		  // ConvertToCAN(&UART_rx_frame &CAN_frame);
		  lock_UART_rx_frame = 0; // unlock
		  // SendCAN(&CAN_frame);
		  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
	  }

	  // CAN MSG received
	  if(uwTick > timer /*CanRX()*/)
	  {
		  timer = uwTick + 100; // ms

		  Count=0;

		  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_14);
		  // lets pretend we received a CAN msg
		  // ConvertToUART(&UART_tx_frame &CAN_frame);
		  if(lock_UART_tx_frame==0){
			  lock_UART_tx_frame++;
			  if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)&UART_tx_frame, sizeof(UART_tx_frame))!= HAL_OK){
				  Error_Handler(4);
			  }
		  }else{
			  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_15);
		  	  //else lose data
		  }
	  }

  }while(1);
  //KL @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  
  
}


// Setup some debug LEDS
//
void KL_GPIO_init()
{
	/*KL Configure GPIO pin : PC13 */
	GPIO_InitTypeDef GPIO_InitStruct;
	__GPIOC_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = (GPIO_CRL_MODE0_1);
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = (GPIO_CRL_MODE0_1);
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = (GPIO_CRL_MODE0_1);
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 64000000
  *            HCLK(Hz)                       = 64000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = 16
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  
  /* Configure PLL ------------------------------------------------------*/
  /* PLL configuration: PLLCLK = (HSI / 2) * PLLMUL = (8 / 2) * 16 = 64 MHz */
  /* PREDIV1 configuration: PREDIV1CLK = PLLCLK / HSEPredivValue = 64 / 1 = 64 MHz */
  /* Enable HSI and activate PLL with HSi_DIV2 as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSI;
  oscinitstruct.HSEState        = RCC_HSE_OFF;
  oscinitstruct.LSEState        = RCC_LSE_OFF;
  oscinitstruct.HSIState        = RCC_HSI_ON;
  oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSI_DIV2;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  lock_UART_tx_frame=0;
  //HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
  
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	static uint8_t	ptr=0;
	static uint8_t  buff[50];
	static int state=0;
	static uint8_t	csum=0;

	//Rx another
	//TODO: statement rather than function call?
	//HAL_UART_Receive_IT(UartHandle, (uint8_t *)&aRxByte, 1) ;
	//Can be removed if we hack the API see hal_uart.c line2438

	switch(state)
	{
		case 0:	if(aRxByte == HEADER_SOF1){
					state++;
					csum = 0;
					ptr=0;
				}else
					state = 0;
				break;
		case 1:	if(aRxByte == HEADER_SOF2){
					state++;
				}else
					state = 0;
				break;
		case 2:	if(aRxByte == HEADER_SOF3){
					state++;
				}
				else
					state = 0;
				break;
		case 3:	if(aRxByte == HEADER_SOF4){
					state++;
				}
				else
					state = 0;
				break;
		default:// get 14-4=10abcd=========9 bytes of data
				buff[ptr++] = aRxByte;

	}

	// checksum codeblock - process msg
	if(ptr==10)
	{
		// all 14 bytes received
		if(aRxByte == csum){
			if(lock_UART_rx_frame==0){
				lock_UART_rx_frame=1;
				memcpy((uint8_t *)&UART_rx_frame,buff,sizeof(UART_rx_frame));
			}else
				HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_15);
				//else lose data
		}
		state = 0;

	}else
		// 4 bytes SYNC + ID + 8 bytes = 13 bytes csum data
		// e.g "abcd=========" == '9'
		csum ^= aRxByte;
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler(6);
}


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == USER_BUTTON_PIN)
  {  
    UserButtonStatus = 1;
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(int Num)
{
	// KL
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_15);
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
