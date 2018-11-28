/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "usart.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
uint8_t UartRec[1];

uint32_t USART_RX_CNT = 0;
uint8_t APP_DATA[APP_DATA_LEN] __attribute__ ((at(0x20004000)));


/* USER CODE END 0 */

UART_HandleTypeDef huart2;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = USART2_TX_Pin|USART2_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, USART2_TX_Pin|USART2_RX_Pin);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */





//void STMFLASH_Write_NoCheck ( uint32_t WriteAddr, uint16_t * pBuffer, uint16_t NumToWrite )
//{
//    uint16_t i;
//
//    for (i=0; i<NumToWrite; i++) 
//    {
//        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, WriteAddr, pBuffer[i]);
//        WriteAddr+=2; //地址增加 2.
//    }
//}
//
//
//void STMFLASH_Write ( uint32_t WriteAddr, uint16_t * pBuffer, uint16_t NumToWrite )
//{
//#define STM32_FLASH_SIZE    32
//#define STM_SECTOR_SIZE     1024
//    
//    uint16_t secoff; //扇区内偏移地址(16 位字计算)
//    uint16_t secremain; //扇区内剩余地址(16 位字计算)
//    uint16_t i;
//    uint32_t secpos; //扇区地址
//    uint32_t offaddr; //去掉 0X08000000 后的地址
//
//    if (WriteAddr<FLASH_BASE||(WriteAddr>=(FLASH_BASE+1024*STM32_FLASH_SIZE)))return; //非法地址
//
//    HAL_FLASH_Unlock(); //解锁
//    offaddr=WriteAddr-FLASH_BASE; //实际偏移地址.
//    secpos=offaddr/STM_SECTOR_SIZE; //扇区地址 0~127 for STM32F103RBT6
//    secoff=(offaddr%STM_SECTOR_SIZE)/2; //在扇区内的偏移(2 个字节为基本单位.)
//    secremain=STM_SECTOR_SIZE/2-secoff; //扇区剩余空间大小
//    if (NumToWrite<=secremain)secremain=NumToWrite; //不大于该扇区范围
//    while (1) 
//    {
//        STMFLASH_Read(secpos*STM_SECTOR_SIZE+FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE/2);//读出整个扇区的内容
//        for (i=0; i<secremain; i++) 
//        { //校验数据
//            if (STMFLASH_BUF[secoff+i]!=0XFFFF)break; //需要擦除
//        }
//        if (i<secremain) 
//        { //需要擦除
//            FLASH_PageErase(secpos*STM_SECTOR_SIZE+FLASH_BASE);//擦除这个扇区
//            for (i=0; i<secremain; i++) { //复制
//            STMFLASH_BUF[i+secoff]=pBuffer[i];
//        }
//        STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);
//        //写入整个扇区
//        }
//        else 
//            STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//写已经擦除了的,直接写入扇区剩余区间.
//            
//        if (NumToWrite==secremain)
//        {
//            break; //写入结束了
//        }
//        else
//        { //写入未结束
//            secpos++; //扇区地址增 1
//            secoff=0; //偏移位置为 0
//            pBuffer+=secremain; //指针偏移
//            WriteAddr+=secremain; //写地址偏移
//            NumToWrite-=secremain; //字节(16 位)数递减
//            if (NumToWrite>(STM_SECTOR_SIZE/2))
//            {
//                secremain=STM_SECTOR_SIZE/2; //下一个扇区还是
//            }
//            else
//            {
//                secremain=NumToWrite;//下一个扇区可以写完了
//            }
//        }
//    }
//    HAL_FLASH_Lock();//上锁
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
//        HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
//		printf(UartRec);
//    USART_RX_CNT++;
    


    HAL_UART_Transmit(&huart2, UartRec, 1, 0);
    
    HAL_UART_Receive_IT(&huart2, UartRec, 1);
    

    APP_DATA[USART_RX_CNT] = UartRec[0];
    USART_RX_CNT++;
//    if (UartRec == '@' )

}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
