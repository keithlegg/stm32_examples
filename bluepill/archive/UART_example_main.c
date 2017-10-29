/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************

KEITH WAS HERE 

SEE:
    http://pandafruits.com/stm32_primer/stm32_primer_uart.php


*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
ErrorStatus HSEStartUpStatus;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void NVIC_Configuration(void);
void Delay(vu32 nCount);




/*******************************************
 * Toggle LED 
 *******************************************/
void led_toggle(void)
{
    // Read LED output (GPIOA PIN8) status 
    //keith sayz - probably not configured TO BE READ!?
    uint8_t led_bit = GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13);
     
    if(led_bit == (uint8_t)Bit_SET)
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
    }else{
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
    }

}

/*****************************************************
 * Initialize USART1: enable interrupt on reception
 * of a character
 *****************************************************/


void USART1_Init(void)
{
    // USART configuration structure for USART1 
    USART_InitTypeDef usart1_init_struct;
    // Bit configuration structure for GPIOA PIN9 and PIN10 
    GPIO_InitTypeDef gpioa_init_struct;
     
    // Enalbe clock for USART1, AFIO and GPIOA
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO | 
                           RCC_APB2Periph_GPIOA, ENABLE);
                            
    // GPIOA PIN9 alternative function Tx 
    gpioa_init_struct.GPIO_Pin = GPIO_Pin_9;
    gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioa_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpioa_init_struct);

    // GPIOA PIN9 alternative function Rx 
    gpioa_init_struct.GPIO_Pin = GPIO_Pin_10;
    gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioa_init_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpioa_init_struct);
 
    // Enable USART1 
    USART_Cmd(USART1, ENABLE);  

    // Baud rate 9600, 8-bit data, One stop bit
    // No parity, Do both Rx and Tx, No HW flow control
    usart1_init_struct.USART_BaudRate = 9600;   
    usart1_init_struct.USART_WordLength = USART_WordLength_8b;  
    usart1_init_struct.USART_StopBits = USART_StopBits_1;   
    usart1_init_struct.USART_Parity = USART_Parity_No ;
    usart1_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart1_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    
    // Configure USART1 
    USART_Init(USART1, &usart1_init_struct);
    
    // Enable RXNE interrupt 
    //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    // Enable USART1 global interrupt 
    //NVIC_EnableIRQ(USART1_IRQn);
}





/**********************************************************
 * USART1 interrupt request handler: on reception of a 
 * character 't', toggle LED and transmit a character 'T'
 *********************************************************/

/*
void USART1_IRQHandler(void)
{
    // RXNE handler 
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        // If received 't', toggle LED and transmit 'T' 
        if((char)USART_ReceiveData(USART1) == 't')
        {
            led_toggle();
            USART_SendData(USART1, 'T');
            // Wait until Tx data register is empty, not really 
            // required for this example but put in here anyway.
            
            // while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}
                       
        }
    }
    // Other USART1 interrupts handler can go here ...      
}   
*/


/* Private functions ---------------------------------------------------------*/


//keith was here (Indiana book)
int putchar(int c){
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}
    USART1->DR = (c & 0xff);
    return 0; 
}


int main(void)
{
#ifdef DEBUG
  debug();
#endif

  /* Configure the system clocks */
  RCC_Configuration();
    
  /* NVIC Configuration */
  NVIC_Configuration();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //GPIO clock  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //keith was here 


  // Configure PC.4 as Output push-pull 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // keith was here 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  USART1_Init();

  while (1)
  {
      //GPIO_SetBits(GPIOA, GPIO_Pin_3);
      //Delay(0x1FFFFF);
      //GPIO_ResetBits(GPIOA, GPIO_Pin_3);

      //led_toggle();
      GPIO_SetBits(GPIOC, GPIO_Pin_13);
      //Delay(2000000);
      
      //led_toggle();
      GPIO_ResetBits(GPIOC, GPIO_Pin_13);
      //Delay(2000000);

      putchar(0xaa);
      putchar(0x55);
      putchar(0xf0);
      putchar(0x0f);

  }


  /*
    #define perif  (u32) (0x40010000)
    #define geepioa_base (perif + 0x0800)
    #define geepioa ((GPIO_TypeDef *) geepioa_base)

    while (1)
    {
        //GPIO_SetBits(geepioa, ((uint16_t)2) );
        //GPIO_ResetBits(geepioa,  ((uint16_t)2) );
        geepioa->BSRR = ((uint16_t)8);      
        geepioa->BRR = ((uint16_t)8); 

    }
  */


}

/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
 	
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
#ifdef  VECT_TAB_RAM  
  /* Set the Vector Table base location at 0x20000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif
}

/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : nCount: specifies the delay time length.
* Output         : None
* Return         : None
*******************************************************************************/
void Delay(vu32 nCount)
{
  for(; nCount != 0; nCount--);
}

#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
