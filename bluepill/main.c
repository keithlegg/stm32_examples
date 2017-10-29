

// http://embeddedsystemengineering.blogspot.com/2016/04/arm-cortex-m3-stm32f103-tutorial-spi.html


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



enum spiSpeed { SPI_SLOW , SPI_MEDIUM , SPI_FAST };
/*
void spiInit ( SPI_TypeDef * SPIx);
int spiReadWrite ( SPI_TypeDef * SPIx , uint8_t *rbuf ,
                   const uint8_t *tbuf , int cnt ,
                   enum spiSpeed speed );

int spiReadWrite16 ( SPI_TypeDef * SPIx , uint16_t *rbuf ,
                     const uint16_t *tbuf , int cnt ,
                     enum spiSpeed speed );
*/


static const uint16_t speeds [] = {
[ SPI_SLOW ] = SPI_BaudRatePrescaler_64 ,
[ SPI_MEDIUM ] = SPI_BaudRatePrescaler_8 ,
[ SPI_FAST ] = SPI_BaudRatePrescaler_2 };


//#define SPIx SPI1
#define SPIx_RCC RCC_APB2Periph_SPI1
#define SPI_GPIO_RCC RCC_APB2Periph_GPIOA
#define SPI_GPIO GPIOA
#define SPI_PIN_MOSI GPIO_Pin_7
#define SPI_PIN_MISO GPIO_Pin_6
#define SPI_PIN_SCK GPIO_Pin_5
#define SPI_PIN_SS GPIO_Pin_4


void spiInit ( SPI_TypeDef *SPIx)
{
    SPI_InitTypeDef SPI_InitStructure ;
    GPIO_InitTypeDef GPIO_InitStructure ;
    GPIO_StructInit (& GPIO_InitStructure );
    SPI_StructInit (& SPI_InitStructure );
    
    // Step 1: Initialize SPI
    RCC_APB2PeriphClockCmd(SPIx_RCC, ENABLE);
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
    SPI_Init(SPIx, &SPI_InitStructure);
    SPI_Cmd(SPIx, ENABLE);

    // Step 2: Initialize GPIO
    RCC_APB2PeriphClockCmd(SPI_GPIO_RCC, ENABLE);
    // GPIO pins for MOSI, MISO, and SCK (MISO IS WRONG - SHOULD BE INPUT )
    GPIO_InitStructure.GPIO_Pin =  SPI_PIN_SCK | SPI_PIN_MOSI; //SPI_PIN_MISO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
    // GPIO pin for SS
    GPIO_InitStructure.GPIO_Pin = SPI_PIN_SS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SPI_GPIO, &GPIO_InitStructure);

    // keith was here to set MOSI as input pull up
    GPIO_InitStructure.GPIO_Pin =  SPI_PIN_MISO; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
    


    // Disable SPI slave device
    SPIx_DisableSlave();


}


uint8_t SPIx_Transfer(SPI_TypeDef *SPIx, uint8_t data)
{
    // Write data to be transmitted to the SPI data register
    SPIx->DR = data;
    // Wait until transmit complete
    while (!(SPIx->SR & (SPI_FLAG_TXE)));
    // Wait until receive complete
    while (!(SPIx->SR & (SPI_FLAG_RXNE)));
    // Wait until SPI is not busy anymore
    while (SPIx->SR & (SPI_FLAG_BSY));
    // Return received data from SPI data register
    return SPIx->DR;
}
 
void SPIx_EnableSlave()
{
    // Set slave SS pin low
    //SPI_GPIO->BRR = SPI_PIN_SS;
    
    GPIO_ResetBits(SPI_GPIO, SPI_PIN_SS);

}
 
void SPIx_DisableSlave()
{
    // Set slave SS pin high
    //SPI_GPIO->BSRR = SPI_PIN_SS;

    GPIO_SetBits(SPI_GPIO, SPI_PIN_SS);

}


///////////////////////////////////////////////////////



int spiReadWrite ( SPI_TypeDef * SPIx , uint8_t *rbuf ,
                   const uint8_t *tbuf , int cnt , enum spiSpeed speed)
{
    int i;
    SPIx ->CR1 = (SPIx ->CR1 & ~ SPI_BaudRatePrescaler_256 ) |
    speeds [ speed ];
    for (i = 0; i < cnt; i++){
        if (tbuf) {
            SPI_SendData(SPIx , *tbuf ++);
        } else {
           SPI_SendData (SPIx , 0xff);
        }
        while ( SPI_GetFlagStatus (SPIx , SPI_FLAG_RXNE ) == RESET)
        {    
            if (rbuf) {
            *rbuf ++ = SPI_ReceiveData (SPIx);
            } else {
               SPI_ReceiveData (SPIx);
            }
        }  

    }
    return i;
}



///////////////////////////////////////////////////////


uint8_t txbuf [4], rxbuf [4];
uint16_t txbuf16 [4], rxbuf16 [4];

int main(void)
{
    #ifdef DEBUG
      debug();
    #endif
    RCC_Configuration();
    NVIC_Configuration();
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    spiInit(SPI1);

    int i, j;
    
    //csInit (); // Initialize chip select PC03
    /*
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    //keith was here 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    */

    uint8_t buffer = 0;


    //while(1){
       
        /*
        for (i = 0; i < 8; i++) {
            for (j = 0; j < 4; j++)
                txbuf [j] = i*4 + j;
            //GPIO_WriteBit (GPIOC , GPIO_Pin_3 , 0);
            spiReadWrite (SPI1 , rxbuf , txbuf , 4, SPI_SLOW );
            //GPIO_WriteBit (GPIOC , GPIO_Pin_3 , 1);
           // for (j = 0; j < 4; j++)
           //     if (rxbuf [j] != txbuf [j])
           //         assert_failed (__FILE__ , __LINE__ );
        }  */


        //spiReadWrite(SPI1 , rxbuf , txbuf , 4, SPI_SLOW);

        SPIx_EnableSlave();
        buffer = SPIx_Transfer(SPI1 , 0xf0);
        SPIx_DisableSlave();

        SPIx_EnableSlave();
        buffer = SPIx_Transfer(SPI1 , 0x0f);
        SPIx_DisableSlave();

        SPIx_EnableSlave();
        buffer = SPIx_Transfer(SPI1 , 0x55);
        SPIx_DisableSlave();

     

   // }
    //////////////////////


    /*
    for (i = 0; i < 8; i++) {
        for (j = 0; j < 4; j++)
            txbuf16 [j] = i*4 + j + (i << 8);
        GPIO_WriteBit (GPIOC , GPIO_Pin_3 , 0);
        spiReadWrite16 (SPI1 , rxbuf16 , txbuf16 , 4, SPI_SLOW );
        GPIO_WriteBit (GPIOC , GPIO_Pin_3 , 1);
        //for (j = 0; j < 4; j++)
        //    if ( rxbuf16 [j] != txbuf16 [j])
        //        assert_failed (__FILE__ , __LINE__ );
    }*/

}//end main 



/*******************************************************************************/
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
