
/* Includes ------------------------------------------------------------------*/

//#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
//#include "usbd_hid_core.h"
//#include "usbd_usr.h"
//#include "usbd_desc.h"

#include "main.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_adc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TESTRESULT_ADDRESS         0x080FFFFC
#define ALLTEST_PASS               0x00000000
#define ALLTEST_FAIL               0x55555555
#define BUFFERSIZE 128
#define ADC1_DR_Address   ((uint32_t)0x4001204C)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint16_t PrescalerValue = 0;
uint16_t ADCConvertedValues[BUFFERSIZE];
uint16_t USING_PIN[]={GPIO_Pin_3, GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_8, GPIO_Pin_9, GPIO_Pin_10, GPIO_Pin_11, GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14};

__IO uint32_t TimingDelay;
__IO uint8_t UserButtonPressed = 0x00;
__IO uint16_t ADCoverValue;

volatile int ConvertedValue = 4; //Converted value readed from ADC

/* Private function prototypes -----------------------------------------------*/
static void LED_task(void *pvParameters);
static void button_task(void *pvParameters);
static void adc_task(void *pvParameters);

void ADC_Config(void);
void NVIC_Config(void);
void GPIO_Output_Config(void);
void GPIO_Input_Config(void);
void DMA_Config(void);

/**
  * @brief  This function handles EXTI0_IRQ Handler.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
  UserButtonPressed = 0x01;
  
  /* Clear the EXTI line pending bit */
  EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
}

int main(void)
{
  RCC_ClocksTypeDef RCC_Clocks;
  
  /* Initialize LEDs and User_Button on STM32F4-Discovery --------------------*/
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI); 

  /* Initialize LEDs to be managed by GPIO */
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED6);

  /* Turn OFF all LEDs */
  STM_EVAL_LEDOff(LED4);
  STM_EVAL_LEDOff(LED3);
  STM_EVAL_LEDOff(LED5);
  STM_EVAL_LEDOff(LED6);
    
  /* Reset UserButton_Pressed variable */
  UserButtonPressed = 0x00;

  /* Setting for ADC and DMA */
  //RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);

  while(TimingDelay);
  FLASH_ProgramWord(TESTRESULT_ADDRESS, ALLTEST_PASS);

  SystemInit();
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  GPIO_Output_Config();
  GPIO_Input_Config();
  
  DMA_Config();
  ADC_Config();
  NVIC_Config();

  /* Create a task to flash the LED. */
  xTaskCreate(LED_task,
             (signed portCHAR *) "LED Flash",
             512 /* stack size */, NULL,
             tskIDLE_PRIORITY + 5, NULL);

  /* Create a task to button check. */
  xTaskCreate(button_task,
             (signed portCHAR *) "User Button",
             512 /* stack size */, NULL,
             tskIDLE_PRIORITY + 5, NULL);

  xTaskCreate(adc_task,
             (signed portCHAR *) "User ADC",
             512 /* stack size */, NULL,
             tskIDLE_PRIORITY + 5, NULL);

  /* Start running the tasks. */
  vTaskStartScheduler(); 

  return 0;
}

static void LED_task(void *pvParameters)
{
  RCC_ClocksTypeDef RCC_Clocks;
  uint8_t togglecounter = 0x00;

  while(1)
  {    
      /* Toggle LED5 */
      STM_EVAL_LEDToggle(LED5);
      vTaskDelay(100);
      /* Toggle LED6 */
      STM_EVAL_LEDToggle(LED6);
      vTaskDelay(100);
  }
}

static void button_task(void *pvParameters)
{
	while (1)
	{
		    /* Waiting User Button is pressed */
    		    if (UserButtonPressed == 0x01)
    		    {
      		    	/* Toggle LED4 */
      			STM_EVAL_LEDToggle(LED4);
      			vTaskDelay(100);
      			/* Toggle LED3 */
      			STM_EVAL_LEDToggle(LED3);
      			vTaskDelay(100);
    		    }
		    /* Waiting User Button is Released */
    		    while (STM_EVAL_PBGetState(BUTTON_USER) == Bit_SET);
		    UserButtonPressed = 0x00;
	}
}

static void adc_task(void *pvParameters){
  ADC_SoftwareStartConv(ADC1); // Start conversion by software.
  ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE); // Ready to handle interrupt.

  while (1) {
      GPIO_ResetBits(GPIOE, GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14);
      uint16_t sum = 0;
      
      register int i;
      for(i=0; i<4; ++i)
         sum|=(ConvertedValue & (1 << i)?USING_PIN[i]:0);
  
      GPIO_SetBits(GPIOE, sum);
      vTaskDelay(100);
  }

}

void ADC_Config(void)
{
    ADC_InitTypeDef ADC_InitStructure; // Structure for single-ADC configuration
    ADC_CommonInitTypeDef ADC_CommonInitStructure; // Structure for inter-ADC configuration

    // Clock configuration
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // ADC1 is connected to APB2 peripheral bus
    RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOCEN, ENABLE); // Clock for the ADC port!! (do not forget it)

    // ADC structure configuration
    ADC_DeInit(); // Reset all parameters to their default values
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; // Input voltage is converted into a 12-bit number whose maximum value is 4095
    ADC_InitStructure.ADC_ScanConvMode = ENABLE; // No scan (only one channel)
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // the conversion is continuous (periodic)
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; // no external trigger for conversion
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1; // use timer 1 capture/compare channel 1 for external trigger (may be forced)
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // converted data will be shifted to the right
    ADC_InitStructure.ADC_NbrOfConversion = 2; // Number of used ADC channels
    ADC_Init(ADC1, &ADC_InitStructure);      

    // ADC common structure configuration
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; // independent mode
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4; // f(ADC1)=84/4=21MHz
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; // disable DMA_MODE
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; // there are 5 clock cycles between 2 samplings
    ADC_CommonInit(&ADC_CommonInitStructure);

    // use channel 10 from ADC1, with sample time 15 cycles
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);

    ADC_ITConfig(ADC1, ADC_IT_EOC, DISABLE); // not ready for interrupt

    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
    ADC_DMACmd(ADC1, ENABLE);

    ADC_Cmd(ADC1, ENABLE);
}

void NVIC_Config()
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* ADC interrupt configure */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

void GPIO_Output_Config(void){
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource3|GPIO_PinSource4|GPIO_PinSource5|GPIO_PinSource6|GPIO_PinSource7|GPIO_PinSource8|GPIO_PinSource9|GPIO_PinSource10|GPIO_PinSource11|GPIO_PinSource12|GPIO_PinSource13|GPIO_PinSource14, GPIO_AF_TIM3);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;            // Alt Function - Push Pull
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init( GPIOE, &GPIO_InitStructure ); 
}

void GPIO_Input_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // Set GPIO clock
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  GPIO_StructInit(&GPIO_InitStructure);

  //Analog input pin configuration
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//The channel 10 is connected to PC0
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; //The PC0 pin is configured in analog mode
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //We don't need any pull up or pull down
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void DMA_Config(){
  DMA_InitTypeDef DMA_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  
  DMA_DeInit(DMA2_Stream4);
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) ADC1_DR_Address;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &ADCoverValue;

  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;

  DMA_InitStructure.DMA_BufferSize = 2;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;

  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_Init(DMA2_Stream4, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream4, ENABLE);

  DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);
}

void DMA2_Stream4_IRQHandler(){
  static int flag2=0;
  static int count2=0;
  if(DMA_GetITStatus(DMA2_Stream4, DMA_IT_TCIF0) != RESET){
    count2++;
    if(count2%1000000==0){
      if(flag2==0)
        GPIO_SetBits(GPIOE, GPIO_Pin_9), flag2=1;
      else
        GPIO_ResetBits(GPIOE, GPIO_Pin_9), flag2=0;
    }
    DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_TCIF0);
  }
}

//volatile int count_interrupt = 10; // count-down counter for interrupts
void ADC_IRQHandler(void)
{

  static int count=0;
  count++;

  if(ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET){
   ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
  }

  static int flag=0;
  if(count%1000000==0){
    if(flag==0)
      GPIO_SetBits(GPIOE, GPIO_Pin_8), flag=1;
    else
      GPIO_ResetBits(GPIOE, GPIO_Pin_8), flag=0;
  }
 
  return;
}

/**
  * @brief  This function handles the test program fail.
  * @param  None
  * @retval None
  */
void Fail_Handler(void)
{
  /* Erase last sector */ 
  FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
  /* Write FAIL code at last word in the flash memory */
  FLASH_ProgramWord(TESTRESULT_ADDRESS, ALLTEST_FAIL);
  
  while(1)
  {
    /* Toggle Red LED */
    STM_EVAL_LEDToggle(LED5);
    vTaskDelay(5);
  }
}


void vApplicationTickHook()
{
}


