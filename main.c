
/* Includes ------------------------------------------------------------------*/

//#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "stm32f4xx_conf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TESTRESULT_ADDRESS         0x080FFFFC
#define ALLTEST_PASS               0x00000000
#define ALLTEST_FAIL               0x55555555

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint16_t PrescalerValue = 0;

__IO uint32_t TimingDelay;
__IO uint8_t UserButtonPressed = 0x00;


/* Private function prototypes -----------------------------------------------*/
static void LED_task(void *pvParameters);
static void button_task(void *pvParameters);

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


