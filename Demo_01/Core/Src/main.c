/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <includes.h>
#include "stm32f1xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*csdn 参考网站:
*(1)本次项目代码移植参考：
		https://blog.csdn.net/qq_45945548/article/details/121667025?ops_request_misc=&request_id=&biz_id=102&utm_term=ucosiii%E5%AE%98%E6%96%B9stm32f103%E7%A7%BB%E6%A4%8D&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-1-121667025.142^v67^control,201^v3^control_2,213^v2^t3_esquery_v1&spm=1018.2226.3001.4187
*(2)软件仿真参考：
		https://blog.csdn.net/qq_41675500/article/details/121727792?app_version=5.11.1&code=app_1562916241&csdn_share_tail=%7B%22type%22%3A%22blog%22%2C%22rType%22%3A%22article%22%2C%22rId%22%3A%22121727792%22%2C%22source%22%3A%22qq_44596875%22%7D&uLinkId=usr1mkqgl919blen&utm_source=app
*/
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 任务优先级*/
#define START_TASK_PRIO		3
#define LED0_TASK_PRIO		4
#define MSG_TASK_PRIO		  5

/* 任务栈大小	*/
#define START_STK_SIZE 		64
#define LED0_STK_SIZE 		64
#define MSG_STK_SIZE 		  64 //任务栈大小过大会报错，可以试着改小一点

/* 任务栈 */	
CPU_STK START_TASK_STK[START_STK_SIZE];
CPU_STK LED0_TASK_STK[LED0_STK_SIZE];
CPU_STK MSG_TASK_STK[MSG_STK_SIZE];
/* 任务控制块 */
OS_TCB StartTaskTCB;
OS_TCB Led0TaskTCB;
OS_TCB MsgTaskTCB;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 任务函数定义*/
void start_task(void *p_arg);
static  void  AppTaskCreate(void);
static  void  AppObjCreate(void);
static  void  led_pc13(void *p_arg);
static  void  send_msg(void *p_arg);
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
	OS_ERR  err;
	OSInit(&err);
  /* USER CODE BEGIN 1 */

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
  //MX_GPIO_Init();  //这个在BSP的初始化里也会初始化
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
/* 创建任务 */
	OSTaskCreate((OS_TCB     *)&StartTaskTCB,                /* Create the start task                                */
				 (CPU_CHAR   *)"start task",
				 (OS_TASK_PTR ) start_task,
				 (void       *) 0,
				 (OS_PRIO     ) START_TASK_PRIO,
				 (CPU_STK    *)&START_TASK_STK[0],
				 (CPU_STK_SIZE) START_STK_SIZE/10,
				 (CPU_STK_SIZE) START_STK_SIZE,
				 (OS_MSG_QTY  ) 0,
				 (OS_TICK     ) 0,
				 (void       *) 0,
				 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				 (OS_ERR     *)&err);
	/* 启动多任务系统，控制权交给uC/OS-III */
	OSStart(&err);            /* Start multitasking (i.e. give control to uC/OS-III). */
			         

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
/* 
			 while (1)
  {
		
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET); 		//GPIO_RED 置1 灭    
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);  //GPIO_BLUE 置0 亮
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET); 		//GPIO_RED 置1    
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET); 			//GPIO_BLUE 置0
		HAL_Delay(500);
     USER CODE END WHILE 

     USER CODE BEGIN 3 
  }
  USER CODE END 3 */
}


void start_task(void *p_arg){
OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;
	
  BSP_Init();                                                   /* Initialize BSP functions */
  //CPU_Init();
  //Mem_Init();                                                 /* Initialize Memory Management Module */

#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  		//统计任务           
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN			//如果使能了测量中断 关闭时间
    CPU_IntDisMeasMaxCurReset();	
#endif

#if	OS_CFG_SCHED_ROUND_ROBIN_EN  		//当使用时间片轮转的时候
	 //使能时间片轮转调度功能，时间片长度为1个系统时间节拍————(1*5=5ms)
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif		
	
	OS_CRITICAL_ENTER();	//进入临界区
	/* 创建LED0任务 */
	OSTaskCreate((OS_TCB 	* )&Led0TaskTCB,		
				 (CPU_CHAR	* )"led_pc13", 		
                 (OS_TASK_PTR )led_pc13, 			
                 (void		* )0,					
                 (OS_PRIO	  )LED0_TASK_PRIO,     
                 (CPU_STK   * )&LED0_TASK_STK[0],	
                 (CPU_STK_SIZE)LED0_STK_SIZE/10,	
                 (CPU_STK_SIZE)LED0_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);				
				 
	/* 创建LED1任务  */
	OSTaskCreate((OS_TCB 	* )&MsgTaskTCB,		
				 (CPU_CHAR	* )"send_msg", 		
                 (OS_TASK_PTR )send_msg, 			
                 (void		* )0,					
                 (OS_PRIO	  )MSG_TASK_PRIO,     	
                 (CPU_STK   * )&MSG_TASK_STK[0],	
                 (CPU_STK_SIZE)MSG_STK_SIZE/10,	
                 (CPU_STK_SIZE)MSG_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);
				 
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//挂起开始任务			 
	OS_CRITICAL_EXIT();	//进入临界区

}

/**
  * 函数功能：启动任务函数载体
  * 输入参数：p_arg 是在创建该任务是传递的形参
  * 返回值：无
  * 说  明：无
  */
static  void  led_pc13 (void *p_arg)
{
  OS_ERR      err;

  (void)p_arg;

  BSP_Init();                                                 /* Initialize BSP functions                             */
  CPU_Init();

  Mem_Init();                                                 /* Initialize Memory Management Module                  */

#if OS_CFG_STAT_TASK_EN > 0u
  OSStatTaskCPUUsageInit(&err);                               /* Compute CPU capacity with no task running            */
#endif

  CPU_IntDisMeasMaxCurReset();

  AppTaskCreate();                                            /* Create Application Tasks                             */

  AppObjCreate();                                             /* Create Application Objects                           */

	/**
	*PIN_9 LED_RED
	*PIN_10 LED_BLUE
	*/
  while (DEF_TRUE)
  {
		HAL_GPIO_WritePin(GPIOB,LED_RED_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,LED_BLUE_Pin,GPIO_PIN_SET);
		printf("LED_BLUE is lightup \r\n");
		OSTimeDlyHMSM(0, 0, 1, 0,OS_OPT_TIME_HMSM_STRICT,&err);
		HAL_GPIO_WritePin(GPIOB,LED_RED_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,LED_BLUE_Pin,GPIO_PIN_RESET);
		printf("LED_RED is lightup \r\n");
		OSTimeDlyHMSM(0, 0, 1, 0,OS_OPT_TIME_HMSM_STRICT,&err);
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

static  void  send_msg (void *p_arg)
{
  OS_ERR      err;

  (void)p_arg;

  BSP_Init();                                                 /* Initialize BSP functions                             */
  CPU_Init();

  Mem_Init();                                                 /* Initialize Memory Management Module                  */

#if OS_CFG_STAT_TASK_EN > 0u
  OSStatTaskCPUUsageInit(&err);                               /* Compute CPU capacity with no task running            */
#endif

  CPU_IntDisMeasMaxCurReset();

  AppTaskCreate();                                            /* Create Application Tasks                             */

  AppObjCreate();                                             /* Create Application Objects                           */

  while (DEF_TRUE)
  {
			//printf("hello world \r\n");
		printf("hello uc/OS 欢迎来到RTOS多任务环境！ \r\n");
		OSTimeDlyHMSM(0, 0, 1, 0,OS_OPT_TIME_HMSM_STRICT,&err);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


/* USER CODE BEGIN 4 */
/**
  * 函数功能：创建应用任务
  */
static  void  AppTaskCreate (void)
{
  
}

/**
	* 函数功能: uCosIII内核对象创建
  */
static  void  AppObjCreate (void)
{

}





/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
