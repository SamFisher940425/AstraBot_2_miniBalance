/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
u8 Ros_count=0,Ros_Rate = 0;                            //ROS发送数据的频率是10HZ，而中断在5ms一次就会把小车的移动的数据清0，这是为了让ros端控制小车运动的更加顺畅的一个计数参数
u8  Pick_up_stop=0;                         //检查是否被拿起标志位
int Middle_angle = -2;                           //机械中值默认为0
u16 Get_Voltage;                            //电池电压
u8 Way_Angle=2;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波 
u16 Flag_front,Flag_back,Flag_Left,Flag_Right,Flag_velocity=2,Target_Velocity=300; //蓝牙遥控相关的变量
float RC_Velocity,RC_Turn_Velocity;			//遥控控制的速度
u8 Flag_Stop=1,Flag_Show=0;                 //电机停止标志位和显示标志位  默认停止 显示打开
u32 speed_left;
u32 speed_right;
u8 PS2_ON_Flag = 0,Usart1_ON_Flag,PID_Send;		//默认所有方式不控制
u8 Mode = 0;								                //模式选择，默认是普通的控制模式
float Move_X,Move_Z;                          //控制小车避障、跟随时前进的变量，转弯的变量
u16 determine;
int Encoder_Left,Encoder_Right;             					//左右编码器的脉冲计数
int Motor_Left,Motor_Right;                 //电机PWM变量 应是Motor的 向Moto致敬	
int Temperature;                            //温度变量
int Voltage;                                //电池电压采样相关的变量
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
u32 Distance;                               //雷达测距
volatile u8 delay_50,delay_flag; 						//延时和调参相关变量
u8 Flag_follow=0,Flag_avoid=0,Flag_straight=0;							//雷达跟随、雷达避障标志位
u8 Lidar_Detect = Lidar_Detect_ON;			    //巡线模式雷达检测障碍物，默认开启
float Acceleration_Z;                       //Z轴加速度计  
u8 CCD_Zhongzhi,CCD_Yuzhi;                  //线性CCD相关
float Balance_Kp=20000,Balance_Kd=10,Velocity_Kp=220,Velocity_Ki=1,Turn_Kp=3200,Turn_Kd=10;//PID参数（放大100倍）110 100
float Distance_KP =250,Distance_KD =0 ;	//距离调整PID参数//10000
uint8_t Init_Finished_Flag = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

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
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_CAN_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);//原本由CubeMX工具生成的代码中存在HAL_NVIC_EnableIRQ(EXTI9_5_IRQn) 由于生成的代码总在用户代码之上，而这个中断是需要所有的外设初始化完毕最后才能开启的，所以在这里先失能，在所有外设初始化完成后再使能
	JTAG_Set(JTAG_SWD_DISABLE);     //关闭JTAG接口
	JTAG_Set(SWD_ENABLE);           //打开SWD接口 可以利用主板的SWD接口调试
  delay_init();                   //延迟函数初始化
  OLED_Init();                    //OLED初始化
  MPU6050_initialize();           //MPU6050初始化	
	DMP_Init();                     //初始化DMP 	
	Init_Finished_Flag = 1;
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn); //开启引脚外部中断
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(Flag_Show==0)          		//使用MiniBalance APP和OLED显示屏
		{
			 APP_Show();								//发送数据给APP
			 oled_show();          			//显示屏打开
			 PS2_Read();								//手柄数据读取
		}
		else                      		//使用MiniBalance上位机 上位机使用的时候需要严格的时序，故此时关闭app监控部分和OLED显示屏
		{
			 DataScope();          			//开启MiniBalance上位机
		}	
		delay_flag=1;	
		delay_50=0;
		while(delay_flag);		//示波器需要50ms	高精度延时，delay函数不满足要求，故使用MPU6050中断提供50ms延时
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
