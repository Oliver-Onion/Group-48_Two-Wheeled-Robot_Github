/**
  ******************************************************************************
  * @file           : ultrasonic.c
  * @brief          : Ultrasonic distance measurement implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ultrasonic.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static TIM_HandleTypeDef *ultrasonic_htim = NULL;
static uint32_t ultrasonic_IC_Val1 = 0;
static uint32_t ultrasonic_IC_Val2 = 0;
static uint32_t ultrasonic_Difference = 0;
static uint8_t ultrasonic_Is_First_Captured = 0;
static uint32_t ultrasonic_distance_mm = 0;
static uint8_t ultrasonic_data_ready = 0;
static uint16_t ultrasonic_sampling_interval = 100;  // 默认100ms
static uint32_t ultrasonic_last_trigger_time = 0;

/* Private function prototypes -----------------------------------------------*/
static void ultrasonic_delay(uint16_t time);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  微秒级延时函数
  * @param  time: 延时时间（微秒）
  * @retval None
  */
static void ultrasonic_delay(uint16_t time) {
    if (ultrasonic_htim != NULL) {
        __HAL_TIM_SET_COUNTER(ultrasonic_htim, 0);
        while (__HAL_TIM_GET_COUNTER(ultrasonic_htim) < time);
    }
}

/**
  * @brief  初始化超声波测距模块
  * @param  htim: 定时器句柄指针
  * @retval None
  */
void Ultrasonic_Init(TIM_HandleTypeDef *htim) {
    ultrasonic_htim = htim;
    ultrasonic_distance_mm = 0;
    ultrasonic_data_ready = 0;
    ultrasonic_Is_First_Captured = 0;
    ultrasonic_last_trigger_time = 0;
    
    // 启动输入捕获中断
    if (ultrasonic_htim != NULL) {
        HAL_TIM_IC_Start_IT(ultrasonic_htim, TIM_CHANNEL_1);
    }
}

/**
  * @brief  触发超声波测距
  * @param  None
  * @retval None
  */
void Ultrasonic_TriggerMeasurement(void) {
    if (ultrasonic_htim == NULL) return;
    
    // 产生10us的高电平脉冲
    HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, GPIO_PIN_SET);
    ultrasonic_delay(10);
    HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, GPIO_PIN_RESET);
    
    // 启用输入捕获中断
    __HAL_TIM_ENABLE_IT(ultrasonic_htim, TIM_IT_CC1);
    
    ultrasonic_data_ready = 0;  // 标记数据未准备好
}

/**
  * @brief  获取距离值（毫米）
  * @param  None
  * @retval 距离值（毫米）
  */
uint32_t Ultrasonic_GetDistanceMM(void) {
    uint32_t current_time = HAL_GetTick();
    
    // 自动触发测量
    if (current_time - ultrasonic_last_trigger_time >= ultrasonic_sampling_interval) {
        Ultrasonic_TriggerMeasurement();
        ultrasonic_last_trigger_time = current_time;
    }
    
    return ultrasonic_distance_mm;
}

/**
  * @brief  检查数据是否准备好
  * @param  None
  * @retval 1: 数据准备好, 0: 数据未准备好
  */
uint8_t Ultrasonic_IsDataReady(void) {
    return ultrasonic_data_ready;
}

/**
  * @brief  设置采样间隔
  * @param  interval_ms: 采样间隔（毫秒）
  * @retval None
  */
void Ultrasonic_SetSamplingInterval(uint16_t interval_ms) {
    ultrasonic_sampling_interval = interval_ms;
}

/**
  * @brief  获取采样间隔
  * @param  None
  * @retval 采样间隔（毫秒）
  */
uint16_t Ultrasonic_GetSamplingInterval(void) {
    return ultrasonic_sampling_interval;
}

/**
  * @brief  定时器输入捕获回调函数
  * @param  htim: 定时器句柄指针
  * @retval None
  * @note   这个函数应该在main.c的HAL_TIM_IC_CaptureCallback中调用
  */
void Ultrasonic_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance != ultrasonic_htim->Instance) return;
    
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        if (ultrasonic_Is_First_Captured == 0) {
            // 捕获第一个值（上升沿）
            ultrasonic_IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            ultrasonic_Is_First_Captured = 1;
            // 改变极性为下降沿
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else if (ultrasonic_Is_First_Captured == 1) {
            // 捕获第二个值（下降沿）
            ultrasonic_IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            __HAL_TIM_SET_COUNTER(htim, 0);  // 重置计数器
            
            // 计算时间差
            ultrasonic_Difference = (ultrasonic_IC_Val2 >= ultrasonic_IC_Val1) ? 
                                   (ultrasonic_IC_Val2 - ultrasonic_IC_Val1) : 
                                   (0xFFFF - ultrasonic_IC_Val1 + ultrasonic_IC_Val2 + 1);
            
            // 计算距离 (mm)
            // 公式: 距离 = (时间差 * 声速) / 2
            // 声速 = 343 m/s = 0.343 mm/us
            // 定时器每个tick的时间 = (Prescaler + 1) / PCLK2_Freq (微秒)
            float tick_duration_us = (float)(htim->Init.Prescaler + 1) / (HAL_RCC_GetPCLK2Freq() / 1000000.0f);
            ultrasonic_distance_mm = (uint32_t)((ultrasonic_Difference * tick_duration_us * 0.343f) / 2.0f);
            
            ultrasonic_Is_First_Captured = 0;  // 重置状态
            ultrasonic_data_ready = 1;  // 标记数据准备好
            
            // 设置极性为上升沿
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);  // 禁用中断
        }
    }
} 