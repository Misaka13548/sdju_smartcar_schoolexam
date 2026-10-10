/*********************************************************************************************************************
* STC32G144K Opensourec Library 即（STC32G144K 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2025 SEEKFREE 逐飞科技
*
* 本文件是STC32G144K开源库的一部分
*
* STC32G144K 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK FOR C251
* 适用平台          STC32G144K
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-11-20        大W            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"

/* 电机 */
#define LEFT_MOTOR_DIR_1   ( IO_P50 )          // 左轮方向  // 对应物理引脚P5.0  
#define LEFT_MOTOR_PWM_1   ( PWMD_CH2_P51 )    // 左轮速度  // 代码内的 PWMD_CH2_P51 换成 LEFT_MOTOR_PWM_1	    // PWMD_CH2_P51  PWM的 D组 第4通道 对应 物理引脚P5.1
#define RIGHT_MOTOR_DIR_2   ( IO_P52 )          // 右轮方向 // 对应物理引脚P5.2
#define RIGHT_MOTOR_PWM_2   ( PWMD_CH4_P53 )    // 右轮速度 // 代码内的 PWMD_CH4_P53 换成 RIGHT_MOTOR_PWM_2	// PWMD_CH4_P53  PWM的 D组 第4通道 对应 物理引脚P5.3
#define MOTOR_FREQ          ( 17000 )

// 注意 DIR的只能输出高低两种状态 用来控制方向



/* 舵机 */
#define SERVO_PWM ( PWME_CH1P_PA0 )  // 代码内的 PWME_CH1P_PA0 换成 SERVO_PWM	// PWME_CH1P_PA0  PWM的 E组 第1通道 对应 物理引脚 PA0
#define SERVO_FREQ              ( 50 )                                           // 定义主板上舵机频率  请务必注意范围 50-300
#define SERVO_L_MAX             ( 80 )                                           // 定义主板上舵机活动范围最小值(80)
#define SERVO_R_MAX             ( 100 )                                           // 定义主板上舵机活动范围最大值(100)
																				// 中线 90 度
#define SERVO_DUTY(x)         ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_FREQ) * (0.5 + (float)(x) / 90.0))
// 这里把角度分为三步转换为了pwm信号的占空比
// (1) (0.5 + (float)(x) / 90.0)) 填入x获得了舵机需要的脉宽
// (2) (1000.0 / (float)SERVO_FREQ)) 获得了一帧多少ms
// (3) (10000.0 / (2)) 每 1 毫秒，对应多少个 duty 格
// (4) ((3)*(1)) 获得了 duty 占空比



/*
    IO_P50 IO_P52 PWMD_CH2_P51 PWMD_CH4_P53 在驱动板的中下部
    PWME_CH1P_PA0 板子最左侧那一排
*/




float servo_motor_duty = 90.0;                  // 舵机动作角度 此处设置为90度
int duty = 0;                                   // 占空比

/* 电感 */
// L1 左电感 L4 右电感 L2 中左电感 L3 中右电感 
volatile float adc_14 = 0.0f;        // 引脚 1.4  → ADC1_CH4_P14      左电感
volatile float adc_11 = 0.0f;        // 引脚 1.1  → ADC1_CH1_P11      中左电感
volatile float adc_07 = 0.0f;        // 引脚 0.7  → ADC2_CH7_P07      中右电感
volatile float adc_06 = 0.0f;        // 引脚 0.6  → ADC2_CH6_P06      右电感

float track_left = 0.0;                  // 左，右电感读数
float track_right = 0.0;





/* 调参 */
float Kp = 30.0;                       // 比例增益
float Kd = 10.0;                       // 微分增益
float period_s = 0.005;             // 控制周期
float alpha = 0.7;                  // 滤波参数 范围(0.0-1.0) 1.0 = 完全不滤波



/* 中间变量 */
float e = 0.0;
float error_last = 0.0f;        // 上一轮偏差 (e)
float d_filtered = 0.0f;        // D 项滤波记忆
float d_raw;
float angle;
float p_term;
float d_term;





/* PD */    // (中断函数 5ms)
void control_handler(void) {

    /* 左右电感读取 */
    adc_14 = (float)adc_mean_filter_convert(ADC1_CH4_P14, 5);           //  adc_mean_filter_convert: 采样 x 次取平均值
    adc_06 = (float)adc_mean_filter_convert(ADC2_CH6_P06, 5);

    track_left = adc_14;        // 左电感
    track_right = adc_06;       // 右电感

    e = (track_left - track_right) / (track_left + track_right);  // 求差值
    d_raw = (e - error_last) / period_s;                                // 求导
    error_last = e;                                                     // 把误差给到上一次里面
    d_filtered = alpha * d_raw + (1.0f - alpha) * d_filtered;           // D 项滤波记忆计算

    p_term = Kp * e;                                                    // 计算比例项                                                 
    // 偏得越多，打角越多

    d_term = Kd * d_filtered;                                           // 计算微分项
    // 偏得越快，提前打角越多 (阻尼)                                        // (如果使用 d_raw 就是不滤波)

    angle = 90.0f - (p_term + d_term);                                  // 角度 = 中值 - (p项 + d项)

    pwm_set_duty(SERVO_PWM, (uint32)SERVO_DUTY(angle));                 // 驱动
}



/* 主循环 */
void main(void)
{
    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化

    

    /* 电磁 */
    adc_init(ADC1_CH4_P14, ADC_12BIT);      // 引脚 1.4
    adc_init(ADC1_CH1_P11, ADC_12BIT);      // 引脚 1.1
    adc_init(ADC2_CH7_P07, ADC_12BIT);      // 引脚 0.7
    adc_init(ADC2_CH6_P06, ADC_12BIT);      // 引脚 0.6



    /* 电机 */
    gpio_init(LEFT_MOTOR_DIR_1, GPO, GPIO_LOW, GPO_PUSH_PULL);      // 选择了物理引脚P5.0 输出模式 低电平 推免（可高可低）
    pwm_init(LEFT_MOTOR_PWM_1, MOTOR_FREQ, 0);                      // 设置 P5.1 物理引脚的 0占空比   
    gpio_set_level(LEFT_MOTOR_DIR_1, GPIO_HIGH);					// 正转 让驱动模块的DIR端检测到高电平(high),造成正转   

    gpio_init(RIGHT_MOTOR_DIR_2, GPO, GPIO_LOW, GPO_PUSH_PULL);     // 选择了物理引脚P5.0 输出模式 低电平 推免（可高可低）  
    pwm_init(RIGHT_MOTOR_PWM_2, MOTOR_FREQ, 0);                     // 设置 P5.3 物理引脚的 0占空比
    gpio_set_level(RIGHT_MOTOR_DIR_2, GPIO_HIGH);					// 正转 让驱动模块的DIR端检测到高电平(high),造成正转   



    /* 舵机 */
    pwm_init(SERVO_PWM, SERVO_FREQ, 0);         // 设置 PA0 物理引脚的 pwm 信号的 0占空比

    pit_ms_init(TIM0_PIT, 5, control_handler);   // 5ms 中断

    while(1)
    {
        pwm_set_duty(LEFT_MOTOR_PWM_1, duty);
        pwm_set_duty(RIGHT_MOTOR_PWM_2, duty);

        if (duty < 750) {
            duty = duty + 25;       // 加速到 8% 保持不变
        }
        
    }
}


