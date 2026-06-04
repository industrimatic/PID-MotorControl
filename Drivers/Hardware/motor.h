#ifndef __PID_MOTOR_H
#define __PID_MOTOR_H

#include "stm32f1xx_hal.h"

typedef enum
{
    MOTOR_STOP,
    MOTOR_CW,
    MOTOR_CCW,
} MotorState_t;

typedef struct
{
    TIM_HandleTypeDef *encoder_htim;
    TIM_HandleTypeDef *pwm_htim;
    uint32_t pwm_channel;

    GPIO_TypeDef *IN1_port;
    uint16_t IN1_pin;
    GPIO_TypeDef *IN2_port;
    uint16_t IN2_pin;
    GPIO_TypeDef *STBY_port;
    uint16_t STBY_pin;

    float gear_reduction;
    float encoder_resolution;
    float encoder_multipiler;
    float max_speed;

    uint16_t speed_test_period_ms;
    uint32_t last_tick;

    float current_rpm;
    uint16_t last_counter;
    int16_t pulse_delta;
    uint16_t current_counter;
    MotorState_t current_state;

    float target_rpm;
    float kp;
    float ki;
    float kd;

    float error;
    float last_error;
    float all_error;
    float max_all_error;
    int16_t pwm_output;

    float current_position;
    float target_position;
    float pos_kp;
    float pos_ki;
    float pos_kd;
    float pos_all_error;
    float pos_max_all_error;
    float pos_error;
    float pos_last_error;

} PID_Motor_t;

void PID_Motor_Init(
    PID_Motor_t *self,
    TIM_HandleTypeDef *encoder_htim, TIM_HandleTypeDef *pwm_htim, TIM_HandleTypeDef *timer_htim,
    uint32_t pwm_channel, GPIO_TypeDef *IN1_port, uint16_t IN1_pin,
    GPIO_TypeDef *IN2_port, uint16_t IN2_pin,
    GPIO_TypeDef *STBY_port, uint16_t STBY_pin,
    float Gear_Reduction, float Encoder_Resolution,
    float Encoder_Multipiler, float max_speed, uint16_t Speed_Test_Period_ms,
    float kp, float ki, float kd, float max_all_error,
    float pos_kp, float pos_ki, float pos_kd, float pos_max_all_error);

void PID_Motor_Get_Infos(PID_Motor_t *self);
void PID_Motor_Run(PID_Motor_t *self, int16_t pwm_val);
void PID_Motor_Set_Target(PID_Motor_t *self, float target_speed);
void PID_Motor_Control(PID_Motor_t *self);
void PID_Motor_Set_Pos(PID_Motor_t *self, float target);
void PID_Motor_Pos_Control(PID_Motor_t *self);

#endif
