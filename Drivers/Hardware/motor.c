#include "motor.h"

void PID_Motor_Init(
    PID_Motor_t *self,
    TIM_HandleTypeDef *encoder_htim, TIM_HandleTypeDef *pwm_htim, TIM_HandleTypeDef *timer_htim,
    uint32_t pwm_channel, GPIO_TypeDef *IN1_port, uint16_t IN1_pin,
    GPIO_TypeDef *IN2_port, uint16_t IN2_pin,
    GPIO_TypeDef *STBY_port, uint16_t STBY_pin,
    float Gear_Reduction, float Encoder_Resolution,
    float Encoder_Multipiler, float max_speed, uint16_t Speed_Test_Period_ms,
    float kp, float ki, float kd, float max_all_error,
    float pos_kp, float pos_ki, float pos_kd, float pos_max_all_error)
{
    self->encoder_htim = encoder_htim;
    self->pwm_htim = pwm_htim;
    self->pwm_channel = pwm_channel;

    self->IN1_port = IN1_port;
    self->IN1_pin = IN1_pin;
    self->IN2_port = IN2_port;
    self->IN2_pin = IN2_pin;
    self->STBY_port = STBY_port;
    self->STBY_pin = STBY_pin;

    self->gear_reduction = Gear_Reduction;
    self->encoder_resolution = Encoder_Resolution;
    self->encoder_multipiler = Encoder_Multipiler;
    self->max_speed = max_speed;

    self->speed_test_period_ms = Speed_Test_Period_ms;
    self->last_tick = HAL_GetTick();

    self->current_counter = 0;
    self->pulse_delta = 0;
    self->last_counter = __HAL_TIM_GetCounter(encoder_htim);
    self->current_rpm = 0;
    self->current_state = MOTOR_STOP;

    self->kp = kp;
    self->ki = ki;
    self->kd = kd;
    self->max_all_error = max_all_error;

    self->current_position = 0.f;
    self->pos_kp = pos_kp;
    self->pos_ki = pos_ki;
    self->pos_kd = pos_kd;
    self->pos_max_all_error = pos_max_all_error;

    self->target_rpm = 0.f;
    self->last_error = 0;
    self->all_error = 0;

    __HAL_TIM_SetAutoreload(timer_htim, 10 * Speed_Test_Period_ms);

    HAL_TIM_Encoder_Start(encoder_htim, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start_IT(timer_htim);
}

void PID_Motor_Get_Infos(PID_Motor_t *self)
{
    self->current_counter = __HAL_TIM_GET_COUNTER(self->encoder_htim);
    self->pulse_delta = (int16_t)(self->current_counter - self->last_counter);
    self->last_counter = self->current_counter;

    self->current_rpm =
        60.f * self->pulse_delta / self->gear_reduction / self->encoder_resolution / self->encoder_multipiler / (self->speed_test_period_ms / 1000.f);

    self->current_position = self->current_position +
                             (float)self->pulse_delta / self->gear_reduction / self->encoder_resolution / self->encoder_multipiler;
}

void PID_Motor_Run(PID_Motor_t *self, int16_t pwm_val)
{

    if (pwm_val >= 1000)
        pwm_val = 1000;
    if (pwm_val <= -1000)
        pwm_val = -1000;

    if (pwm_val < 0)
    {
        HAL_GPIO_WritePin(self->STBY_port, self->STBY_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(self->IN1_port, self->IN1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(self->IN2_port, self->IN2_pin, GPIO_PIN_RESET);

        __HAL_TIM_SetCompare(self->pwm_htim, self->pwm_channel, -pwm_val);
        HAL_TIM_PWM_Start(self->pwm_htim, self->pwm_channel);
        self->current_state = MOTOR_CCW;
    }
    else if (pwm_val > 0)
    {
        HAL_GPIO_WritePin(self->STBY_port, self->STBY_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(self->IN1_port, self->IN1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(self->IN2_port, self->IN2_pin, GPIO_PIN_SET);

        __HAL_TIM_SetCompare(self->pwm_htim, self->pwm_channel, pwm_val);
        HAL_TIM_PWM_Start(self->pwm_htim, self->pwm_channel);
        self->current_state = MOTOR_CW;
    }
}

void PID_Motor_Set_Target(PID_Motor_t *self, float target_speed)
{
    if (target_speed >= self->max_speed)
        self->target_rpm = self->max_speed;

    else if (target_speed <= -self->max_speed)
        self->target_rpm = -self->max_speed;

    else
    {
        self->target_rpm = target_speed;
    }
}

void PID_Motor_Control(PID_Motor_t *self)
{
    self->error = self->target_rpm - self->current_rpm;
    self->all_error += self->error;

    if (self->all_error >= self->max_all_error)
        self->all_error = self->max_all_error;
    if (self->all_error <= -self->max_all_error)
        self->all_error = -self->max_all_error;

    float output = self->kp * self->error + self->ki * self->all_error + self->kd * (self->error - self->last_error);
    self->pwm_output = (int16_t)output;

    if (self->pwm_output >= 1000)
        self->pwm_output = 1000;
    if (self->pwm_output <= -1000)
        self->pwm_output = -1000;

    self->last_error = self->error;

    if (self->target_rpm > -0.01f && self->target_rpm < 0.01f)
    {
        if (self->current_rpm >= -3.f && self->current_rpm <= 3.f)
        {
            self->all_error = 0;
            self->error = 0;
            self->last_error = 0;
            self->pwm_output = 0;

            HAL_GPIO_WritePin(self->STBY_port, self->STBY_pin, GPIO_PIN_RESET);
            __HAL_TIM_SetCompare(self->pwm_htim, self->pwm_channel, 0);
            HAL_TIM_PWM_Stop(self->pwm_htim, self->pwm_channel);
            self->current_state = MOTOR_STOP;
        }
    }

    PID_Motor_Run(self, self->pwm_output);
}

void PID_Motor_Set_Pos(PID_Motor_t *self, float target)
{
    self->target_position = target;
}

void PID_Motor_Pos_Control(PID_Motor_t *self)
{
    self->pos_error = self->target_position - self->current_position;
    self->pos_all_error += self->pos_error;

    if (self->pos_all_error >= self->pos_max_all_error)
        self->pos_all_error = self->pos_max_all_error;
    if (self->pos_all_error <= -self->pos_max_all_error)
        self->pos_all_error = -self->pos_max_all_error;

    float output = self->pos_kp * self->pos_error + self->pos_ki * self->pos_all_error + self->pos_kd * (self->pos_error - self->pos_last_error);
    self->pos_last_error = self->pos_error;

    PID_Motor_Set_Target(self, output);
}
