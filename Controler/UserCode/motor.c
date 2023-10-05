/**
 * @file motor.c
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2023/1/2
  */
#include "motor.h"
#include "tim.h"
#include "main.h"
#include "printf.h"


int Motor_Drive(int motorID, float duty, int dir) {
    //Hardware pins table
    static TIM_TypeDef *TIMx[5] = {TIM12, TIM16, TIM12, TIM15, TIM15};
    static volatile uint32_t *CCRx[5] =
            {&TIM12->CCR1, &TIM16->CCR1, &TIM12->CCR2, &TIM15->CCR1, &TIM15->CCR2};
    static GPIO_TypeDef *GPIOx[5] =
            {Dir2_GPIO_Port, Dir3_GPIO_Port, Dir1_GPIO_Port, Dir4_GPIO_Port, DirStep_GPIO_Port};
    static const uint32_t GPIO_Pinx[5] = {Dir2_Pin, Dir3_Pin, Dir1_Pin, Dir4_Pin, DirStep_Pin};

    uint32_t pinState = 0;

    if (duty > 100)duty = 100;
    else if (duty < 0)duty = 0;

    if (motorID == 0 || motorID == 3)
        pinState = dir;
    else
        pinState = !dir;

    *CCRx[motorID] = (uint32_t) ((float) (TIMx[motorID]->ARR) * duty / 100);
    HAL_GPIO_WritePin(GPIOx[motorID], GPIO_Pinx[motorID], pinState);

    return 0;
}
