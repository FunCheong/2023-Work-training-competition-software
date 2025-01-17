/**
 * @file pid.c
 * @author ZheWana
 * @brief PID控制器源文件
 * @version 0.1
 * @date 2022-03-12
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "./pid.h"
#include "main.h"
#include "math.h"

float PID_Realize(Pid_t *pid, float input) {
    pid->ctr.cur = input;

    pid->error.cur = pid->ctr.aim - pid->ctr.cur;
    pid->error.sum += pid->error.cur;
    pid->error.bia = pid->error.cur - pid->error.pre;
    pid->error.pre = pid->error.cur;

    pid->ctr.pre = pid->ctr.cur;
    return pid->kp * pid->error.cur + pid->ki * pid->error.sum + pid->kd * pid->error.bia;
}

//PID for orientation loop
float PID_RealizeForAngle(Pid_t *pid, float input) {
    pid->ctr.cur = input;

    pid->error.cur = pid->ctr.aim - pid->ctr.cur;
    if (fabsf(pid->error.cur) > M_PI && fabsf(pid->error.cur) < 2 * M_PI) {
        if (pid->error.cur > 0) {
            pid->error.cur -= 2 * M_PI;
        } else if (pid->error.cur < 0) {
            pid->error.cur += 2 * M_PI;
        }
    }
    pid->error.sum += pid->error.cur;
    pid->error.bia = pid->error.cur - pid->error.pre;
    pid->error.pre = pid->error.cur;

    pid->ctr.pre = pid->ctr.cur;
    return pid->kp * pid->error.cur + pid->ki * pid->error.sum + pid->kd * pid->error.bia;
}

void PID_Init(Pid_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_Reset(Pid_t *pid) {
    pid->ctr.cur = 0;
    pid->ctr.pre = 0;
    pid->error.cur = 0;
    pid->error.pre = 0;
    pid->error.bia = 0;
    pid->error.sum = 0;
}

float Slew_Func(float *slewVal, float refVal, float slewRate)
{
    static float diff = 0;
    diff = refVal - *slewVal;
    if (diff >= slewRate)
    {
        *slewVal += slewRate;
        return (1);
    } else if (-diff >= slewRate)
    {
        *slewVal -= slewRate;
        return (-1);
    } else
    {
        *slewVal = refVal;
        return (0);
    }
}
