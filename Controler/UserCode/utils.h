/**
 * @file utils.h
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2022/11/8
  */
#ifndef CONTROLCENTER_UTILS_H
#define CONTROLCENTER_UTILS_H

#include "stdint.h"
#include "stdbool.h"
#include "PID/pid.h"
#include "PMW3901/PMW3901.h"
#include "Compass/QMC5883L.h"
#include "ICM42605/ICM42605.h"
#include "LetterShell/shell.h"
#include "ST7735-HAL/fonts.h"

//TODO:调整车体中心，目前还是有点歪
//lidar's offset from middle of car
#define XBIAS (10.5f)
#define YBIAS (-1.5f)

#define ToDig(rad) (rad * 57.295779513082320876798154814105f)
#define ToRad(dig) (dig * 0.01745329251994329576923690768489f)

#define TopHeight       (50)
#define Ground_Height   (1350)
#define Store_Height    (300)
#define Rotator_Height (660)
#define Stack_Height (750)

#define Ground_Angle (232)
#define Store_Angle (52)

#define FirstDegree     (145)
#define SecondDegree    (488)
#define ThirdDegree     (831)

#define CLIP_CLOSE (1000)
#define CLIP_OPEN  (830)
#define CLIP_WIDEOPEN (650)

#define IsCarStatic (!CarInfo.isCarMoving)

#define NONBLOCK 0
#define BLOCK 1

#define CMD_SCAN 0xFD
#define CMD_CAPTURE_START 0xFE
#define CMD_CAPTURE_FINISH 0xFB
#define CMD_SWITCH 0xFC

#define VecRotate(x, y, theta) do{              \
    float tx = x,ty = y;                        \
    tx = x * cosf(theta) - y * sinf(theta);     \
    ty = x * sinf(theta) + y * cosf(theta);     \
    x = tx;                                     \
    y = ty;                                     \
}while(0)

#define MAX(a, b) ((a)>(b)?(a):(b))
#define MIN(a, b) ((a)>(b)?(b):(a))

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

typedef enum command_state_t{
    STATE_IDLE,
    STATE_HEADER_LIDAR,
    STATE_HEADER_IMAGE,
    STATE_HEADER_QRCODE,
    STATE_TAIL_1,
    STATE_TAIL_2
} command_state_t;

typedef struct CarControlBlock {
    // 电机控制相关
    int16_t spd[5];
    float spdAim[5];
    Pid_t msPid[5]; // motor speed pid
    float psi[5];
    Pid_t mpPid[5]; // motor position pid
    float mpPIDout[5];
    float spdStep;
    float spdLimit[5];
    bool mPsiCtr: 1;// motor position loop enable flag
    bool firstPsiLoop: 1;

    // 整车控制相关
    PMW3901 pmw;
    float dx, dy;
    volatile float curX, curY;
    volatile float tarX, tarY;
    Pid_t cpPidX, cpPidY;// Car position pid
    int16_t errX,errY;
    float spdX, spdY;
    float psiX, psiY;
    volatile bool cPsiCtr: 1;
    bool SerialOutputEnable: 1;
    bool Pi_Reset: 1;
    bool Start_State: 1;
    volatile bool PiReceiveFlag: 1;
    volatile bool StoreCaptureFlag: 1;

    // 整车姿态相关数据
    hmcData_t hmc;
    icmData_t icm;
    float yaw;// 弧度制
    int16_t yawOverFlowTime;
    float gyroConfi;
    float opticalConfigX;
    float opticalConfigY;
    float initYawOffset;
    float initGxOffset;
    float initGyOffset;
    float initGzOffset;
    volatile uint32_t isCarMoving;// 定时器中断中监测整车是否静止，用作步骤规划

    // 状态机状态枚举
    enum mainState {// 主状态机
        mStart,
        mScan,
        mRaw,
        mRough,
        mStorage,
        mEnd
    } mainState;

    // PID姿态控制
    Pid_t avPid;// 角速度闭环
    Pid_t aPid;// 角度闭环
    float avPidOut;

    // 状态机相关函数
    void (*RunMainState)(void);
} CCB_Typedef;

extern CCB_Typedef CarInfo;
extern Shell shell;

#define LCD_EOP 0,NULL,Font_7x10,0,0

void LCD_StringLayout(uint16_t maxY, char *buff, FontDef font, uint16_t color, uint16_t bgcolor);

void SupportRotation(float dig, uint32_t time);

void SupportRotationForOS(float dig, uint32_t time);

void StoreRotation(int16_t position);

void ClipRotition(float position, uint32_t time);

void MoveTo(float X, float Y, uint8_t block);

void ClipMoveTo(int height);

void TurnTo(float rad,uint8_t block);

void Pi_ResetFromOS(void);

bool ProcessData(uint8_t c);

uint8_t Data_RoughlyEqual(double curY, double curX, double aimY, double aimX, double thre);

void MaterialGetFromHAL(int slot);

void MaterialPutFromHAL(int slot,bool stack);

void MaterialGetFromOS(int slot);

void MaterialPutFromOS(int slot,bool stack);

void StoreMaterialGetPrepareFromOS();

void StoreMaterialGetFromOS(uint8_t slot);

void Command_Send(uint8_t cmd);

#endif //CONTROLCENTER_UTILS_H
