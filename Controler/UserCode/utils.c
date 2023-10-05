/**
 * @file utils.c
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2022/11/8
  */
#include "tim.h"
#include "key.h"
#include "spi.h"
#include "move.h"
#include "usart.h"
#include "utils.h"
#include "printf.h"
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include "cmsis_os2.h"
#include "CommonKey/comKey.h"
#include "LetterShell/shell.h"
#include "./Compass/QMC5883L.h"
#include "./ST7735-HAL/st7735.h"
#include "SerialParaChanger/SPChanger.h"
#include "LobotSerialServo/LobotSerialServo.h"



union UARTRxBuffer {
    uint16_t u16[4];
    int16_t i16[4];
} UARTRxBuffer;

command_state_t commandState = STATE_IDLE;//串口接收状态机
uint8_t data_recv = 0;

//TODO:原树莓派报文解包，需要修改
#define PI_uX (UARTRxBuffer.u16[1])
#define PI_uY (UARTRxBuffer.u16[2])

#define PI_X (UARTRxBuffer.i16[1])
#define PI_Y (UARTRxBuffer.i16[2])

//TODO:测试用变量
uint8_t go_flag = 0;

int PIMaxError = 3;

#define CloseMapPID()              \
do{PID_Init(&CarInfo.cpPidX, 0, 0, 0);\
PID_Init(&CarInfo.cpPidY, 0, 0, 0);}while(0)

#define OpenMapPID()              \
do{PID_Init(&CarInfo.cpPidX, 0.8f, 0, 0);\
PID_Init(&CarInfo.cpPidY, 0.8f, 0, 0);}while(0)

static void __RunMainState(void) {
    int i;
    float x_record[3],y_record[3];

    switch (CarInfo.mainState) {
        osStatus_t status;
        case mStart:
            // TODO:升降电机初始化

            //TODO:机械臂方向初始化

            //TODO:初始化上位机串口

            //TODO:等待上位机初始化完成
            if(go_flag){
                go_flag = 0;
                CarInfo.mainState = mScan;
            }
            break;
        case mScan:
            MoveTo(0,20,BLOCK);
            MoveTo(55,20,BLOCK);//QRCode position
            CarInfo.mainState = mStorage;
            break;
        case mStorage:
            MoveTo(135,20,BLOCK);//Storage position
            CarInfo.mainState = mRaw;
            break;
        case mRaw:
            MoveTo(180,20,BLOCK);
            TurnTo(1.57f,BLOCK);
            MoveTo(190,90,BLOCK);

            CarPositionLoopSet(0);
            for(i=1;i<=3;i++) {
                HAL_UART_Transmit(&huart4, (uint8_t *) "a", 1, 0xFF);
                osDelay(1000);//wait for camera to switch
                CarInfo.PiReceiveFlag = 0;
                CarPositionLoopSet(0);
                CarInfo.errX = 2;
                while (abs(CarInfo.errX >= 2) || abs(CarInfo.errY >= 2)) {
                    if (CarInfo.PiReceiveFlag) {
                        MecanumSpeedSet(CarInfo.errX * -0.03f, CarInfo.errY * 0.03f);
                        CarInfo.PiReceiveFlag = 0;
                    }
                }

                x_record[i-1] = CarInfo.curX;
                y_record[i-1] = CarInfo.curY;

                MecanumSpeedSet(0.0f, 0.0f);
                MaterialPutFromOS(i,false);
            }
            CarPositionLoopSet(1);

            for(i=1;i<=3;i++){
                MoveTo(-y_record[i-1],-x_record[i-1],BLOCK);
                MaterialGetFromOS(i);
            }
            CarInfo.mainState = mRough;
            break;
        case mRough:
            MoveTo(180,180,BLOCK);
            TurnTo(3.14f,BLOCK);
            MoveTo(90,180,BLOCK);
            CarInfo.mainState = mEnd;
            break;
        case mEnd:
            MoveTo(0,180,BLOCK);
            TurnTo(-1.57f,BLOCK);
            MoveTo(0,0,BLOCK);
            TurnTo(0.0f,BLOCK);
            CarInfo.mainState = mStart;
            break;
    }

}


CCB_Typedef CarInfo = {
        .gyroConfi = 0.8f,
        .opticalConfigX = 1.0f,
        .mPsiCtr = 0,
        .cPsiCtr = 1,
        .spdLimit = {20, 20, 20, 20, 10},
        .mainState = mStart,
        .SerialOutputEnable = 0,
        .RunMainState = __RunMainState,
};
Shell shell;

void LCD_StringLayout(uint16_t maxY, char *buff, FontDef font, uint16_t color, uint16_t bgcolor) {
    static uint16_t y = 0;
    static uint8_t fullScreenFlag = 0;
    uint16_t add = 0;

    // End of page
    if (maxY == 0 && buff == NULL && color == 0 && bgcolor == 0) {
        fullScreenFlag = add = y = 0;
//        ST7735_FillScreen(ST7735_WHITE);
        return;
    }

    if (font.data == Font_11x18.data)
        add = 18;
    else if (font.data == Font_16x26.data)
        add = 26;
    else if (font.data == Font_7x10.data)
        add = 10;

    if (y + add <= maxY) {
        ST7735_DrawString(0, y, buff, font, color, bgcolor);
        y += add;
    } else {
        if (fullScreenFlag == 0)
            ST7735_DrawString(0, maxY - add, "Full Content   ", font, color, bgcolor);
    }
}

/**
 * @brief 龙门支撑底座旋转
 * @param position 旋转位置（0-1000）
 * @param time 旋转所需时间(单位:ms)
 */
void SupportRotation(float dig, uint32_t time) {
    int16_t position = (int16_t) (dig * 77.0f / 18 + 10);
    LobotSerialServoMove(SupportServoID, (int16_t) position, time);
}

void SupportRotationForOS(float dig, uint32_t time) {
    int16_t position = (int16_t) (dig * 77.0f / 18 + 10);
    LobotSerialServoMove(SupportServoID, (int16_t) position, time);
    osDelay(time + 100);
}

void StoreRotation(int16_t position){
    LobotSerialServoMove(StoreServoID, position, 500);
}

/**
 * @brief 夹子电机旋转(夹子开关)
 * @param position 舵机位置单位(0-1000)
 * @param time 运行所需时间(单位:ms)
 */
void ClipRotition(float position, uint32_t time) {
    LobotSerialServoMove(ClipServoID, (int16_t) position, time);;
}

void ClipCloseForOS(void) {
    ClipRotition(CLIP_CLOSE, 700);
}

void ClipOpenForOS(void) {
    ClipRotition(CLIP_OPEN, 700);
}

extern char chBuff;

void MoveTo(float X, float Y,uint8_t block) {
    CarInfo.cpPidY.ctr.aim = -X ;
    CarInfo.cpPidX.ctr.aim = -Y ;

    if(block)
        while (!Data_RoughlyEqual(CarInfo.curY, CarInfo.curX, CarInfo.cpPidY.ctr.aim, CarInfo.cpPidX.ctr.aim, 2));
}

void ClipMoveTo(int height) {
    CarInfo.mpPid[4].ctr.aim = (float) height;
    while (fabsf(CarInfo.psi[4] - CarInfo.mpPid[4].ctr.aim) >= 3);
}

void TurnTo(float rad,uint8_t block) {
    CarInfo.aPid.ctr.aim = rad;
    if(block)
        while (1)if (IsCarStatic)return;
}

void MaterialGetFromHAL(int slot)
{
    SupportRotation(Ground_Angle, 500);
    ClipRotition(CLIP_OPEN,500);
    switch(slot){
        case 1:
            StoreRotation(FirstDegree);
            break;
        case 2:
            StoreRotation(SecondDegree);
            break;
        case 3:
            StoreRotation(ThirdDegree);
            break;
    }
    HAL_Delay(500);

    CarInfo.mpPid[4].ctr.aim = Ground_Height;
    while(CarInfo.psi[4] < Ground_Height);

    ClipRotition(CLIP_CLOSE, 700);
    HAL_Delay(700);

    CarInfo.mpPid[4].ctr.aim = TopHeight;
    while(CarInfo.psi[4] > TopHeight);
    SupportRotation(Store_Angle, 500);
    HAL_Delay(700);

    CarInfo.mpPid[4].ctr.aim = Store_Height;
    while(CarInfo.psi[4] != Store_Height);

    ClipRotition(CLIP_OPEN, 700);
    HAL_Delay(700);

    CarInfo.mpPid[4].ctr.aim = TopHeight;
    while(CarInfo.psi[4] > TopHeight);
    StoreRotation(SecondDegree);
    SupportRotation(Ground_Angle, 500);
}

void MaterialPutFromHAL(int slot,bool stack)
{
    //旋转到对应存储槽位
    CarInfo.mpPid[4].ctr.aim = TopHeight;

    ClipRotition(CLIP_OPEN,500);
    switch(slot){
        case 1:
            StoreRotation(FirstDegree);
            break;
        case 2:
            StoreRotation(SecondDegree);
            break;
        case 3:
            StoreRotation(ThirdDegree);
            break;
    }
    HAL_Delay(500);
    while(CarInfo.psi[4] > TopHeight);

    SupportRotation(Store_Angle, 500);
    HAL_Delay(1000);
    //夹子移动到存储高度
    CarInfo.mpPid[4].ctr.aim = Store_Height;
    while(CarInfo.psi[4] != Store_Height);
    //夹起物料
    ClipRotition(CLIP_CLOSE, 700);
    HAL_Delay(700);
    CarInfo.mpPid[4].ctr.aim = TopHeight;
    while(CarInfo.psi[4] > TopHeight);
    SupportRotation(Ground_Angle, 500);
    HAL_Delay(500);

    //放下物料
    CarInfo.mpPid[4].ctr.aim = Ground_Height;
    while(CarInfo.psi[4] != Ground_Height);
    ClipRotition(CLIP_OPEN, 700);
    HAL_Delay(700);

    //复位
    CarInfo.mpPid[4].ctr.aim = TopHeight;
    StoreRotation(SecondDegree);
    while(CarInfo.psi[4] > TopHeight);
}

void MaterialGetFromOS(int slot)
{
    SupportRotation(Ground_Angle, 500);
    ClipRotition(CLIP_OPEN,500);
    switch(slot){
        case 1:
            StoreRotation(FirstDegree);
            break;
        case 2:
            StoreRotation(SecondDegree);
            break;
        case 3:
            StoreRotation(ThirdDegree);
            break;
    }
    osDelay(500);

    CarInfo.mpPid[4].ctr.aim = Ground_Height;
    while(CarInfo.psi[4] < Ground_Height);

    ClipRotition(CLIP_CLOSE, 700);
    osDelay(700);

    CarInfo.mpPid[4].ctr.aim = TopHeight;
    while(CarInfo.psi[4] > TopHeight);
    SupportRotation(Store_Angle, 500);
    osDelay(1000);

    CarInfo.mpPid[4].ctr.aim = Store_Height;
    while(CarInfo.psi[4] != Store_Height);

    ClipRotition(CLIP_OPEN, 700);
    osDelay(700);

    CarInfo.mpPid[4].ctr.aim = TopHeight;
    while(CarInfo.psi[4] > TopHeight);
    StoreRotation(SecondDegree);
    SupportRotation(Ground_Angle, 500);
}

void MaterialPutFromOS(int slot,bool stack)
{
    //旋转到对应存储槽位
    CarInfo.mpPid[4].ctr.aim = TopHeight;

    ClipRotition(CLIP_OPEN,500);
    switch(slot){
        case 1:
            StoreRotation(FirstDegree);
            break;
        case 2:
            StoreRotation(SecondDegree);
            break;
        case 3:
            StoreRotation(ThirdDegree);
            break;
    }
    osDelay(500);
    while(CarInfo.psi[4] > TopHeight);

    SupportRotation(Store_Angle, 500);
    osDelay(1000);
    //夹子移动到存储高度
    CarInfo.mpPid[4].ctr.aim = Store_Height;
    while(CarInfo.psi[4] != Store_Height);
    //夹起物料
    ClipRotition(CLIP_CLOSE, 700);
    osDelay(700);
    CarInfo.mpPid[4].ctr.aim = TopHeight;
    while(CarInfo.psi[4] > TopHeight);    SupportRotation(Ground_Angle, 500);
    osDelay(500);

    //放下物料
    CarInfo.mpPid[4].ctr.aim = Ground_Height;
    while(CarInfo.psi[4] != Ground_Height);
    ClipRotition(CLIP_OPEN, 700);
    osDelay(700);

    //复位
    CarInfo.mpPid[4].ctr.aim = TopHeight;
    StoreRotation(SecondDegree);
    while(CarInfo.psi[4] > TopHeight);
}

void Pi_SwitchFromOS(void) {
    __disable_irq();
    HAL_GPIO_WritePin(Pi_Switch_GPIO_Port, Pi_Switch_Pin, GPIO_PIN_SET);
    osDelay(5);
    HAL_GPIO_WritePin(Pi_Switch_GPIO_Port, Pi_Switch_Pin, GPIO_PIN_RESET);
    printf("Pi_Switch\r\n");
    __enable_irq();
}

void Pi_SwitchFromHAL(void) {
    HAL_GPIO_WritePin(Pi_Switch_GPIO_Port, Pi_Switch_Pin, GPIO_PIN_SET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(Pi_Switch_GPIO_Port, Pi_Switch_Pin, GPIO_PIN_RESET);
}

void Pi_ResetFromOS(void) {
    __disable_irq();
    HAL_GPIO_WritePin(Pi_Reset_GPIO_Port, Pi_Reset_Pin, GPIO_PIN_SET);
    osDelay(5);
    HAL_GPIO_WritePin(Pi_Reset_GPIO_Port, Pi_Reset_Pin, GPIO_PIN_RESET);
    printf("Pi_Reset\r\n");
    __enable_irq();
}

void Pi_ResetFromHAL(void) {
    HAL_GPIO_WritePin(Pi_Reset_GPIO_Port, Pi_Reset_Pin, GPIO_PIN_SET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(Pi_Reset_GPIO_Port, Pi_Reset_Pin, GPIO_PIN_RESET);
}


void Data_ReFormatData(uint16_t *array, int len) {
    char record;
    uint16_t temp[len];
    int j = 0;

    if (array[0] == 0x5555) {
        return;
    } else {
        for (int i = 0;; i++) {
            if (array[i] == 0x5555) {
                record = i;
                break;
            }
        }
        for (int i = record; i < len; i++) {
            temp[j++] = array[i];
        }
        for (int i = 0; i < record; i++) {
            temp[j++] = array[i];
        }
        for (int i = 0; i < len; i++) {
            array[i] = temp[i];
        }
    }
}

uint8_t Data_RoughlyEqual(double curY, double curX, double aimY, double aimX, double thre) {
    if ((aimY - curY) * (aimY - curY) + (aimX - curX) * (aimX - curX) < thre * thre)
        return 1;
    return 0;
}

bool ProcessData(uint8_t c)
{
    float x, y, yaw;
    static float x_offset = 0, y_offset = 0, yaw_offset = 0;
    static uint8_t position_cnt = 0;
    static uint8_t data_cnt = 0;
    static command_state_t current_command = STATE_IDLE;
    switch(commandState){
        case STATE_IDLE:
            switch(c){
                case 0xAA:
                    current_command = STATE_HEADER_LIDAR;
                    commandState = STATE_HEADER_LIDAR;
                    data_cnt = 0;
                    break;
                case 0X55:
                    current_command = STATE_HEADER_IMAGE;
                    commandState = STATE_HEADER_IMAGE;
                    data_cnt = 0;
                    break;
                default:
                    current_command = STATE_IDLE;
                    return false;
            }
            break;
        case STATE_HEADER_LIDAR:
            ((uint8_t *)UARTRxBuffer.i16)[data_cnt++] = c;
            if(data_cnt == 6)
                commandState = STATE_TAIL_1;
            break;

        case STATE_HEADER_IMAGE:
            ((uint8_t *)UARTRxBuffer.i16)[data_cnt++] = c;
            if(data_cnt == 4)
                commandState = STATE_TAIL_1;
            break;

        case STATE_TAIL_1:
            if(c == 0x0A)
                commandState = STATE_TAIL_2;
            else{
                commandState = STATE_IDLE;
                return false;
            }
            break;
        case STATE_TAIL_2:
            if(c == 0x0D){
                switch(current_command){
                    case STATE_HEADER_LIDAR:
                        x = -(float)UARTRxBuffer.i16[0];
                        y = (float)UARTRxBuffer.i16[1];
                        yaw = ((float)UARTRxBuffer.i16[2])/100;

                        //TODO:做零点初始化
                        if(position_cnt <= 100){
                            x_offset += x;
                            y_offset += y;
                            yaw_offset += yaw;
                            if(position_cnt == 100){
                                x_offset /= 100;
                                y_offset /= 100;
                                yaw_offset /= 100;
                            }
                            position_cnt++;
                        }
                        else {
                            CarInfo.curX = sinf(yaw) * XBIAS + cosf(yaw) * YBIAS + x - x_offset - YBIAS;
                            CarInfo.curY = -sinf(yaw) * YBIAS + cosf(yaw) * XBIAS + y - y_offset - XBIAS;
                            CarInfo.yaw = yaw - yaw_offset;
                        }
                        break;
                    case STATE_HEADER_IMAGE:
                        CarInfo.errX = UARTRxBuffer.i16[0] + 15;
                        CarInfo.errY = UARTRxBuffer.i16[1] - 15;
                        CarInfo.PiReceiveFlag = 1;
                        break;
                    default:
                        break;
                }
                commandState = STATE_IDLE;
            }
            else{
                commandState = STATE_IDLE;
                return false;
            }
            break;
        default:
            return false;
    }
    return true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART5) {// shell用串口
        SPC_GetChar(chBuff);
        shellHandler(&shell, chBuff);
        HAL_UART_Receive_IT(huart, (uint8_t *) &chBuff, 1);
    }
    if (huart->Instance == UART4) {// 上位机通信串口
        ProcessData(data_recv);
        HAL_UART_Receive_IT(&huart4,&data_recv,1);
    }
}

