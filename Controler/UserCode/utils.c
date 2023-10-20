/**
 * @file utils.c
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2022/11/8
  */
#include "tim.h"
#include "move.h"
#include "usart.h"
#include "utils.h"
#include "printf.h"
#include <stddef.h>
#include <stdlib.h>
#include "cmsis_os2.h"
#include "LetterShell/shell.h"
#include "./ST7735-HAL/st7735.h"
#include "SerialParaChanger/SPChanger.h"
#include "LobotSerialServo/LobotSerialServo.h"



union UARTRxBuffer {
    uint16_t u16[4];
    int16_t i16[4];
} UARTRxBuffer;

command_state_t commandState = STATE_IDLE;//串口接收状态机
uint8_t data_recv = 0;

//TODO:发车测试用变量
uint8_t go_flag = 0;

int PIMaxError = 3;

#define CloseMapPID()              \
do{PID_Init(&CarInfo.cpPidX, 0, 0, 0);\
PID_Init(&CarInfo.cpPidY, 0, 0, 0);}while(0)

#define OpenMapPID()              \
do{PID_Init(&CarInfo.cpPidX, 0.8f, 0, 0);\
PID_Init(&CarInfo.cpPidY, 0.8f, 0, 0);}while(0)

static void __RunMainState(void) {
    int i,target_cnt;
    float x_record[3],y_record[3];
    static uint8_t loop_cnt = 0;

    switch (CarInfo.mainState) {
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
            SupportRotation(Ground_Angle, 500);
            ClipRotition(CLIP_OPEN, 700);
            MoveTo(65,20,BLOCK);//QRCode position
//            Command_Send(CMD_SCAN);
            osDelay(1000);//Wait for scan to finish
            CarInfo.mainState = mStorage;
            break;
        case mStorage:
            MoveTo(142,15,BLOCK);//Storage position
            CarInfo.StoreCaptureFlag = false;
            Command_Send(CMD_CAPTURE_START);
            for(i=1;i<=3;i++)
            {
                StoreMaterialGetPrepareFromOS();
                while(!CarInfo.StoreCaptureFlag);
                CarInfo.StoreCaptureFlag = false;
                StoreMaterialGetFromOS(i);
            }
            osDelay(12000);
            Command_Send(CMD_CAPTURE_FINISH);

            CarInfo.mpPid[4].ctr.aim = -10000;
            while (CarInfo.mpPid[4].ctr.aim != TopHeight);

            CarInfo.mainState = mRaw;
            break;
        case mRaw:
            MoveTo(180,20,NONBLOCK);
            while(-CarInfo.curY < 160);
            TurnTo(1.57f,NONBLOCK);

            Command_Send(CMD_CAPTURE_START);

            for(i=1;i<=3;i++) {
                MoveTo(195,115,BLOCK);
                osDelay(500);//wait for camera to switch
                CarInfo.PiReceiveFlag = 0;
                CarPositionLoopSet(0);
                CarInfo.errX = 2;
                target_cnt = 0;
                while (target_cnt<=5) {
                    if (CarInfo.PiReceiveFlag) {
                        MecanumSpeedSet(CarInfo.errX * -0.03f, CarInfo.errY * 0.03f);
                        if(abs(CarInfo.errX >= 2) || abs(CarInfo.errY >= 2))
                            target_cnt = 0;
                        else
                            target_cnt++;
                        CarInfo.PiReceiveFlag = 0;
                    }
                }

                x_record[i-1] = CarInfo.curX;
                y_record[i-1] = CarInfo.curY;

                MecanumSpeedSet(0.0f, 0.0f);
                MaterialPutFromOS(i,false);

                Command_Send(CMD_SWITCH);

                CarInfo.cpPidY.ctr.aim = CarInfo.curY;
                CarInfo.cpPidX.ctr.aim = CarInfo.curX;
                MoveTo(-CarInfo.curY,-CarInfo.curX,NONBLOCK);//Ensure the slew function to work properly

                CarPositionLoopSet(1);
            }

            Command_Send(CMD_CAPTURE_FINISH);

            for(i=1;i<=3;i++){
                MoveTo(-y_record[i-1],-x_record[i-1],BLOCK);
                MaterialGetFromOS(i);
            }

            CarInfo.mpPid[4].ctr.aim = -10000;
            while (CarInfo.mpPid[4].ctr.aim != TopHeight);
            CarInfo.mainState = mRough;
            break;
        case mRough:
            MoveTo(180,180,NONBLOCK);
            while(-CarInfo.curX < 160);
            TurnTo(3.14f,NONBLOCK);

            Command_Send(CMD_CAPTURE_START);

            for(i=1;i<=3;i++) {
                MoveTo(100,185,BLOCK);
                osDelay(500);//wait for camera to switch
                CarInfo.PiReceiveFlag = 0;
                CarPositionLoopSet(0);
                CarInfo.errX = 2;
                target_cnt = 0;
                while (target_cnt<=5) {
                    if (CarInfo.PiReceiveFlag) {
                        MecanumSpeedSet(CarInfo.errX * -0.03f, CarInfo.errY * 0.03f);
                        if(abs(CarInfo.errX >= 2) || abs(CarInfo.errY >= 2))
                            target_cnt = 0;
                        else
                            target_cnt++;
                        CarInfo.PiReceiveFlag = 0;
                    }
                }
                MecanumSpeedSet(0.0f, 0.0f);
                if(loop_cnt == 1)
                    MaterialPutFromOS(i,true);
                else
                    MaterialPutFromOS(i,false);

                Command_Send(CMD_SWITCH);

                CarInfo.cpPidY.ctr.aim = CarInfo.curY;
                CarInfo.cpPidX.ctr.aim = CarInfo.curX;
                MoveTo(-CarInfo.curY,-CarInfo.curX,NONBLOCK);//Ensure the slew function to work properly

                CarPositionLoopSet(1);
            }

            Command_Send(CMD_CAPTURE_FINISH);

            CarInfo.mpPid[4].ctr.aim = -10000;
            while (CarInfo.mpPid[4].ctr.aim != TopHeight);

            if(loop_cnt == 1) {
                MoveTo(10, 180, NONBLOCK);
                while (-CarInfo.curY > 30);
                TurnTo(-1.57f, NONBLOCK);
                MoveTo(0, 20, NONBLOCK);
                while (-CarInfo.curX > 100);
                TurnTo(0.0f, BLOCK);
                CarInfo.mainState = mEnd;
            }
            else{
                loop_cnt ++;
                MoveTo(190,180,NONBLOCK);
                while(-CarInfo.curY < 160);
                TurnTo(1.57f, NONBLOCK);
                MoveTo(190,20,NONBLOCK);
                while(-CarInfo.curX > 40);
                TurnTo(0.0f, NONBLOCK);
                CarInfo.mainState = mStorage;
            }

            break;
        case mEnd:
            MoveTo(0,0,BLOCK);
            loop_cnt = 0;
            CarInfo.mainState = mStart;
            break;
    }

}


CCB_Typedef CarInfo = {
        .gyroConfi = 0.8f,
        .opticalConfigX = 1.0f,
        .mPsiCtr = 0,
        .cPsiCtr = 1,
        .spdLimit = {50, 50, 50, 50,35},
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
    CarInfo.tarY = -X ;
    CarInfo.tarX = -Y ;
//    CarInfo.cpPidY.ctr.aim = -X ;
//    CarInfo.cpPidX.ctr.aim = -Y ;

    if(block)
        while (!Data_RoughlyEqual(CarInfo.curY, CarInfo.curX, CarInfo.tarY, CarInfo.tarX, 2) || CarInfo.isCarMoving);
}

void ClipMoveTo(int height) {
    CarInfo.mpPid[4].ctr.aim = (float) height;
    while (fabsf(CarInfo.psi[4] - CarInfo.mpPid[4].ctr.aim) >= 3);
}

void TurnTo(float rad,uint8_t block) {
    CarInfo.aPid.ctr.aim = rad;
    if(block)
        while (fabs(CarInfo.yaw - CarInfo.aPid.ctr.aim) > 0.05 || CarInfo.isCarMoving);
}

void MaterialGetFromHAL(int slot)
{
    SupportRotation(Ground_Angle, 500);
    ClipRotition(CLIP_OPEN,200);
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
    while(fabs(CarInfo.psi[4] - Ground_Height) > 5);

    ClipRotition(CLIP_CLOSE, 200);
    HAL_Delay(200);

    CarInfo.mpPid[4].ctr.aim = TopHeight;
    while(fabs(CarInfo.psi[4] - TopHeight) > 5);
    SupportRotation(Store_Angle, 200);
    HAL_Delay(1200);

    CarInfo.mpPid[4].ctr.aim = Store_Height;
    while(fabs(CarInfo.psi[4] - Store_Height) > 5);

    ClipRotition(CLIP_OPEN, 200);
    HAL_Delay(200);

    CarInfo.mpPid[4].ctr.aim = TopHeight;
    while(fabs(CarInfo.psi[4] - TopHeight) > 5);
    StoreRotation(SecondDegree);
    SupportRotation(Ground_Angle, 200);
}

void MaterialPutFromHAL(int slot,bool stack)
{
    //旋转到对应存储槽位
    CarInfo.mpPid[4].ctr.aim = TopHeight;

    ClipRotition(CLIP_OPEN,200);
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
    HAL_Delay(200);
    while(fabs(CarInfo.psi[4] - TopHeight) > 5);

    SupportRotation(Store_Angle, 300);
    HAL_Delay(1000);
    //夹子移动到存储高度
    CarInfo.mpPid[4].ctr.aim = Store_Height;
    while(fabs(CarInfo.psi[4] - Store_Height) > 5);
    //夹起物料
    ClipRotition(CLIP_CLOSE, 200);
    HAL_Delay(200);
    CarInfo.mpPid[4].ctr.aim = TopHeight;
    while(fabs(CarInfo.psi[4] - TopHeight) > 5);
    SupportRotation(Ground_Angle, 500);
    HAL_Delay(500);

    //放下物料
    CarInfo.mpPid[4].ctr.aim = Ground_Height;
    while(fabs(CarInfo.psi[4] - Ground_Height) > 5);
    ClipRotition(CLIP_OPEN, 200);
    HAL_Delay(200);

    //复位
    CarInfo.mpPid[4].ctr.aim = TopHeight;
    StoreRotation(SecondDegree);
    while(fabs(CarInfo.psi[4] - TopHeight) > 5);
}

void MaterialGetFromOS(int slot)
{
    SupportRotation(Ground_Angle, 500);
    ClipRotition(CLIP_OPEN,200);
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
    while(fabs(CarInfo.psi[4] - Ground_Height) > 5);

    ClipRotition(CLIP_CLOSE, 200);
    osDelay(200);

    CarInfo.mpPid[4].ctr.aim = TopHeight;
    while(fabs(CarInfo.psi[4] - TopHeight) > 5);
    SupportRotation(Store_Angle, 200);
    osDelay(1200);

    CarInfo.mpPid[4].ctr.aim = Store_Height;
    while(fabs(CarInfo.psi[4] - Store_Height) > 5);

    ClipRotition(CLIP_OPEN, 200);
    osDelay(200);

    CarInfo.mpPid[4].ctr.aim = TopHeight;
    while(fabs(CarInfo.psi[4] - TopHeight) > 5);
    StoreRotation(SecondDegree);
    SupportRotation(Ground_Angle, 200);
}

void MaterialPutFromOS(int slot,bool stack)
{
    //旋转到对应存储槽位
    CarInfo.mpPid[4].ctr.aim = TopHeight;

    ClipRotition(CLIP_OPEN,200);
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
    osDelay(200);
    while(fabs(CarInfo.psi[4] - TopHeight) > 5);

    SupportRotation(Store_Angle, 300);
    osDelay(1000);
    //夹子移动到存储高度
    CarInfo.mpPid[4].ctr.aim = Store_Height;
    while(fabs(CarInfo.psi[4] - Store_Height) > 5);
    //夹起物料
    ClipRotition(CLIP_CLOSE, 200);
    osDelay(200);
    CarInfo.mpPid[4].ctr.aim = TopHeight;
    while(fabs(CarInfo.psi[4] - TopHeight) > 5);
    SupportRotation(Ground_Angle, 200);

    //放下物料
    if(stack){
        osDelay(700);
        CarInfo.mpPid[4].ctr.aim = Stack_Height;
        while(fabs(CarInfo.psi[4] - Stack_Height) > 5);
    }
    else{
        osDelay(500);
        CarInfo.mpPid[4].ctr.aim = Ground_Height;
        while(fabs(CarInfo.psi[4] - Ground_Height) > 5);
    }

    ClipRotition(CLIP_OPEN, 200);
    osDelay(200);

    //复位
    CarInfo.mpPid[4].ctr.aim = TopHeight;
    StoreRotation(SecondDegree);
    while(fabs(CarInfo.psi[4] - TopHeight) > 5);
}

void StoreMaterialGetPrepareFromOS()
{
    ClipRotition(CLIP_WIDEOPEN, 700);
    CarInfo.mpPid[4].ctr.aim = Rotator_Height;
    while(CarInfo.psi[4] != Rotator_Height);
}

void StoreMaterialGetFromOS(uint8_t slot)
{
    ClipRotition(CLIP_CLOSE,700);
    osDelay(700);
    CarInfo.mpPid[4].ctr.aim = TopHeight;
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
        default:
            return;
    }
    while(CarInfo.psi[4] != TopHeight);

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

uint8_t Data_RoughlyEqual(double curY, double curX, double aimY, double aimX, double thre) {
    if ((aimY - curY) * (aimY - curY) + (aimX - curX) * (aimX - curX) < thre * thre)
        return 1;
    return 0;
}

void Command_Send(uint8_t cmd)
{
    HAL_UART_Transmit(&huart4,&cmd,1,0xFF);
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
                case 0x55:
                    current_command = STATE_HEADER_IMAGE;
                    commandState = STATE_HEADER_IMAGE;
                    data_cnt = 0;
                    break;
                case 0xFC:
                    CarInfo.StoreCaptureFlag = true;
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

                        //use first 100 data for calibration
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
                        CarInfo.errX = UARTRxBuffer.i16[0] + 12;
                        CarInfo.errY = UARTRxBuffer.i16[1] - 12;
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

