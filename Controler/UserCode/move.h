/**
 * @file move.h
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2023/1/3
  */
#ifndef CONTROLER_MOVE_H
#define CONTROLER_MOVE_H

#define MOVE_X_Grid (1330)
#define MOVE_Y_Grid (1290)

#include "stdbool.h"

void PositionLoopSet(int status);

void MecanumMove(float disY, float disX, float spdLimit, bool waitUntilStop);

void MecanumSpeedSet(float spdY, float spdX);

void MecanumRotate(float dig, float spdLimit, bool waitUntilStop);

#endif //CONTROLER_MOVE_H
