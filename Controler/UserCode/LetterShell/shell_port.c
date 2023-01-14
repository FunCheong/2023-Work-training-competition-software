/**
 * @file shell_port.c
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2023/1/6
  */
#include <stdlib.h>
#include "shell_port.h"
#include "string.h"
#include "stdio.h"
#include "printf.h"
#include "shell.h"
#include "move.h"

short uart_charPut(char *data, unsigned short len) {
    HAL_UART_Transmit(&huart5, (const uint8_t *) data, len, HAL_MAX_DELAY);
    return len;
}

uint8_t set(int argc, char *argv[]) {
    if (argc == 1) {
        printf("need an argument!\r\n");
        return 1;
    } else {
        for (int i = 1; i < argc; i++) {
            if (!strcasecmp("gp", argv[i])) { ;
            }
        }
    }
    return 0;
}

uint8_t move(int argc, char *argv[]) {
    if (argc == 1) {
        printf("need an argument!\r\n");
        return 1;
    } else {
        float a = (float) atof(argv[1]);
        float b = (float) atof(argv[2]);
        float c = (float) atof(argv[3]);

//        printf("disY = %f,disX = %f,spdLimit = %f\r\n", a, b, c);
        printf("disY = %f,spdLimit = %f\r\n", a, b);
        MecanumRotate(a, b, 0);

    }
    return 0;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN), move, move, Move the car);
