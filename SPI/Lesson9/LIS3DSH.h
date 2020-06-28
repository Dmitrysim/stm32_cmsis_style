#ifndef __LIS3DSH_H
#define __LIS3DSH_H

#include "global.h"

#define RD_ 0x80
#define WR_ 0x00

#define WHO_AM_I 0x0F
#define INFO1 0x0D
#define CTRL_REG4 0x20
#define ODR3 0x80
#define ODR2 0x40
#define ODR1 0x20
#define ODR0 0x10
#define BDU 0x08
#define Zen 0x04
#define Yen 0x02
#define Xen 0x01
#define CTRL_REG3 0x23
#define DR_EN 0x80
#define IEA 0x40
#define IEL 0x20
#define INT2_EN 0x10
#define INT1_EN 0x08
#define VFILT 0x04
#define STRT 0x01
#define STATUS 0x27
#define ZYXOR 0x80
#define ZOR 0x40
#define YOR 0x20
#define XOR 0x10
#define ZYXDA 0x08
#define ZDA 0x04
#define YDA 0x02
#define XDA 0x01
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D

#ifdef glob
 uint16_t Acc_X, Acc_Y, Acc_Z; // ускорения
 int16_t Acc_X_, Acc_Y_, Acc_Z_; // ускорения
 float Acc_X__, Acc_Y__, Acc_Z__; // ускорения
#else
 extern int16_t Acc_X, Acc_Y, Acc_Z; // ускорения
 extern int16_t Acc_X_, Acc_Y_, Acc_Z_; // ускорения
 extern float Acc_X__, Acc_Y__, Acc_Z__; // ускорения
#endif

#endif
