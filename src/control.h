/*
 * control.h
 *
 *  Created on: May 31, 2016
 *      Author: aliu
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<stdint.h>
#include<stdbool.h>
#include<dirent.h>
#include<math.h>
#include<string.h>
#include<endian.h>
#include<modbus/modbus.h>

// GRP A
#define GRP_A1 0
#define GRP_A2 1
#define GRP_A3 0
#define GRP_A4 0
#define GRP_A5 0

// GRP B
#define GRP_B1 5
#define GRP_B2 7
#define GRP_B3 8
#define GRP_B4 9

// GRP C
#define GRP_C1 0
#define GRP_C2 0
#define GRP_C3 0
#define GRP_C4 0
#define GRP_C5 0
#define GRP_C6 1
#define GRP_C7 0
#define GRP_C8 0
#define GRP_C9 0
#define GRP_C10 0
#define GRP_C11 0
#define GRP_C12 0
#define GRP_C13 0

// GRP D
#define GRP_D1 0
#define GRP_D2 100
#define GRP_D3 50

// GRP E
#define GRP_E1 30.000
#define GRP_E2 30.000
#define GRP_E3 100

// GRP F
#define GRP_F1 3000
#define GRP_F2 30.000
#define GRP_F3 100
#define GRP_F4 0
#define GRP_F5 0

// GRP G
#define GRP_G1 50
#define GRP_G2 30.000
#define GRP_G3 100
#define GRP_G4 0
#define GRP_G5 0
#define GRP_G6 0
#define GRP_G7 0
#define GRP_G8 500
#define GRP_G9 72

// GRP H
#define GRP_H1 285

// GRP I
#define GRP_I1 85

// GRP J
#define GRP_J1 1

// GRP K
#define GRP_K1 5000
#define GRP_K2 -5000
#define GRP_K3 0
#define GRP_K4 1

// GRP L
#define GRP_L1 1
#define GRP_L2 0

// GRP SYS
#define GRP_S1 1
#define GRP_S2 0
#define GRP_S3 0
#define GRP_S4 500
#define GRP_S5 500
#define GRP_S6 1
#define GRP_S7 1
#define GRP_S8 0
#define GRP_S9 100.0
#define GRP_S10 0
#define GRP_S11 5

modbus_t * connect_lrd(const char * addr);
int set_sys_param(modbus_t * ctx);
int set_op_param(modbus_t * ctx);
int home_stage(modbus_t * ctx);

// Library wide debug variable
bool _debug;

#endif /* CONTROL_H_ */
