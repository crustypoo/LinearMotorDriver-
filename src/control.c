/*
 * control.c
 *
 *  Created on: May 31, 2016
 *      Author: aliu
 */

#include "control.h"

// define random register ADDR
#define CMD_ONE_ADDR 0x001E
#define CMD_TWO_ADDR 0x001F
#define STAT_ONE_ADDR 0x0020
#define STAT_TWO_ADDR 0x0021
#define RST_ALM_ADDR 0x0040
#define NV_WRITE_ADDR 0x0045
#define RST_DEFAULT_ADDR 0x0046

#define PRESENT_ALM_ADDR 0x0100
#define PRESENT_WRN_ADDR 0x010B

#define CMD_POS_START_ADDR 0x0118
#define DRV_STAT_START_ADDR 0x0133
// define random register ADDR END


// define Parameter Start ADDR
/**************************************************************
 * REGISTRY GUIDE
 *
 * GRP A - start addr = 0200h
 * 	0200h = Start input mode
 * 	0201h = I/O Stop input
 * 	0202h = Stop action
 * 	0203h = Stop contact confg
 *  0204h = C-ON logic config
 *
 * GRP B - start addr = 0206h
 * 	0206h = OUT1
 * 	0207h = OUT2
 *  0208h = OUT3
 *	0209h = OUT4
 *
 * GRP C - start addr = 020Ah
 *	020Ah = HOME/ P_PRESET input switching
 *	020Bh = MOTOR EXCT MODE
 *	020Ch = HOME/FWD/RVS input mode
 *	020Dh = Data no. input mode
 *	020Eh = AWO Contact Config
 *	020Fh = Hardware Overtravel
 *	0210h = LS contact config
 *	0211h = HOMES contact config
 *	0212h = SLIT contact COnfig
 *	0213h = Overtravel Action
 *	0214h - 0215h = preset position
 *	0216h - 0217h = AREA1
 *	0218h - 0219h = AREA2
 *
 * GRP D -start addr = 021Ch
 *	021Ch - 021Dh = Encoder counter preset value
 *	021Eh = Operating Current
 *	021Fh = Standstill Current
 *
 * GRP E - start addr = 0224h
 * 	0224h - 0225h = COM Accl Rate
 *	0226h - 0227h = COM Deccel Rate
 *	0228h - 0229h = START speed
 *
 * GRP F - start addr = 0230h
 * 	0230h - 0231h = Jog operating speed
 * 	0232h - 0233h = Jog accel rate
 *	0234h - 0235h = Jog start speed
 *	0236h = Accel type
 *	0237h = Home seeking mode
 *
 * GRP G - start addr = 023Ah
 *  023Ah - 023Bh = Home seeking op-speed
 *  023Ch - 023Dh = Accel rate Home Seeking
 *  023Eh - 023Fh = Start speed of home seeking
 *  0240h - 0241h = pos offset of home seeking
 *  0242h = starting direction of home seeking
 *  0243h = SLIT detection of home seeking
 *  0244h = TIM detection of home seeking
 *  0245h = Backward steps in 2 sensor mode home seeking
 *  0246h = Stepout detection band
 *
 * GRP H - start addr = 024Ah
 * 	024Ah = Overvoltage warning
 *
 * GRP I - start addr = 024Ch
 * 	024Ch = Overheat warning
 *
 * GRP J - start addr = 0252h
 *  0252h = Software over travel
 *
 * GRP K - start addr = 0254h
 *  0254h - 0255h = Pos software limit
 *  0256h - 0257h = Neg software limit
 *  0258h = disp mode of data setter speed
 *  0259h = data setter editing mode
 *
 * GRP L - start addr = 025Bh
 *  025bh = Communication timeout action
 *  025Ch = stepout detection action
 *
 **************************************************************/
#define GRP_A_ADDR 0x0200
#define GRP_B_ADDR 0x0206
#define GRP_C_ADDR 0x020A
#define GRP_D_ADDR 0x021C
#define GRP_E_ADDR 0x0224
#define GRP_F_ADDR 0x0230
#define GRP_G_ADDR 0x023A
#define GRP_H_ADDR 0x024A
#define GRP_I_ADDR 0x024C
#define GRP_J_ADDR 0x0252
#define GRP_K_ADDR 0x0254
#define GRP_L_ADDR 0x025B
#define GRP_S_ADDR 0x030E
#define GRP_S_ADDR2 0x0310
#define GRP_S_ADDR3 0x0318
// define Parameter Start ADDR END


// define OPERATION DATA AREA
#define OP_POS_ADDR_START 0x0402
#define OP_SPPED_ADDR_START 0x0502
#define OP_POS_MODE_ADDR_START 0x0601
#define OP_SEQ_MODE_ADDR_START 0x0701
#define OP_ACCEL_RATE_ADDR_START 0x0902
#define OP_DECCEL_RATE_ADDR_START 0x0A02
#define OP_DWELL_TIME_ADDR_START 0x0C01
// define OPERATION DATA AREA END


// define MAX SIZE of messages
#define MAX_MOD_MESSAGE_SIZE 10
// define MAX SIZE of messages END

// define COM parameters
#define BAUDRATE 115200
#define PARITY 'E'
#define DATABIT 8
#define STOPBIT 1

#define TIMEOUT_SEC 0
#define TIMEOUT_USEC 500000
// define COM parameters END


// define slave axis number
#define SLV_ADDR 1
// define slave axis number END


// define prfm path
#define PRFM_PATH "/"


// typedef PARAM GRP STRUCTS
typedef struct PARAM_GROUP_A{
	uint16_t START_MODE;
	uint16_t STOP_INPUT;
	uint16_t STOP_ACT;
	uint16_t STOP_CFG;
	uint16_t C_ON_CFG;
} PARAM_GROUP_A;

typedef struct PARAM_GROUP_B{
	uint16_t OUT_1;
	uint16_t OUT_2;
	uint16_t OUT_3;
	uint16_t OUT_4;
} PARAM_GROUP_B;

typedef struct PARAM_GROUP_C{
	uint16_t HOME_SWITCH;
	uint16_t MOT_EXC_MODE;
	uint16_t HOME_INPUT;
	uint16_t DATA_NO_INPUT;
	uint16_t AWO;
	uint16_t HRDWR_OVERTRAVEL;
	uint16_t LS_CFG;
	uint16_t HOMES_CON_CFG;
	uint16_t SLIT_CONTACT;
	uint16_t OVERTRAVEL_ACT;
	int PRESET_POS;
	int AREA1;
	int AREA2;
} PARAM_GROUP_C;

typedef struct PARAM_GROUP_D{
	int ENC_CNT_PRESET;
	uint16_t OP_CURRENT;
	uint16_t STBY_CURRENT;
} PARAM_GROUP_D;

typedef struct PARAM_GROUP_E{
	uint32_t COMMON_ACCEL_RATE;
	uint32_t COMMON_DECCEL_RATE;
	uint32_t STARTING_SPD;
} PARAM_GROUP_E;

typedef struct PARAM_GROUP_F {
	uint32_t JOG_SPD;
	uint32_t JOG_ACCEL_RATE;
	uint32_t JOG_START_SPD;
	uint16_t ACCEL_RATE_TYPE;
	uint16_t HOME_MODE;
} PARAM_GROUP_F;

typedef struct PARAM_GROUP_G{
	uint32_t OP_SPD_HOME;
	uint32_t ACCEL_RATE_HOME;
	int START_SPD_HOME;
	int POS_OFFSET_HOME;
	uint16_t START_DIR_HOME;
	uint16_t SLIT_DETECT;
	uint16_t TIM_DETECT;
	uint16_t BACKWARD_STPS;
	uint16_t STEPOUT_DETECT;
} PARAM_GROUP_G;

typedef struct PARAM_GROUP_H{
	uint16_t OVER_VOLTAGE;
} PARAM_GROUP_H;

typedef struct PARAM_GROUP_I{
	uint16_t OVER_HEAT;
} PARAM_GROUP_I;

typedef struct PARAM_GROUP_J{
	uint16_t SFTWR_OVERTRAVEL;
} PARAM_GROUP_J;

typedef struct PARAM_GROUP_K{
	int POS_S_LIMIT;
	int NEG_S_LIMIT;
	uint16_t DISPLAY_MODE;
	uint16_t DATA_SET;
} PARAM_GROUP_K;

typedef struct PARAM_GROUP_L{
	uint16_t COM_TIMEOUT_ACT;
	uint16_t STEPOUT_DETECT_ACT;
} PARAM_GROUP_L;


typedef struct PARAM_GROUP_SYS{
	uint16_t MOT_ROT_DIR;
	uint16_t STEPOUT_DIR;
	uint16_t MOTOR_STEP_ANGLE;
	uint32_t ELEC_GEAR_A;
	uint32_t ELEC_GEAR_B;
	uint16_t COM_AXIS;
	uint16_t _PARITY;
	uint16_t _STOP_BIT;
	uint16_t WAITING_TIME;
	uint16_t COM_TIMEOUT;
	uint16_t COM_ERR_ALM;
}PARAM_GROUP_SYS;

// typedef PARAM GRP STRUCTS END

// '*.h'exposed functions
modbus_t * connect_lrd(const char * addr);
int set_sys_param(modbus_t * ctx);
int set_op_param(modbus_t * ctx);
int home_stage(modbus_t * ctx);

// '*.c' only functions
static int _set_grp_a(modbus_t * ctx);
static int _set_grp_b(modbus_t * ctx);
static int _set_grp_c(modbus_t * ctx);
static int _set_grp_d(modbus_t * ctx);
static int _set_grp_e(modbus_t * ctx);
static int _set_grp_f(modbus_t * ctx);
static int _set_grp_g(modbus_t * ctx);
static int _set_grp_h(modbus_t * ctx);
static int _set_grp_i(modbus_t * ctx);
static int _set_grp_j(modbus_t * ctx);
static int _set_grp_k(modbus_t * ctx);
static int _set_grp_l(modbus_t * ctx);
static uint32_t _swap_2byte_order(uint32_t input);

modbus_t * connect_lrd(const char * addr){
	modbus_t * ctx = modbus_new_rtu(addr, BAUDRATE, PARITY, DATABIT, STOPBIT);
	if (ctx == NULL){
		if (_debug)
			printf(" >> ERROR in 'connect_lrd()' :: 'modbus_new_rtu()' :: Failed to create modbus context!\n");
		modbus_free(ctx);
		return NULL;
	}

	if (_debug)
		modbus_set_debug(ctx,1);

	int err = modbus_set_slave(ctx, SLV_ADDR);
	if (err == -1){
		if (_debug)
			printf(" >> ERROR in 'connect_lrd()' :: 'modbus_set_slave()' :: Undefined slave addr!\n");
		modbus_free(ctx);
		return NULL;
	}

	struct timeval timevalues;
	modbus_get_response_timeout(ctx, &timevalues);

	if ((int)timevalues.tv_sec != TIMEOUT_SEC || (int)timevalues.tv_usec != TIMEOUT_USEC){
		timevalues.tv_usec = TIMEOUT_USEC;
		timevalues.tv_sec = TIMEOUT_SEC;

		modbus_set_response_timeout(ctx, &timevalues);
	}

	modbus_flush(ctx);
	err = modbus_connect(ctx);
	if (err < 0){
		if (_debug)
			printf(" >> ERROR in 'connect_lrd()' :: 'modbus_connect()' :: Connection failed!\n");
		modbus_free(ctx);
		return NULL;
	}

	return ctx;
}

int set_sys_param(modbus_t * ctx){
	int _size = (int)(sizeof(PARAM_GROUP_SYS) / 2);

	union{
		uint16_t _pg_a[_size];
		PARAM_GROUP_SYS pgs;
	} _container;

	_container.pgs.MOT_ROT_DIR = GRP_S1;
	_container.pgs.STEPOUT_DIR = GRP_S2;
	_container.pgs.MOTOR_STEP_ANGLE = GRP_S3;
	_container.pgs.ELEC_GEAR_A = _swap_2byte_order(GRP_S4);
	_container.pgs.ELEC_GEAR_B = _swap_2byte_order(GRP_S5);
	_container.pgs.COM_AXIS = GRP_S6;
	_container.pgs._PARITY = GRP_S7;
	_container.pgs._STOP_BIT = GRP_S8;
	_container.pgs.WAITING_TIME = GRP_S9;
	_container.pgs.COM_TIMEOUT = GRP_S10;
	_container.pgs.COM_ERR_ALM = GRP_S11;

	uint16_t _buff[_size];
	bool _write_params = false;

	int err = modbus_read_registers(ctx, GRP_S_ADDR, 1, _buff[0]);
	if (err < 0){
		if (_debug){
			printf(" >> ERROR in 'set_sys_param()' :: 'modbus_read_register()' :: READ_ FAILURE\n");
			printf
		}
	}

	int i = 0;
	for(i = 0; i < _size; i++){
		uint16_t _not_same = _buff[i] ^ _container._pg_a[i];
		if (_not_same > 0)
			_write_params = true;
		printf("%d, %d\n", _buff[i], _container._pg_a[i]);
	}

	if (_write_params){
		if (_debug)
			printf(" >> No match found through read, writing correct sys params...\n");
		err = modbus_write_register(ctx, GRP_S_ADDR, &(_container._pg_a[0]));
		if (err < 0){
			if (_debug)
				printf(" >> ERROR in 'set_sys_param()' :: 'modbus_write_register' :: Could not set Motor Rotation Direction\n");
			return -1;
		}

		err = modbus_write_registers(ctx, GRP_S_ADDR2, MAX_MOD_MESSAGE_SIZE, &_container._pg_a[1]);
		if (err < 0){
			if (_debug)
				printf(" >> ERROR in 'set_sys_param()' :: 'modbus_write_registers()' :: Could not set sys params\n");
			return -1;
		} else if (err > 0 && err < MAX_MOD_MESSAGE_SIZE){
			if (_debug)
				printf(" >> ERROR in 'set_sys_param()' :: 'modbus_write_register()' :: RESOLVING MISSED WRITES\n");

			uint16_t _offset_addr = GRP_S_ADDR2 + err;
			err = modbus_write_registers(ctx, _offset_addr, MAX_MOD_MESSAGE_SIZE - err, &_container._pg_a[1 + err]);
			if (err < 0){
				if (_debug)
					printf(" >> ERROR in 'set_sys_param()' :: 'modbus_write_register()' :: UNRESOLVED WRITES\n");
				return -1;
			}
		}
	}
	return 0;
}

int set_op_param(modbus_t * ctx){
	int issue_track = 0;

	if (_set_grp_a(ctx) < 0)
		issue_track = 1;
	if (_set_grp_b(ctx) < 0)
		issue_track = 2;
	if (_set_grp_c(ctx) < 0)
		issue_track = 3;
	if (_set_grp_d(ctx) < 0)
		issue_track = 4;
	if (_set_grp_e(ctx) < 0)
		issue_track = 5;
	if (_set_grp_f(ctx) < 0)
		issue_track = 6;
	if (_set_grp_g(ctx) < 0)
		issue_track = 7;
	if (_set_grp_h(ctx) < 0)
		issue_track = 8;
	if (_set_grp_i(ctx) < 0)
		issue_track = 9;
	if (_set_grp_j(ctx) < 0)
		issue_track = 10;
	if (_set_grp_k(ctx) < 0)
		issue_track = 11;
	if (_set_grp_l(ctx) < 0)
		issue_track = 12;

	if (issue_track > 0){
		char _ascii_code = ((issue_track - 1) + 65);
		if (_debug)
			printf(" >> ERROR in 'set_lrd_driver()' :: Problem in setting PARAM GRP %c\n", _ascii_code);
		return -1;
	}

	return 0;
}

int home_stage(modbus_t * ctx){
	const uint16_t _home_c = 0b00001000;
	const uint16_t _c_on_c = 0b00100000;
	const uint16_t _stop_c = 0b00010000;

	uint16_t _cmd = _c_on_c << 8;
	int err = modbus_write_register(ctx, CMD_ONE_ADDR, _cmd);
	if (err < 0){
		if (_debug ) {
			printf(" >> ERROR in 'home_stage()' :: 'modbus_write_register' :: Initial motor excitation via RS485 failed\n");
			printf(" >> HOMING OPERATION FAILED!\n");
		}
		return -1;
	}

	_cmd = (_c_on_c | _home_c);
	_cmd = _cmd << 8;
	err = modbus_write_register(ctx, CMD_ONE_ADDR, _cmd);
	if (err < 0){
		if (_debug) {
			printf(" >> ERROR in 'home_stage()' :: 'modbus_write_register' :: Home cmd via RS485 was lost\n");
			printf(" >> HOMING OPERATION FAILED!\n");
		}
		return -1;
	}

	_cmd = _c_on_c << 8;
	err = modbus_write_register(ctx, CMD_ONE_ADDR, _cmd);
	if (err < 0){
		if (_debug) {
			printf(" >> ERROR in 'home_stage()' :: 'modbus_write_register' :: Homing stop via RS485 failed\n");
			printf(" >> STOPPING MOTOR...\n");
		}
		_cmd = _stop_c << 8;
		err = modbus_write_register(ctx, CMD_ONE_ADDR, _cmd);
		if (err < 0){
			if (_debug)
				printf(" >> Cannot communicate with device, stop servo manually!\n");
		}

		return -1;
	}

	return 0;
}



static int _set_grp_a(modbus_t * ctx){
	if (_debug)
		printf(" >> SETTING GRP A VALUES\n");

	PARAM_GROUP_A * pga = malloc(sizeof(PARAM_GROUP_A));
	if (pga == NULL){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_a()' :: 'malloc()' :: FAILED TO ALLOCATE MEMORY FOR PGA\n");
		return -1;
	}

	pga->START_MODE = GRP_A1;
	pga->STOP_INPUT = GRP_A2;
	pga->STOP_ACT = GRP_A3;
	pga->STOP_CFG = GRP_A4;
	pga->C_ON_CFG = GRP_A5;

	int _size = (int)(sizeof(PARAM_GROUP_A) / 2);

	union {
		uint16_t _pg_a[_size];
		PARAM_GROUP_A pg_s;
	} _container;

	_container.pg_s = *pga;

	int err = modbus_write_registers(ctx, GRP_A_ADDR, _size, &_container._pg_a[0]);
	if (err < 0){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_a()' :: 'modbus_write_registers()' :: UNABLE TO WRITE TO CTX\n");
		return -1;
	} else if (err > 0 && err < _size){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_a()' :: 'modbus_write_registers()' :: RESOLVING MISSED WRITES\n");
		uint16_t _offset_addr = GRP_A_ADDR + err;
		err = modbus_write_registers(ctx, _offset_addr, _size - err, &_container._pg_a[err]);
		if (err < _size - err){
			if (_debug)
				printf(" >>> ERROR in '_set_grp_a()' :: 'modbus_write_registers()' :: UNRESOLVED MISSED WRITES\n");
			return -1;
		}

	}

	if (_debug)
		printf(" >> GRP A VALUES HAVE BEEN SET\n");

	free(pga);
	return 0;
}

static int _set_grp_b(modbus_t * ctx){
	if (_debug)
		printf(" >> SETTING GRP B VALUES\n");

	PARAM_GROUP_B * pgb = malloc(sizeof(PARAM_GROUP_B));
	if (pgb == NULL){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_b()' :: 'malloc()' :: FAILED TO ALLOCATE MEMORY FOR PGB\n");
		return -1;
	}

	pgb->OUT_1 = GRP_B1;
	pgb->OUT_2 = GRP_B2;
	pgb->OUT_3 = GRP_B3;
	pgb->OUT_4 = GRP_B4;

	int _size = (int)(sizeof(PARAM_GROUP_B) / 2);

	union {
		uint16_t _pg_a[_size];
		PARAM_GROUP_B pg_s;
	} _container;

	_container.pg_s = *pgb;

	int err = modbus_write_registers(ctx, GRP_B_ADDR, _size, &_container._pg_a[0]);
	if (err < 0){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_b()' :: 'modbus_write_registers()' :: UNABLE TO WRITE TO CTX\n");
		return -1;
	} else if (err > 0 && err < _size){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_b()' :: 'modbus_write_registers()' :: RESOLVING MISSED WRITES\n");
		uint16_t _offset_addr = GRP_B_ADDR + err;
		err = modbus_write_registers(ctx, _offset_addr, _size - err, &_container._pg_a[err]);
		if (err < _size - err){
			if (_debug)
				printf(" >>> ERROR in '_set_grp_b()' :: 'modbus_write_registers()' :: UNRESOLVED MISSED WRITES\n");
			return -1;
		}

	}

	if (_debug)
		printf(" >> GRP B VALUES HAVE BEEN SET\n");

	free(pgb);
	return 0;
}

static int _set_grp_c(modbus_t * ctx){
	if (_debug)
		printf(" >> SETTING GRP C VALUES\n");

	PARAM_GROUP_C * pgc = malloc(sizeof(PARAM_GROUP_C));
	if (pgc == NULL){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_c()' :: 'malloc()' :: FAILED TO ALLOCATE MEMORY FOR PGB\n");
		return -1;
	}
	pgc->HOME_SWITCH = GRP_C1;
	pgc->MOT_EXC_MODE = GRP_C2;
	pgc->HOME_INPUT = GRP_C3;
	pgc->DATA_NO_INPUT = GRP_C4;
	pgc->AWO = GRP_C5;
	pgc->HRDWR_OVERTRAVEL = GRP_C6;
	pgc->LS_CFG = GRP_C7;
	pgc->HOMES_CON_CFG = GRP_C8;
	pgc->SLIT_CONTACT = GRP_C9;
	pgc->OVERTRAVEL_ACT = GRP_C10;
	pgc->PRESET_POS = _swap_2byte_order(GRP_C11);
	pgc->AREA1 = _swap_2byte_order(GRP_C12);
	pgc->AREA2 = _swap_2byte_order(GRP_C13);

	int _size = (int)(sizeof(PARAM_GROUP_C) / 2);

	union {
		uint16_t _pg_a[_size];
		PARAM_GROUP_C pg_s;
	} _container;

	_container.pg_s = *pgc;

	int err = modbus_write_registers(ctx, GRP_C_ADDR, MAX_MOD_MESSAGE_SIZE, &_container._pg_a[0]);
	if (err < 0){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_c()' :: 'modbus_write_registers()' :: UNABLE TO WRITE TO CTX\n");
		return -1;
	} else if (err > 0 && err < MAX_MOD_MESSAGE_SIZE){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_c()' :: 'modbus_write_registers()' :: RESOLVING MISSED WRITES\n");
		uint16_t _offset_addr = GRP_C_ADDR + err;
		err = modbus_write_registers(ctx, _offset_addr, MAX_MOD_MESSAGE_SIZE - err, &_container._pg_a[err]);
		if (err < MAX_MOD_MESSAGE_SIZE - err){
			if (_debug)
				printf(" >>> ERROR in '_set_grp_c()' :: 'modbus_write_registers()' :: UNRESOLVED MISSED WRITES\n");
			return -1;
		}

	}

	err = modbus_write_registers(ctx, GRP_C_ADDR + MAX_MOD_MESSAGE_SIZE, _size - MAX_MOD_MESSAGE_SIZE, &_container._pg_a[MAX_MOD_MESSAGE_SIZE]);
	if (err < 0){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_c()' :: 'modbus_write_registers()' :: UNABLE TO WRITE TO CTX\n");
		return -1;
	} else if (err > 0 && err < _size - MAX_MOD_MESSAGE_SIZE){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_c()' :: 'modbus_write_registers()' :: RESOLVING MISSED WRITES\n");
		uint16_t _offset_addr = GRP_C_ADDR + MAX_MOD_MESSAGE_SIZE + err;
		err = modbus_write_registers(ctx, _offset_addr, _size - MAX_MOD_MESSAGE_SIZE - err, &_container._pg_a[MAX_MOD_MESSAGE_SIZE + err]);
		if (err < _size - MAX_MOD_MESSAGE_SIZE - err){
			if (_debug)
				printf(" >>> ERROR in '_set_grp_c()' :: 'modbus_write_registers()' :: UNRESOLVED MISSED WRITES\n");
			return -1;
		}

	}

	if (_debug)
		printf(" >> GRP C VALUES HAVE BEEN SET\n");

	free(pgc);
	return 0;
}


static int _set_grp_d(modbus_t * ctx){
	if (_debug)
		printf(" >> SETTING GRP D VALUES\n");

	PARAM_GROUP_D * pgd = malloc(sizeof(PARAM_GROUP_D));
	if (pgd == NULL){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_d()' :: 'malloc()' :: FAILED TO ALLOCATE MEMORY FOR PGB\n");
		return -1;
	}

	pgd->ENC_CNT_PRESET = _swap_2byte_order(GRP_D1);
	pgd->OP_CURRENT = GRP_D2;
	pgd->STBY_CURRENT = GRP_D3;

	int _size = (int)(sizeof(PARAM_GROUP_D) / 2);

	union {
		uint16_t _pg_a[_size];
		PARAM_GROUP_D pg_s;
	} _container;

	_container.pg_s = *pgd;

	int err = modbus_write_registers(ctx, GRP_D_ADDR, _size, &_container._pg_a[0]);
	if (err < 0){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_d()' :: 'modbus_write_registers()' :: UNABLE TO WRITE TO CTX\n");
		return -1;
	} else if (err > 0 && err < _size){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_d()' :: 'modbus_write_registers()' :: RESOLVING MISSED WRITES\n");
		uint16_t _offset_addr = GRP_D_ADDR + err;
		err = modbus_write_registers(ctx, _offset_addr, _size - err, &_container._pg_a[err]);
		if (err < _size - err){
			if (_debug)
				printf(" >>> ERROR in '_set_grp_d()' :: 'modbus_write_registers()' :: UNRESOLVED MISSED WRITES\n");
			return -1;
		}

	}

	if (_debug)
		printf(" >> GRP D VALUES HAVE BEEN SET\n");

	free(pgd);
	return 0;
}

static int _set_grp_e(modbus_t * ctx){
	if (_debug)
		printf(" >> SETTING GRP E VALUES\n");

	PARAM_GROUP_E * pge = malloc(sizeof(PARAM_GROUP_E));
	if (pge == NULL){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_e()' :: 'malloc()' :: FAILED TO ALLOCATE MEMORY FOR PGE\n");
		return -1;
	}

	pge->COMMON_ACCEL_RATE = _swap_2byte_order(GRP_E1 * 1000);
	pge->COMMON_DECCEL_RATE = _swap_2byte_order(GRP_E2 * 1000);
	pge->STARTING_SPD = _swap_2byte_order(GRP_E3);

	int _size = (int)(sizeof(PARAM_GROUP_E) / 2);

	union {
		uint16_t _pg_a[_size];
		PARAM_GROUP_E pg_s;
	} _container;

	_container.pg_s = *pge;

	int err = modbus_write_registers(ctx, GRP_E_ADDR, _size, &_container._pg_a[0]);
	if (err < 0){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_e()' :: 'modbus_write_registers()' :: UNABLE TO WRITE TO CTX\n");
		return -1;
	} else if (err > 0 && err < _size){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_e()' :: 'modbus_write_registers()' :: RESOLVING MISSED WRITES\n");
		uint16_t _offset_addr = GRP_E_ADDR + err;
		err = modbus_write_registers(ctx, _offset_addr, _size - err, &_container._pg_a[err]);
		if (err < _size - err){
			if (_debug)
				printf(" >>> ERROR in '_set_grp_e()' :: 'modbus_write_registers()' :: UNRESOLVED MISSED WRITES\n");
			return -1;
		}
	}

	if (_debug)
		printf(" >> GRP E VALUES HAVE BEEN SET\n");

	free(pge);
	return 0;
}

static int _set_grp_f(modbus_t * ctx){
	if (_debug)
		printf(" >> SETTING GRP F VALUES\n");

	PARAM_GROUP_F * pgf = malloc(sizeof(PARAM_GROUP_F));
	if (pgf == NULL){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_f()' :: 'malloc()' :: FAILED TO ALLOCATE MEMORY FOR PGF\n");
		return -1;
	}

	pgf->JOG_SPD = _swap_2byte_order(GRP_F1);
	pgf->JOG_ACCEL_RATE = _swap_2byte_order(GRP_F2 * 1000);
	pgf->JOG_START_SPD = _swap_2byte_order(GRP_F3);
	pgf->ACCEL_RATE_TYPE = GRP_F4;
	pgf->HOME_MODE = GRP_F5;

	int _size = (int)(sizeof(PARAM_GROUP_F) / 2);

	union {
		uint16_t _pg_a[_size];
		PARAM_GROUP_F pg_s;
	} _container;

	_container.pg_s = *pgf;

	int err = modbus_write_registers(ctx, GRP_F_ADDR, _size, &_container._pg_a[0]);
	if (err < 0){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_f()' :: 'modbus_write_registers()' :: UNABLE TO WRITE TO CTX\n");
		return -1;
	} else if (err > 0 && err < _size){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_f()' :: 'modbus_write_registers()' :: RESOLVING MISSED WRITES\n");
		uint16_t _offset_addr = GRP_F_ADDR + err;
		err = modbus_write_registers(ctx, _offset_addr, _size - err, &_container._pg_a[err]);
		if (err < _size - err){
			if (_debug)
				printf(" >>> ERROR in '_set_grp_f()' :: 'modbus_write_registers()' :: UNRESOLVED MISSED WRITES\n");
			return -1;
		}
	}

	if (_debug)
		printf(" >> GRP F VALUES HAVE BEEN SET\n");

	free(pgf);
	return 0;
}

static int _set_grp_g(modbus_t * ctx){
	if (_debug)
		printf(" >> SETTING GROUP G VALUES\n");

	PARAM_GROUP_G * pgg = malloc(sizeof(PARAM_GROUP_G));
	if (pgg == NULL){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_g()' :: 'malloc()' :: FAILED TO ALLOCATE MEMORY FOR PGG\n");
		return -1;
	}

	pgg->OP_SPD_HOME = _swap_2byte_order(GRP_G1);
	pgg->ACCEL_RATE_HOME = _swap_2byte_order(GRP_G2 * 1000);
	pgg->START_SPD_HOME = _swap_2byte_order(GRP_G3);
	pgg->POS_OFFSET_HOME = _swap_2byte_order(GRP_G4);
	pgg->START_DIR_HOME = GRP_G5;
	pgg->SLIT_DETECT = GRP_G6;
	pgg->TIM_DETECT = GRP_G7;
	pgg->BACKWARD_STPS = GRP_G8;
	pgg->STEPOUT_DETECT = GRP_G9;

	int _size = (int)(sizeof(PARAM_GROUP_G) / 2);

	union {
		uint16_t _pg_a[_size];
		PARAM_GROUP_G pg_s;
	} _container;

	_container.pg_s = *pgg;

	int err = modbus_write_registers(ctx, GRP_G_ADDR, MAX_MOD_MESSAGE_SIZE, &_container._pg_a[0]);
	if (err < 0){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_g()' :: 'modbus_write_registers()' :: UNABLE TO WRITE TO CTX\n");
		return -1;
	} else if (err > 0 && err < MAX_MOD_MESSAGE_SIZE){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_g()' :: 'modbus_write_registers()' :: RESOLVING MISSED WRITES\n");
		uint16_t _offset_addr = GRP_G_ADDR + err;
		err = modbus_write_registers(ctx, _offset_addr, MAX_MOD_MESSAGE_SIZE - err, &_container._pg_a[err]);
		if (err < MAX_MOD_MESSAGE_SIZE - err){
			if (_debug)
				printf(" >>> ERROR in '_set_grp_g()' :: 'modbus_write_registers()' :: UNRESOLVED MISSED WRITES\n");
			return -1;
		}

	}

	err = modbus_write_registers(ctx, GRP_G_ADDR + MAX_MOD_MESSAGE_SIZE, _size - MAX_MOD_MESSAGE_SIZE, &_container._pg_a[MAX_MOD_MESSAGE_SIZE]);
	if (err < 0){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_g()' :: 'modbus_write_registers()' :: UNABLE TO WRITE TO CTX\n");
		return -1;
	} else if (err > 0 && err < _size - MAX_MOD_MESSAGE_SIZE){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_g()' :: 'modbus_write_registers()' :: RESOLVING MISSED WRITES\n");
		uint16_t _offset_addr = GRP_G_ADDR + MAX_MOD_MESSAGE_SIZE + err;
		err = modbus_write_registers(ctx, _offset_addr, _size - MAX_MOD_MESSAGE_SIZE - err, &_container._pg_a[MAX_MOD_MESSAGE_SIZE + err]);
		if (err < _size - MAX_MOD_MESSAGE_SIZE - err){
			if (_debug)
				printf(" >>> ERROR in '_set_grp_g()' :: 'modbus_write_registers()' :: UNRESOLVED MISSED WRITES\n");
			return -1;
		}

	}

	if (_debug)
		printf(" >> GRP G VALUES HAVE BEEN SET\n");

	free(pgg);
	return 0;
}


static int _set_grp_h(modbus_t * ctx){
	if (_debug)
		printf(" >> SETTING GROUP H VALUES\n");

	int _size = (int)(sizeof(PARAM_GROUP_H) / 2);

	union {
		uint16_t _pg_a[_size];
		PARAM_GROUP_H pg_s;
	} _container;

	_container.pg_s.OVER_VOLTAGE = GRP_H1;

	int err = modbus_write_register(ctx, GRP_H_ADDR, _container._pg_a[0]);
	if (err < 0){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_h()' :: 'modbus_write_register()' :: UNABLE TO WRITE TO CTX\n");
		return -1;
	}

	if (_debug)
		printf(" >> GRP H VALUES HAVE BEEN SET\n");

	return 0;
}

static int _set_grp_i(modbus_t * ctx){
	if (_debug)
		printf(" >> SETTING GROUP I VALUES\n");

	int _size = (int)(sizeof(PARAM_GROUP_I) / 2);

	union {
		uint16_t _pg_a[_size];
		PARAM_GROUP_I pg_s;
	} _container;

	_container.pg_s.OVER_HEAT = GRP_I1;

	int err = modbus_write_register(ctx, GRP_I_ADDR, _container._pg_a[0]);
	if (err < 0){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_i()' :: 'modbus_write_register()' :: UNABLE TO WRITE TO CTX\n");
		return -1;
	}

	if (_debug)
		printf(" >> GRP I VALUES HAVE BEEN SET\n");

	return 0;
}

static int _set_grp_j(modbus_t * ctx){
	if (_debug)
		printf(" >> SETTING GROUP J VALUES\n");

	int _size = (int)(sizeof(PARAM_GROUP_I) / 2);

	union {
		uint16_t _pg_a[_size];
		PARAM_GROUP_J pg_s;
	} _container;

	_container.pg_s.SFTWR_OVERTRAVEL = GRP_J1;

	int err = modbus_write_register(ctx, GRP_J_ADDR, _container._pg_a[0]);
	if (err < 0){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_j()' :: 'modbus_write_register()' :: UNABLE TO WRITE TO CTX\n");
		return -1;
	}

	if (_debug)
		printf(" >> GRP J VALUES HAVE BEEN SET\n");

	return 0;
}

static int _set_grp_k(modbus_t * ctx){
	if (_debug)
		printf(" >> SETTING GROUP K VALUES\n");

	int _size = (int)(sizeof(PARAM_GROUP_K) / 2);

	union {
		uint16_t _pg_a[_size];
		PARAM_GROUP_K pg_s;
	} _container;

	_container.pg_s.POS_S_LIMIT = _swap_2byte_order(GRP_K1);
	_container.pg_s.NEG_S_LIMIT = _swap_2byte_order(GRP_K2);
	_container.pg_s.DISPLAY_MODE = GRP_K3;
	_container.pg_s.DATA_SET = GRP_K4;

	int err = modbus_write_registers(ctx, GRP_K_ADDR, _size, &_container._pg_a[0]);
	if (err < 0){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_k()' :: 'modbus_write_registers()' :: UNABLE TO WRITE TO CTX\n");
		return -1;
	} else if (err > 0 && err < _size){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_k()' :: 'modbus_write_registers()' :: RESOLVING MISSED WRITES\n");
		uint16_t _offset_addr = GRP_K_ADDR + err;
		err = modbus_write_registers(ctx, _offset_addr, _size - err, &_container._pg_a[err]);
		if (err < _size - err){
			if (_debug)
				printf(" >>> ERROR in '_set_grp_k()' :: 'modbus_write_registers()' :: UNRESOLVED MISSED WRITES\n");
			return -1;
		}
	}

	if (_debug)
		printf(" >> GRP K VALUES HAVE BEEN SET\n");

	return 0;
}


static int _set_grp_l(modbus_t * ctx){
	if (_debug)
		printf(" >> SETTING GROUP L VALUES\n");

	int _size = (int)(sizeof(PARAM_GROUP_L) / 2);

	union {
		uint16_t _pg_a[_size];
		PARAM_GROUP_L pg_s;
	} _container;

	_container.pg_s.COM_TIMEOUT_ACT = GRP_L1;
	_container.pg_s.STEPOUT_DETECT_ACT = GRP_L2;

	int err = modbus_write_registers(ctx, GRP_L_ADDR, _size, &_container._pg_a[0]);
	if (err < 0){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_l()' :: 'modbus_write_registers()' :: UNABLE TO WRITE TO CTX\n");
		return -1;
	} else if (err > 0 && err < _size){
		if (_debug)
			printf(" >>> ERROR in '_set_grp_l()' :: 'modbus_write_registers()' :: RESOLVING MISSED WRITES\n");
		uint16_t _offset_addr = GRP_L_ADDR + err;
		err = modbus_write_registers(ctx, _offset_addr, _size - err, &_container._pg_a[err]);
		if (err < _size - err){
			if (_debug)
				printf(" >>> ERROR in '_set_grp_l()' :: 'modbus_write_registers()' :: UNRESOLVED MISSED WRITES\n");
			return -1;
		}
	}

	if (_debug)
		printf(" >> GRP L VALUES HAVE BEEN SET\n");

	return 0;
}


static uint32_t _swap_2byte_order(uint32_t input){
	uint32_t output = input;

	output = ( output >> 16 ) | (input << 16);

	return output;
}





























