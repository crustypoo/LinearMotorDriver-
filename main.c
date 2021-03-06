/*
 * main.c
 *
 *  Created on: May 31, 2016
 *      Author: aliu
 */

#include "src/control.h"

int main(){
	_debug = true;
	modbus_t * ctx = connect_lrd("/dev/ttyUSB1");
	uint16_t a[2];
	modbus_read_registers(ctx, 0x021C, 2, a);
	//printf("%x, %x\n", a[0], a[1]);
	//int err = set_op_param(ctx);

	//set_sys_param(ctx);

	home_stage(ctx);

	set_raster_bounds(ctx, 150, 3600);

	demo_raster(ctx);

	if (ctx != NULL){
		modbus_close(ctx);
		modbus_free(ctx);
	}
	return EXIT_SUCCESS;
}
