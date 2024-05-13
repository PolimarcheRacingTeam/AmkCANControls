/*
 * amkfse.h
 *
 *  Created on: 20 feb 2022
 *      Author: Giovanni Latiano
 */

#ifndef INC_AMKFSE_H_
#define INC_AMKFSE_H_

#define AMK_ACTUAL_VALUES_1_BASE_ADDRESS  	0x282   			//inverter -> CAN controller
#define AMK_ACTUAL_VALUES_2_BASE_ADDRESS  	0x284				//inverter -> CAN controller
#define AMK_SETPOINTS_1_BASE_ADDRESS  		0x183				//CAN controller -> inverter

#define AMK_INVERTER_1_NODE_ADDRESS 			1				//inverter 1
#define AMK_INVERTER_2_NODE_ADDRESS 			2				//inverter 2
#define AMK_INVERTER_3_NODE_ADDRESS 			5				//inverter 3
#define AMK_INVERTER_4_NODE_ADDRESS 			6				//inverter 4

/*INVERTER1*/
#define AMK_INVERTER_1_ACTUAL_VALUES_1 		0x283
#define AMK_INVERTER_1_ACTUAL_VALUES_2		0x285
#define AMK_INVERTER_1_SETPOINTS_1			0x184

/*INVERTER2*/
#define AMK_INVERTER_2_ACTUAL_VALUES_1 		0x284
#define AMK_INVERTER_2_ACTUAL_VALUES_2		0x286
#define AMK_INVERTER_2_SETPOINTS_1			0x185

/*INVERTER3*/
#define AMK_INVERTER_3_ACTUAL_VALUES_1 		0x287
#define AMK_INVERTER_3_ACTUAL_VALUES_2		0x289
#define AMK_INVERTER_3_SETPOINTS_1			0x188

/*INVERTER4*/
#define AMK_INVERTER_4_ACTUAL_VALUES_1 		0x288
#define AMK_INVERTER_4_ACTUAL_VALUES_2		0x290
#define AMK_INVERTER_4_SETPOINTS_1			0x189

/*AMK_CONTROL*/
#define SET_NULL 						    0x0
#define AMK_INVERTER_ON         			(1 << 7)     		// 1xxxxxxx Controller enable
#define AMK_DC_ON							(1 << 6)			// x1xxxxxx High Voltage activation
#define AMK_DRIVER_ENABLE					(1 << 5)			// xx1xxxxx Driver Enable
#define AMK_ERROR_SET_ON					(1 << 4)			// xxx1xxxx Remove error
#define AMK_ERROR_SET_OFF                   (~(1 << 4))         // xxx0xxxx

/*PARAMS*/
#define ID110                               107

#endif /* INC_AMKFSE_H_ */