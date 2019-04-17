#ifndef _COMPASS_H
#define _COMPASS_H

#include <Arduino.h>
#include <math.h>
#include <Wire.h>


/*register mappings taken from pg6 
https://cdn-shop.adafruit.com/datasheets/LSM303DLHC.PDF */

typedef enum {

	/* 0x00 - 0x1F reservered*/
	CTRL_REG1_A = 0x20,
	CTRL_REG2_A = 0x21,
	CTRL_REG3_A = 0x22,
	CTRL_REG4_A = 0x23,
	CTRL_REG5_A = 0x24,
	HP_FILTER_RESET_A = 0x25,
	REFERENCE_A = 0x26,
	/*linear acceleration*/
	STATUS_REG_A = 0x27,

	OUT_X_L_A = 0x28,
	OUT_X_H_A = 0x29,
	OUT_Y_L_A = 0x2A,
	OUT_Y_H_A = 0x2B,
	OUT_Z_L_A = 0x2C,
	OUT_Z_H_A = 0x2D,

	INT1_CFG_A = 0x30,
	INT1_SOURCE_A = 0x31,
	INT1_THS_A = 0x32,
	INT1_DURATION_A = 0x33,
	CRA_REG_M = 0x00,
	CRB_REG_M = 0x01,
	MR_REG_M = 0x02,
	OUT_X_H_M = 0x03,
	OUT_X_L_M = 0x04,
	OUT_Y_H_M = 0x05,
	OUT_Y_L_M = 0x06,
	OUT_Z_H_M = 0x07,
	OUT_Z_L_M = 0x08,
	SR_REG_M = 0x09,
	IRA_REG_M = 0x0A,
	IRB_REG_M = 0x0B,
	IRC_REG_M = 0x0C


} registers;


typedef struct accell {

	int accel_value[6];
	int x;
	int y;
	int z;

} accell;

						/*sens can be 2 4 8 16*/
const accell* init_accel(int sensitivity){
	/*27 (50MHZ) + xyz axis*/
	Write( 0x27 ,CTRL_REG1_A,);
	if (sensitivity != 2)
		Write((0x00  | (sensitivity - sensitivity/2-1) << 4) , CTRL_REG4_A);
	else
		Write(0x00 , CTRL_REG4_A);
	Write(0x00 , MR_REG_M); /*set continous mode on mag sensor*/
	Write(0x14 , CRA_REG_M); /*go from 15 to 30*/

}

/*reads accel data */
void read_accel(const accell* ac){

	/*read accel values for x,y,z*/
	ac->accel_value[0] = read_addr(0x18 , OUT_X_L_A);
	ac->accel_value[1] = read_addr(0x18 , OUT_Y_L_A);
	ac->accel_value[2] = read_addr(0x18 , OUT_Z_L_A);

	ac->accel_value[3] = read_addr(0x18 , OUT_X_H_A);
	ac->accel_value[4] = read_addr(0x18 , OUT_Y_H_A);
	ac->accel_value[5] = read_addr(0x18 , OUT_Z_H_A);

	ac.x = (int) (ac->accel_value[0] << 8) + ac->accel_value[3];
	ac.y = (int) (ac->accel_value[1] << 8) + ac->accel_value[4];
	ac.z = (int) (ac->accel_value[2] << 8) + ac->accel_value[5];
}

static byte read_addr(byte address){
	byte b;
											/*accel value                 magnetic*/
	(address >= 20) ? wire.beginTransmission(0x18) : wire.beginTransmission(0x1E);
	Wire.write(address);
	(address >= 20) ? wire.beginTransmission(0x18,1) : wire.beginTransmission(0x1E,1);
	while(!Wire.available())
		;
	b = Wire.read();
	Wire.endTransmission();
	return b;
}	

/*get raw angle */
float raw_angle(){



}



#endif