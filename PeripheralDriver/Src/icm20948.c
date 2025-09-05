#include <string.h>
#include "icm20948.h"
#include <stdlib.h>

static uint8_t buf[20];
static uint32_t time_elapsed = 0x00;
static uint32_t prev_time_elapsed = 0x00;
static uint32_t time_difference = 0x00;static uint16_t Temp_Data = 0x0000;
static int16_t gyro_error = 717;
static double gyro_sensitivityScaleFactor = 125.199262;
//static int16_t gyro_error = 720;
//static double gyro_sensitivityScaleFactor = 131;
static I2C_HandleTypeDef* hi2c1_Addr;

int IMU_Init(I2C_HandleTypeDef* hi2c_Addr){
	int position = 0;
	int ret = 0;

	hi2c1_Addr = hi2c_Addr;

	position++;
	buf[0] = REG_BANK_SEL_0;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, REG_BANK_SEL, 1, buf, 1, 5);
	if (ret != HAL_OK){
		return position;
	}

	position++;
	buf[0] = USER_CTRL_VAL; // TODO Reset SRAM and DMP (check what happens here)
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, USER_CTRL, 1, buf, 1, 5);
	if (ret != HAL_OK){
		return position;
	}

	position++;
	buf[0] = PWR_MGMT_1_VAL;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, PWR_MGMT_1, 1, buf, 1, 5);
	if (ret != HAL_OK){
		return position;
	}

	position++;
	buf[0] = PWR_MGMT_2_VAL;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, PWR_MGMT_2, 1, buf, 1, 5);
	if (ret != HAL_OK){
		return position;
	}


	//further explanation: I2C Master in Duty Cycled Mode: Enabling the I2C master in duty cycled mode means that
	//the master device may periodically turn off its I2C interface to save power.
	//It will only enable the I2C interface when it needs to communicate with a slave device.
	//During the periods when the master's I2C interface is off, it won't be able to communicate with any slave devices.

	//Slave with Duty Cycle Disabled: Disabling the duty cycle for the slave generally means that the
	//slave device is always ready to receive data when addressed by the master.
	//In this context, "working" for the slave means it's ready to respond to requests from the master
	//as soon as it's addressed.

	position++;
	buf[0] = LP_CONFIG_VAL;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, LP_CONFIG, 1, buf, 1, 5);
	if (ret != HAL_OK){
		return position;
	}


	position++;
	buf[0] = 0x00;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, USER_CTRL, 1, buf, 1, 5);
	if (ret != HAL_OK){
		return position;
	}

	buf[0] = REG_BANK_SEL_1;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, REG_BANK_SEL, 1, buf, 1, 5);
		if (ret != HAL_OK){
		return position++;
	}

	position++;
	buf[0] = REG_BANK_SEL_2;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, REG_BANK_SEL, 1, buf, 1, 5);
	if (ret != HAL_OK){
		return position;
	}

	position++;
	buf[0] = ODR_ALIGN_EN_VAL;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, ODR_ALIGN_EN, 1, buf, 1, 5);
	if (ret != HAL_OK){
		return position;
	}


	position++;
	buf[0] = ZG_OFFS_USRH_VAL;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, ZG_OFFS_USRH, 1, buf, 1, 5);
	if (ret != HAL_OK){
		return position;
	}

	position++;
	buf[0] = ZG_OFFS_USRL_VAL;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, ZG_OFFS_USRL, 1, buf, 1, 5);
	if (ret != HAL_OK){
		return position;
	}

	position++;
	buf[0] = GYRO_CONFIG_1_VAL;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, GYRO_CONFIG_1, 1, buf, 1, 5);
	if (ret != HAL_OK){
		return position;
	}

	position++;
	buf[0] = GYRO_CONFIG_2_VAL;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, GYRO_CONFIG_2, 1, buf, 1, 5);
	if (ret != HAL_OK){
		return position;
	}

	//TODO check if this is even necessary

	position++;
	buf[0] = REG_BANK_SEL_0;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, REG_BANK_SEL, 1, buf, 1, 5);
	if (ret != HAL_OK){
		return position;
	}

	position++;
	buf[0] = REG_BANK_SEL_2;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, REG_BANK_SEL, 1, buf, 1, 5);
	if (ret != HAL_OK){
		return position;
	}

	position++;
	buf[0] = GYRO_SMPLRT_DIV_VAL;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, GYRO_SMPLRT_DIV, 1, buf, 1, 5);
	if (ret != HAL_OK){
		return position;
	}

	position++;
	//return to bank 0
	buf[0] = REG_BANK_SEL_0;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, REG_BANK_SEL, 1, buf, 1, 5);
	if (ret != HAL_OK){
		return position;
	}

	//last position 18
	return ++position;

}

int IMU_Read(int16_t* Data){
	int ret = 0;
	int position = 0;
	//TODO take a look, maybe replace this logic of using HAL_getTick, because the tick counter may reset, causing values to be wrong
	time_elapsed = HAL_GetTick();
	time_difference = time_elapsed - prev_time_elapsed;

	position++;
	buf[0] = 0;
	ret = HAL_I2C_Mem_Read(hi2c1_Addr, ICM20948_ADDR, GYRO_ZOUT_H, 1, buf, 1, 5);
	if (ret != HAL_OK)
	{
		return position;
	}

	position++;
	Temp_Data = buf[0] << 8;
	ret = HAL_I2C_Mem_Read(hi2c1_Addr, ICM20948_ADDR, GYRO_ZOUT_L, 1, buf, 1, 5);
	if (ret != HAL_OK)
	{
		return position;
	}
	//GYRO z axis is being written on data[5]. Our IMU_data in main file will have this reading.
	Data[4] = (Temp_Data | buf[0]);
	Data[5] =  (int16_t)(int)((double)(Data[4] - gyro_error)/gyro_sensitivityScaleFactor);
	if(abs(Data[5])<2){
		Data[6] = 0;
 	}
	else{
		Data[6] = (int16_t)(time_difference*Data[5]);
	}
	prev_time_elapsed = time_elapsed;

	//last position 3
	return ++position;
}

//TODO i dont understand this part
int Magnetometer_Read(uint8_t reg, uint8_t* Data){
	int ret = 0;
	int position = 0;


	position++;
	buf[0] = USER_BANK_3;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, REG_BANK_SEL, 1, buf, 1, 5);
	if (ret != HAL_OK)
	{
		return position;
	}

	position++;
	buf[0] = 0x0C | 0x80;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, 0x03, 1, buf, 1, 5);
	if (ret != HAL_OK)
	{
		return position;
	}

	position++;
	buf[0] = reg;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, 0x04, 1, buf, 1, 5);
	if (ret != HAL_OK)
	{
		return position;
	}

	position++;
	buf[0] = 0xff;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, 0x06, 1, buf, 1, 5);
	if (ret != HAL_OK)
	{
		return position;
	}

	position++;
	buf[0] = 0x00;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, 0x7F, 1, buf, 1, 5);
	if (ret != HAL_OK)
	{
		return position;
	}

	position++;
	ret = HAL_I2C_Mem_Read(hi2c1_Addr, ICM20948_ADDR, 0x3B, 1, Data, 1, 5);
	if (ret != HAL_OK)
	{
		return position;
	}


	//last position 7
	return ++position;

}

int Magnetometer_Write(uint8_t reg, uint8_t value){
	int ret = 0;
	int position = 0;

	position++;
	buf[0] = 0x30;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, REG_BANK_SEL, 1, buf, 1, 5);
	if (ret != HAL_OK)
	{
		return position;
	}


	position++;
	buf[0] = 0x0C;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, 0x03, 1, buf, 1, 5);
	if (ret != HAL_OK)
	{
		return position;
	}

	position++;
	buf[0] = reg;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, 0x04, 1, buf, 1, 5);
	if (ret != HAL_OK)
	{
		return position;
	}

	position++;
	buf[0] = value;
	ret = HAL_I2C_Mem_Write(hi2c1_Addr, ICM20948_ADDR, 0x06, 1, buf, 1, 5);
	if (ret != HAL_OK)
	{
		return position;
	}

	//last position 5
	return ++position;

}





